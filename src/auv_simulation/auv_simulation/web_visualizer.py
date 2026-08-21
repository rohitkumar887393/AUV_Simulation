#!/usr/bin/env python3

import os
import sys
import json
import math
import time
import struct
import base64
import hashlib
import threading
from socketserver import ThreadingMixIn
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String, Bool
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from auv_interfaces.msg import Odometry, Orientation, ActuatorCmds, Mission

# Shared telemetry state dictionary
simulation_state = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.5},
    "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "velocities": {"vx": 0.0, "vy": 0.0, "vz": 0.0},
    "accel": {"ax": 0.0, "ay": 0.0, "az": -9.81},
    "actuators": {
        "elevator_left": 1500,
        "elevator_right": 1500,
        "rudder_left": 1500,
        "rudder_right": 1500,
        "main_thruster": 1500
    },
    "actual_actuators": {
        "elevator_left": 1500,
        "elevator_right": 1500,
        "rudder_left": 1500,
        "rudder_right": 1500,
        "main_thruster": 1500
    },
    "status": {
        "armed": False,
        "gain": 75,
        "mode": "MANUAL",
        "heading_hold": "OFF",
        "depth_hold": "OFF",
        "distance_hold": "OFF",
        "roll_stabilizer": "ACTIVE",
        "mission": "IDLE"
    },
    "autonomy": {
        "target_depth": 0.0,
        "current_depth": 0.5,
        "depth_error": 0.0,
        "depth_cmd": 0.0,
        "target_heading": 0.0,
        "current_heading": 0.0,
        "heading_error": 0.0,
        "rudder_cmd": 0.0,
        "target_distance": 0.0,
        "current_distance": 0.0,
        "distance_remaining": 0.0,
        "throttle_cmd": 0.0,
        "active_depth_target": 0.0,
        "active_heading_target": 0.0
    },
    "physics": {
        "u": 0.0,
        "q_dyn": 0.0,
        "m_pitch": 0.0,
        "m_restore": 0.0,
        "n_yaw": 0.0,
        "l_roll": 0.0,
        "vz_world": 0.0
    },
    "packets": 0,
    "world_motion": "NONE"
}

ros_node_instance = None

HTML_CONTENT = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AUV Tactical 3D HUD & Closed-Loop Control Visualizer</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&display=swap');

        * {
            box-sizing: border-box;
            user-select: none;
        }

        body {
            margin: 0;
            padding: 0;
            overflow: hidden;
            background-color: #030a16;
            font-family: 'Share Tech Mono', monospace;
            color: #00f5d4;
        }

        #canvas-container {
            width: 100vw;
            height: 100vh;
            position: absolute;
            top: 0;
            left: 0;
            z-index: 1;
        }

        /* Custom Scrollbar for HUD panels */
        ::-webkit-scrollbar {
            width: 4px;
            height: 4px;
        }
        ::-webkit-scrollbar-track {
            background: rgba(3, 14, 28, 0.6);
            border-radius: 2px;
        }
        ::-webkit-scrollbar-thumb {
            background: #00a8e8;
            border-radius: 2px;
        }
        ::-webkit-scrollbar-thumb:hover {
            background: #00f5d4;
        }

        /* Top HUD Bar */
        #top-hud-bar {
            position: absolute;
            top: 0;
            left: 0;
            width: 100vw;
            height: 42px;
            background: rgba(3, 14, 28, 0.96);
            border-bottom: 1.5px solid #00a8e8;
            box-shadow: 0 2px 15px rgba(0, 168, 232, 0.25);
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 0 16px;
            z-index: 20;
            font-size: 12px;
            letter-spacing: 0.8px;
        }

        .hud-group-left {
            display: flex;
            align-items: center;
            gap: 10px;
        }

        .hud-group-right {
            display: flex;
            align-items: center;
            gap: 8px;
        }

        #header-title {
            position: absolute;
            left: 50%;
            transform: translateX(-50%);
            font-size: 18px;
            font-weight: 800;
            letter-spacing: 2px;
            color: #ffffff;
            text-shadow: 0 0 12px rgba(0, 245, 212, 0.7), 0 0 24px rgba(0, 168, 232, 0.4);
            white-space: nowrap;
            pointer-events: none;
        }

        .hud-pill {
            display: inline-flex;
            align-items: center;
            gap: 7px;
            padding: 3px 9px;
            border-radius: 4px;
            border: 1px solid #00a8e8;
            background: rgba(10, 25, 47, 0.8);
            font-size: 11px;
        }

        .dot-indicator {
            width: 7px;
            height: 7px;
            border-radius: 50%;
            display: inline-block;
        }

        .dot-disarmed { background: #d90429; box-shadow: 0 0 8px #d90429; }
        .dot-armed { background: #00f5d4; box-shadow: 0 0 10px #00f5d4; }

        .hud-divider {
            color: #1c3d5a;
            font-weight: bold;
        }

        .gain-bar-container {
            width: 55px;
            height: 7px;
            background: #0d2238;
            border-radius: 4px;
            overflow: hidden;
            border: 1px solid #00a8e8;
        }

        .gain-bar-fill {
            height: 100%;
            width: 75%;
            background: #00f5d4;
            box-shadow: 0 0 8px #00f5d4;
        }

        .badge-box {
            padding: 3px 8px;
            border-radius: 4px;
            border: 1px solid #00a8e8;
            background: rgba(0, 168, 232, 0.15);
            color: #00f5d4;
            font-weight: bold;
            font-size: 11px;
            letter-spacing: 0.5px;
        }

        .badge-disabled {
            border-color: #3a506b;
            color: #4a6572;
            background: rgba(13, 27, 42, 0.5);
        }

        .badge-enabled {
            border-color: #00f5d4;
            color: #00f5d4;
            background: rgba(0, 245, 212, 0.2);
            box-shadow: 0 0 8px rgba(0, 245, 212, 0.35);
        }

        /* Left Sidebar Container */
        #left-sidebar {
            position: absolute;
            top: 50px;
            left: 14px;
            width: 255px;
            max-height: calc(100vh - 58px);
            overflow-y: auto;
            overflow-x: hidden;
            display: flex;
            flex-direction: column;
            gap: 7px;
            z-index: 10;
            padding-right: 2px;
        }

        .hud-card {
            background: rgba(3, 14, 28, 0.94);
            border: 1.5px solid #00a8e8;
            border-radius: 5px;
            padding: 6px 9px;
            box-shadow: 0 0 20px rgba(0, 0, 0, 0.7);
        }

        .panel-header {
            font-size: 11px;
            color: #00f5d4;
            border-bottom: 1px solid #1c3d5a;
            padding-bottom: 3px;
            margin-bottom: 4px;
            letter-spacing: 1px;
            font-weight: bold;
            display: flex;
            justify-content: space-between;
        }

        .telem-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 2.5px;
            font-size: 11px;
            line-height: 1.25;
        }

        .telem-label { color: #8d99ae; font-size: 10.5px; }
        .telem-val { color: #ffffff; font-weight: bold; font-size: 11px; text-align: right; }
        .telem-val-highlight { color: #ffb703; font-weight: bold; font-size: 11px; text-align: right; }

        /* Left Side: Propulsion Panel */
        .pwm-bar-wrapper {
            margin-bottom: 3.5px;
        }

        .pwm-label-row {
            display: flex;
            justify-content: space-between;
            font-size: 10px;
            margin-bottom: 2px;
            font-weight: bold;
        }

        .pwm-track {
            height: 6px;
            background: #0d2238;
            border-radius: 3px;
            overflow: hidden;
            border: 1px solid #1c3d5a;
        }

        .pwm-fill {
            height: 100%;
            width: 50%;
            background: #00f5d4;
            box-shadow: 0 0 6px #00f5d4;
        }

        /* Left Side: Physics Debug Data Panel */
        #physics-debug-panel {
            border-color: #00a8e8;
            font-size: 10.5px;
        }

        #physics-debug-panel .panel-header {
            color: #ffb703;
            border-bottom-color: rgba(255, 183, 3, 0.4);
        }

        /* Top-Right Compass */
        #compass-container {
            position: absolute;
            top: 48px;
            right: 14px;
            width: 125px;
            height: 125px;
            z-index: 15;
            filter: drop-shadow(0 0 12px rgba(0, 245, 212, 0.2));
            pointer-events: none;
        }

        /* Right Sidebar Container (Bottom Right) */
        #right-sidebar {
            position: absolute;
            top: 178px;
            right: 14px;
            width: 265px;
            max-height: calc(100vh - 186px);
            overflow-y: auto;
            overflow-x: hidden;
            display: flex;
            flex-direction: column;
            gap: 6px;
            z-index: 10;
            padding-right: 2px;
        }

        /* Camera Controls (Bottom Right) */
        #cam-mode-box {
            background: rgba(3, 14, 28, 0.94);
            border: 1.5px solid #00a8e8;
            border-radius: 5px;
            padding: 5px 6px;
            display: flex;
            justify-content: space-between;
            gap: 4px;
            box-shadow: 0 0 15px rgba(0, 0, 0, 0.6);
        }

        .cam-btn {
            flex: 1;
            background: rgba(0, 119, 182, 0.35);
            color: #00f5d4;
            border: 1px solid #00a8e8;
            padding: 4px 2px;
            border-radius: 3px;
            cursor: pointer;
            font-family: inherit;
            font-size: 10.5px;
            font-weight: bold;
            text-align: center;
            transition: all 0.15s;
            white-space: nowrap;
        }

        .cam-btn:hover {
            background: #00b4d8;
            color: #030a16;
            box-shadow: 0 0 8px #00b4d8;
        }

        /* Unified Mission & Control Telemetry Panel */
        #unified-telemetry-panel {
            border-color: #00a8e8;
            padding: 6px 9px;
        }

        #unified-telemetry-panel .panel-header {
            font-size: 11px;
            color: #00f5d4;
            border-bottom: 1px solid #1c3d5a;
            padding-bottom: 3px;
            margin-bottom: 3px;
            letter-spacing: 1px;
            font-weight: bold;
        }

        #unified-telemetry-panel .hud-section-header {
            color: #00f5d4;
            font-size: 10.5px;
            font-weight: bold;
            margin-top: 3.5px;
            margin-bottom: 1px;
            letter-spacing: 0.8px;
        }

        #unified-telemetry-panel .hud-divider-line {
            border-bottom: 1px solid #1c3d5a;
            margin: 3px 0;
        }

        #unified-telemetry-panel .telem-row {
            display: flex;
            justify-content: space-between;
            margin-bottom: 2px;
            font-size: 10.5px;
            line-height: 1.2;
        }

        #unified-telemetry-panel .telem-label {
            color: #8d99ae;
            font-size: 10px;
        }

        #unified-telemetry-panel .telem-val {
            color: #ffffff;
            font-weight: bold;
            font-size: 10.5px;
            text-align: right;
        }

        #unified-telemetry-panel .telem-val-highlight {
            color: #ffb703;
            font-weight: bold;
            font-size: 10.5px;
            text-align: right;
        }

        /* Mission Control Panel */
        #mission-panel {
            border-color: #00a8e8;
            padding: 6px 9px;
        }

        #mission-panel .panel-header {
            font-size: 11px;
            padding-bottom: 3px;
            margin-bottom: 4px;
        }

        .mission-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 3px;
            font-size: 10.5px;
            font-weight: bold;
        }

        .mission-input {
            width: 60px;
            height: 19px;
            background: #0a192f;
            border: 1px solid #00a8e8;
            color: #00f5d4;
            padding: 0 4px;
            border-radius: 3px;
            font-family: inherit;
            font-size: 10.5px;
            font-weight: bold;
            text-align: right;
        }

        .mission-btn-row {
            display: flex;
            gap: 4px;
            margin-top: 4px;
            margin-bottom: 4px;
        }

        .mission-btn {
            padding: 4px 3px;
            border-radius: 3px;
            font-family: inherit;
            font-size: 10px;
            font-weight: bold;
            cursor: pointer;
            border: 1px solid transparent;
            transition: all 0.15s;
        }

        .btn-start {
            flex: 1.8;
            background: rgba(0, 245, 212, 0.25);
            color: #00f5d4;
            border-color: #00f5d4;
        }
        .btn-start:hover {
            background: #00f5d4;
            color: #030a16;
            box-shadow: 0 0 10px #00f5d4;
        }

        .btn-stop {
            flex: 1.2;
            background: rgba(217, 4, 41, 0.25);
            color: #ef233c;
            border-color: #d90429;
        }
        .btn-stop:hover {
            background: #d90429;
            color: #ffffff;
            box-shadow: 0 0 10px #d90429;
        }

        .mission-status-box {
            background: rgba(10, 25, 47, 0.9);
            border: 1px solid #1c3d5a;
            border-radius: 3px;
            padding: 3px 4px;
            text-align: center;
            margin-top: 3px;
        }

        .mission-status-title {
            font-size: 8.5px;
            color: #8d99ae;
            letter-spacing: 0.8px;
            font-weight: bold;
            margin-bottom: 0px;
        }

        .mission-status-val {
            font-size: 12px;
            font-weight: bold;
            letter-spacing: 1px;
            color: #00f5d4;
            text-shadow: 0 0 6px rgba(0, 245, 212, 0.4);
        }

        @media (max-width: 1200px) {
            #left-sidebar { width: 240px; }
            #right-sidebar { width: 250px; }
            #header-title { font-size: 15px; }
        }
        @media (max-height: 768px) {
            #top-hud-bar { height: 36px; font-size: 11px; }
            #left-sidebar { top: 42px; gap: 4px; }
            #compass-container { top: 42px; width: 115px; height: 115px; }
            #right-sidebar { top: 165px; }
            .hud-card { padding: 4px 7px; }
        }
    </style>
    <!-- Three.js & OrbitControls -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
</head>
<body>
    <div id="canvas-container"></div>

    <!-- Top HUD Status Bar -->
    <div id="top-hud-bar">
        <div class="hud-group-left">
            <div class="hud-pill" onclick="toggleArm()" style="cursor: pointer;" title="Click to Arm / Disarm (or Spacebar)">
                <span class="dot-indicator dot-disarmed" id="dot-armed"></span>
                <span id="txt-armed" style="font-weight: bold;">DISARMED</span>
            </div>

            <span class="hud-divider">|</span>

            <span>GAIN</span>
            <div class="gain-bar-container">
                <div class="gain-bar-fill" id="bar-gain"></div>
            </div>
            <span id="txt-gain">75%</span>

            <span class="hud-divider">|</span>

            <div class="badge-box" id="badge-mode">MANUAL</div>
        </div>

        <div id="header-title">CORATIA AUV SIMULATOR</div>

        <div class="hud-group-right">
            <div class="badge-box badge-disabled" id="badge-depth-hold">DEPTH HOLD: OFF</div>
            <div class="badge-box badge-disabled" id="badge-heading-hold">HEADING HOLD: OFF</div>
            <div class="badge-box badge-enabled" id="badge-roll-stab">ROLL STABILIZER: ACTIVE</div>
            <div class="badge-box badge-disabled" id="badge-joy">JOYSTICK: DISCONNECTED</div>
        </div>
    </div>

    <!-- Left Panels Sidebar -->
    <div id="left-sidebar">
        <!-- Telemetry Data Panel -->
        <div id="telemetry-panel" class="hud-card">
            <div class="panel-header">
                <span>TELEMETRY DATA</span>
                <span style="font-size: 10px; color: #8d99ae;" id="txt-packets">0 pkts</span>
            </div>

            <div>
                <div class="telem-row"><span class="telem-label">ROLL:</span> <span class="telem-val" id="val-roll">+0.0°</span></div>
                <div class="telem-row"><span class="telem-label">PITCH:</span> <span class="telem-val" id="val-pitch">+0.0°</span></div>
                <div class="telem-row"><span class="telem-label">YAW:</span> <span class="telem-val-highlight" id="val-yaw">+0.0°</span></div>

                <div style="height: 3px;"></div>

                <div class="telem-row"><span class="telem-label">SPEED X:</span> <span class="telem-val" id="val-vx">+0.00 m/s</span></div>
                <div class="telem-row"><span class="telem-label">SPEED Y:</span> <span class="telem-val" id="val-vy">+0.00 m/s</span></div>
                <div class="telem-row"><span class="telem-label">SPEED Z:</span> <span class="telem-val" id="val-vz">+0.00 m/s</span></div>

                <div style="height: 3px;"></div>

                <div class="telem-row"><span class="telem-label">DEPTH:</span> <span class="telem-val-highlight" id="val-depth">0.50 m</span></div>
                <div class="telem-row"><span class="telem-label">LIN VELOCITY:</span> <span class="telem-val" id="val-lin-vel">[+0.0, +0.0, +0.0] m/s</span></div>
                <div class="telem-row"><span class="telem-label">IMU ACCEL:</span> <span class="telem-val" id="val-imu-accel">[+0.0, +0.0, -9.8] m/s²</span></div>
                <div class="telem-row"><span class="telem-label">WORLD MOTION:</span> <span class="telem-val-highlight" id="val-motion">NONE</span></div>
            </div>
        </div>

        <!-- Propulsion Panel -->
        <div id="propulsion-panel" class="hud-card">
            <div class="panel-header">PROPULSION & CONTROL SURFACES</div>

            <div class="pwm-bar-wrapper">
                <div class="pwm-label-row">
                    <span class="telem-label">MAIN THRUSTER PWM:</span>
                    <span class="telem-val" id="val-thr-pwm">1500 us (0%)</span>
                </div>
                <div class="pwm-track"><div class="pwm-fill" id="fill-thr-pwm"></div></div>
            </div>

            <div class="pwm-bar-wrapper">
                <div class="pwm-label-row">
                    <span class="telem-label">ELEVATOR PWM:</span>
                    <span class="telem-val" id="val-elev-pwm">1500 us (0%)</span>
                </div>
                <div class="pwm-track"><div class="pwm-fill" id="fill-elev-pwm"></div></div>
            </div>

            <div class="pwm-bar-wrapper">
                <div class="pwm-label-row">
                    <span class="telem-label">RUDDER PWM:</span>
                    <span class="telem-val" id="val-rud-pwm">1500 us (0%)</span>
                </div>
                <div class="pwm-track"><div class="pwm-fill" id="fill-rud-pwm"></div></div>
            </div>
        </div>

        <!-- Physics Debug Data Panel -->
        <div id="physics-debug-panel" class="hud-card">
            <div class="panel-header">PHYSICS DEBUG TELEMETRY</div>
            <div class="telem-row"><span class="telem-label">BUOYANCY FORCE:</span> <span class="telem-val-highlight" id="phys-buoyancy">+12.3 N</span></div>
            <div class="telem-row"><span class="telem-label">THRUST FORCE:</span> <span class="telem-val" id="phys-thrust">+0.0 N</span></div>
            <div class="telem-row"><span class="telem-label">FORWARD DRAG:</span> <span class="telem-val" id="phys-drag">-0.0 N</span></div>
            <div class="telem-row"><span class="telem-label">DYNAMIC PRESS:</span> <span class="telem-val-highlight" id="phys-qdyn">0.0 Pa</span></div>
            <div class="telem-row"><span class="telem-label">PITCH MOMENT:</span> <span class="telem-val" id="phys-mpitch">0.00 Nm</span></div>
            <div class="telem-row"><span class="telem-label">YAW MOMENT:</span> <span class="telem-val" id="phys-nyaw">0.00 Nm</span></div>
            <div class="telem-row"><span class="telem-label">ROLL MOMENT:</span> <span class="telem-val" id="phys-lroll">0.00 Nm</span></div>
        </div>

        <!-- Mission Control Panel -->
        <div id="mission-panel" class="hud-card">
            <div class="panel-header">MISSION CONTROL</div>

            <div class="mission-row">
                <span class="telem-label">TARGET DEPTH:</span>
                <span><input type="number" id="input-depth" step="0.5" value="20.00" class="mission-input"> m</span>
            </div>
            <div class="mission-row">
                <span class="telem-label">TARGET SPEED:</span>
                <span><input type="number" id="input-speed" step="0.05" value="1.00" class="mission-input"> m/s</span>
            </div>
            <div class="mission-row">
                <span class="telem-label">TARGET HEADING:</span>
                <span><input type="number" id="input-heading" step="1.0" value="90.0" class="mission-input"> deg</span>
            </div>
            <div class="mission-row">
                <span class="telem-label">MISSION TIME:</span>
                <span><input type="number" id="input-duration" step="1" value="30" class="mission-input"> min</span>
            </div>

            <div class="mission-btn-row">
                <button class="mission-btn btn-start" onclick="startMission()">START MISSION</button>
                <button class="mission-btn btn-stop" onclick="stopMission()">STOP MISSION</button>
            </div>

            <div class="mission-status-box">
                <div class="mission-status-title">MISSION STATUS</div>
                <div id="txt-mission-status" class="mission-status-val">IDLE</div>
            </div>
        </div>
    </div>

    <!-- Compass Widget - Top Right -->
    <div id="compass-container">
        <svg id="compass-svg" viewBox="0 0 140 140" width="125" height="125">
            <!-- Background Outer Ring -->
            <circle cx="70" cy="70" r="68" fill="rgba(3, 14, 28, 0.94)" stroke="#1c3d5a" stroke-width="2" />
            <circle cx="70" cy="70" r="66" fill="none" stroke="#00a8e8" stroke-width="1" opacity="0.6" />

            <!-- Rotating Compass Dial Group -->
            <g id="compass-dial" style="transform-origin: 70px 70px;">
                <!-- Degree Ticks & Numbers -->
                <!-- 0 N -->
                <line x1="70" y1="4" x2="70" y2="12" stroke="#d90429" stroke-width="2" />
                <text x="70" y="24" fill="#d90429" font-size="13" font-weight="bold" font-family="'Share Tech Mono', monospace" text-anchor="middle">N</text>
                
                <!-- 30 -->
                <line x1="103" y1="13" x2="99" y2="20" stroke="#ffffff" stroke-width="1.2" />
                <text x="96" y="29" fill="#ffffff" font-size="8" font-family="'Share Tech Mono', monospace" text-anchor="middle">30</text>
                
                <!-- 60 -->
                <line x1="127" y1="37" x2="120" y2="41" stroke="#ffffff" stroke-width="1.2" />
                <text x="115" y="47" fill="#ffffff" font-size="8" font-family="'Share Tech Mono', monospace" text-anchor="middle">60</text>

                <!-- 90 E -->
                <line x1="136" y1="70" x2="128" y2="70" stroke="#00f5d4" stroke-width="2" />
                <text x="122" y="74" fill="#00f5d4" font-size="12" font-weight="bold" font-family="'Share Tech Mono', monospace" text-anchor="middle">E</text>
                <text x="108" y="73" fill="#ffffff" font-size="7.5" font-family="'Share Tech Mono', monospace" text-anchor="middle">90</text>

                <!-- 120 -->
                <line x1="127" y1="103" x2="120" y2="99" stroke="#ffffff" stroke-width="1.2" />
                <text x="115" y="99" fill="#ffffff" font-size="8" font-family="'Share Tech Mono', monospace" text-anchor="middle">120</text>

                <!-- 150 -->
                <line x1="103" y1="127" x2="99" y2="120" stroke="#ffffff" stroke-width="1.2" />
                <text x="96" y="117" fill="#ffffff" font-size="8" font-family="'Share Tech Mono', monospace" text-anchor="middle">150</text>

                <!-- 180 S -->
                <line x1="70" y1="136" x2="70" y2="128" stroke="#00f5d4" stroke-width="2" />
                <text x="70" y="122" fill="#00f5d4" font-size="12" font-weight="bold" font-family="'Share Tech Mono', monospace" text-anchor="middle">S</text>
                <text x="70" y="108" fill="#ffffff" font-size="7.5" font-family="'Share Tech Mono', monospace" text-anchor="middle">180</text>

                <!-- 210 -->
                <line x1="37" y1="127" x2="41" y2="120" stroke="#ffffff" stroke-width="1.2" />
                <text x="44" y="117" fill="#ffffff" font-size="8" font-family="'Share Tech Mono', monospace" text-anchor="middle">210</text>

                <!-- 240 -->
                <line x1="13" y1="103" x2="20" y2="99" stroke="#ffffff" stroke-width="1.2" />
                <text x="25" y="99" fill="#ffffff" font-size="8" font-family="'Share Tech Mono', monospace" text-anchor="middle">240</text>

                <!-- 270 W -->
                <line x1="4" y1="70" x2="12" y2="70" stroke="#00f5d4" stroke-width="2" />
                <text x="18" y="74" fill="#00f5d4" font-size="12" font-weight="bold" font-family="'Share Tech Mono', monospace" text-anchor="middle">W</text>
                <text x="32" y="73" fill="#ffffff" font-size="7.5" font-family="'Share Tech Mono', monospace" text-anchor="middle">270</text>

                <!-- 300 -->
                <line x1="13" y1="37" x2="20" y2="41" stroke="#ffffff" stroke-width="1.2" />
                <text x="25" y="47" fill="#ffffff" font-size="8" font-family="'Share Tech Mono', monospace" text-anchor="middle">300</text>

                <!-- 330 -->
                <line x1="37" y1="13" x2="41" y2="20" stroke="#ffffff" stroke-width="1.2" />
                <text x="44" y="29" fill="#ffffff" font-size="8" font-family="'Share Tech Mono', monospace" text-anchor="middle">330</text>

                <!-- Intermediate 10-deg tick marks -->
                <!-- 10 --> <line x1="81" y1="5" x2="80" y2="9" stroke="#8d99ae" stroke-width="1" />
                <!-- 20 --> <line x1="93" y1="9" x2="91" y2="13" stroke="#8d99ae" stroke-width="1" />
                <!-- 40 --> <line x1="112" y1="21" x2="109" y2="24" stroke="#8d99ae" stroke-width="1" />
                <!-- 50 --> <line x1="121" y1="28" x2="117" y2="32" stroke="#8d99ae" stroke-width="1" />
                <!-- 70 --> <line x1="132" y1="47" x2="128" y2="49" stroke="#8d99ae" stroke-width="1" />
                <!-- 80 --> <line x1="135" y1="59" x2="131" y2="60" stroke="#8d99ae" stroke-width="1" />
                <!-- 100 --> <line x1="135" y1="81" x2="131" y2="80" stroke="#8d99ae" stroke-width="1" />
                <!-- 110 --> <line x1="132" y1="93" x2="128" y2="91" stroke="#8d99ae" stroke-width="1" />
                <!-- 130 --> <line x1="121" y1="112" x2="117" y2="108" stroke="#8d99ae" stroke-width="1" />
                <!-- 140 --> <line x1="112" y1="119" x2="109" y2="116" stroke="#8d99ae" stroke-width="1" />
                <!-- 160 --> <line x1="93" y1="131" x2="91" y2="127" stroke="#8d99ae" stroke-width="1" />
                <!-- 170 --> <line x1="81" y1="135" x2="80" y2="131" stroke="#8d99ae" stroke-width="1" />
                <!-- 190 --> <line x1="59" y1="135" x2="60" y2="131" stroke="#8d99ae" stroke-width="1" />
                <!-- 200 --> <line x1="47" y1="131" x2="49" y2="127" stroke="#8d99ae" stroke-width="1" />
                <!-- 220 --> <line x1="28" y1="119" x2="31" y2="116" stroke="#8d99ae" stroke-width="1" />
                <!-- 230 --> <line x1="19" y1="112" x2="23" y2="108" stroke="#8d99ae" stroke-width="1" />
                <!-- 250 --> <line x1="8" y1="93" x2="12" y2="91" stroke="#8d99ae" stroke-width="1" />
                <!-- 260 --> <line x1="5" y1="81" x2="9" y2="80" stroke="#8d99ae" stroke-width="1" />
                <!-- 280 --> <line x1="5" y1="59" x2="9" y2="60" stroke="#8d99ae" stroke-width="1" />
                <!-- 290 --> <line x1="8" y1="47" x2="12" y2="49" stroke="#8d99ae" stroke-width="1" />
                <!-- 310 --> <line x1="19" y1="28" x2="23" y2="32" stroke="#8d99ae" stroke-width="1" />
                <!-- 320 --> <line x1="28" y1="21" x2="31" y2="24" stroke="#8d99ae" stroke-width="1" />
                <!-- 340 --> <line x1="47" y1="9" x2="49" y2="13" stroke="#8d99ae" stroke-width="1" />
                <!-- 350 --> <line x1="59" y1="5" x2="60" y2="9" stroke="#8d99ae" stroke-width="1" />
            </g>

            <!-- Fixed Center Display -->
            <circle cx="70" cy="70" r="21" fill="#030e1c" stroke="#00a8e8" stroke-width="1.5" />
            <text id="compass-val" x="70" y="69" fill="#ffffff" font-size="11.5" font-weight="bold" font-family="'Share Tech Mono', monospace" text-anchor="middle">000°</text>
            <text x="70" y="79" fill="#8d99ae" font-size="7" font-weight="bold" font-family="'Share Tech Mono', monospace" text-anchor="middle">HDG</text>

            <!-- Fixed Top Needle Pointer -->
            <polygon points="70,11 66,3 74,3" fill="#d90429" />
            <circle cx="70" cy="70" r="2" fill="#00f5d4" />
        </svg>
    </div>

    <!-- Right Sidebar (Bottom Right) -->
    <div id="right-sidebar">
        <!-- Mission & Control Telemetry Panel -->
        <div id="unified-telemetry-panel" class="hud-card">
            <div class="panel-header">MISSION & CONTROL TELEMETRY</div>

            <div class="hud-section-header">MISSION</div>
            <div class="telem-row"><span class="telem-label">PHASE:</span> <span class="telem-val-highlight" id="ut-phase">IDLE</span></div>
            <div class="telem-row"><span class="telem-label">MISSION TIME:</span> <span class="telem-val" id="ut-time">00:00 / 30:00</span></div>

            <div class="hud-divider-line"></div>

            <div class="hud-section-header">DEPTH</div>
            <div class="telem-row"><span class="telem-label">TARGET:</span> <span class="telem-val" id="ut-depth-tgt">0.00 m</span></div>
            <div class="telem-row"><span class="telem-label">CURRENT:</span> <span class="telem-val" id="ut-depth-cur">0.50 m</span></div>
            <div class="telem-row"><span class="telem-label">ERROR:</span> <span class="telem-val-highlight" id="ut-depth-err">+0.00 m</span></div>
            <div class="telem-row"><span class="telem-label">PID OUTPUT:</span> <span class="telem-val" id="ut-depth-out">+0.0</span></div>

            <div class="hud-divider-line"></div>

            <div class="hud-section-header">HEADING</div>
            <div class="telem-row"><span class="telem-label">TARGET:</span> <span class="telem-val" id="ut-head-tgt">0.0°</span></div>
            <div class="telem-row"><span class="telem-label">CURRENT:</span> <span class="telem-val" id="ut-head-cur">0.0°</span></div>
            <div class="telem-row"><span class="telem-label">ERROR:</span> <span class="telem-val-highlight" id="ut-head-err">+0.0°</span></div>
            <div class="telem-row"><span class="telem-label">PID OUTPUT:</span> <span class="telem-val" id="ut-head-out">+0.0</span></div>

            <div class="hud-divider-line"></div>

            <div class="hud-section-header">SPEED</div>
            <div class="telem-row"><span class="telem-label">TARGET:</span> <span class="telem-val" id="ut-spd-tgt">0.00 m/s</span></div>
            <div class="telem-row"><span class="telem-label">COMMAND:</span> <span class="telem-val" id="ut-spd-cmd">0.00 m/s</span></div>
            <div class="telem-row"><span class="telem-label">ACTUAL:</span> <span class="telem-val-highlight" id="ut-spd-act">0.00 m/s</span></div>

            <div class="hud-divider-line"></div>

            <div class="hud-section-header">ACTUATOR OUTPUT</div>
            <div class="telem-row"><span class="telem-label">THRUSTER CMD:</span> <span class="telem-val" id="ut-act-thr">1500 us</span></div>
            <div class="telem-row"><span class="telem-label">ELEVATOR CMD:</span> <span class="telem-val" id="ut-act-elev">1500 us</span></div>
            <div class="telem-row"><span class="telem-label">RUDDER CMD:</span> <span class="telem-val" id="ut-act-rud">1500 us</span></div>

            <div class="hud-divider-line"></div>

            <div class="hud-section-header">PHYSICAL STATE</div>
            <div class="telem-row"><span class="telem-label">FORWARD SPEED:</span> <span class="telem-val" id="ut-phys-fwd">0.00 m/s</span></div>
            <div class="telem-row"><span class="telem-label">VERTICAL SPEED:</span> <span class="telem-val" id="ut-phys-vz">0.00 m/s</span></div>
            <div class="telem-row"><span class="telem-label">PITCH:</span> <span class="telem-val" id="ut-phys-pitch">0.0°</span></div>
            <div class="telem-row"><span class="telem-label">YAW:</span> <span class="telem-val" id="ut-phys-yaw">0.0°</span></div>
            <div class="telem-row"><span class="telem-label">ROLL:</span> <span class="telem-val" id="ut-phys-roll">0.0°</span></div>
            <div class="telem-row"><span class="telem-label">DEPTH:</span> <span class="telem-val" id="ut-phys-depth">0.50 m</span></div>
        </div>

        <!-- Camera Controls (Right Bottom) -->
        <div id="cam-mode-box">
            <button class="cam-btn" onclick="setCameraMode('orbit')">Free Orbit</button>
            <button class="cam-btn" onclick="setCameraMode('follow')">Follow AUV</button>
            <button class="cam-btn" onclick="setCameraMode('top')">Top View</button>
            <button class="cam-btn" onclick="setCameraMode('side')">Side View</button>
        </div>
    </div>

    <script>
        let scene, camera, renderer, controls;
        let auvGroup, trajectoryLine, trajectoryGeo;
        let trajectoryPoints = [];
        let maxTrajectoryPoints = 600;
        let cameraMode = 'orbit';
        let latestState = null;

        // Actuator Animation References & State
        let propellerGroup = null;
        let finTopGroup = null;
        let finBottomGroup = null;
        let finRightGroup = null;
        let finLeftGroup = null;
        let marineSnowPoints = null;

        let propRotationAngle = 0.0;
        let currentRudAngle = 0.0;
        let targetRudAngle = 0.0;
        let currentElevLAngle = 0.0;
        let targetElevLAngle = 0.0;
        let currentElevRAngle = 0.0;
        let targetElevRAngle = 0.0;
        let currentThrustNorm = 0.0;

        function init3D() {
            const container = document.getElementById('canvas-container');

            scene = new THREE.Scene();

            // 1. Realistic Oceanic Gradient Background & Soft Depth Fog
            const bgCanvas = document.createElement('canvas');
            bgCanvas.width = 2;
            bgCanvas.height = 512;
            const bgCtx = bgCanvas.getContext('2d');
            const oceanGradient = bgCtx.createLinearGradient(0, 0, 0, 512);
            oceanGradient.addColorStop(0.0, '#1a5f8a');  // Shallow / surface sunlight penetration
            oceanGradient.addColorStop(0.35, '#104566'); // Mid-depth ocean blue
            oceanGradient.addColorStop(0.70, '#0a2e47'); // Deeper marine water
            oceanGradient.addColorStop(1.0, '#041829');  // Deep abyss floor
            bgCtx.fillStyle = oceanGradient;
            bgCtx.fillRect(0, 0, 2, 512);

            const bgTexture = new THREE.CanvasTexture(bgCanvas);
            scene.background = bgTexture;
            scene.fog = new THREE.FogExp2(0x0e3d5c, 0.022); // Natural ocean scattering haze

            camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
            camera.position.set(8, 5, 8);

            renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
            renderer.setSize(window.innerWidth, window.innerHeight);
            renderer.setPixelRatio(window.devicePixelRatio);
            renderer.toneMapping = THREE.ACESFilmicToneMapping;
            renderer.toneMappingExposure = 1.05;
            container.appendChild(renderer.domElement);

            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.05;

            // 2. Realistic Underwater Lighting Rig
            // Diffuse volumetric ambient light (ocean water scatter)
            const ambientLight = new THREE.AmbientLight(0x286e96, 0.90);
            scene.add(ambientLight);

            // Primary downwelling sunlight filtered through surface
            const sunLight = new THREE.DirectionalLight(0x8fe0f5, 1.10);
            sunLight.position.set(12, 30, 8);
            scene.add(sunLight);

            // Deep upwelling bounce light from ocean depths
            const bounceLight = new THREE.DirectionalLight(0x0a2d42, 0.50);
            bounceLight.position.set(-10, -20, -10);
            scene.add(bounceLight);

            // Secondary lateral fill light
            const lateralFill = new THREE.DirectionalLight(0x195c80, 0.40);
            lateralFill.position.set(-12, 10, 15);
            scene.add(lateralFill);

            // 3. Realistic Water Surface (Y = 0m, depth = 0m, above submerged AUV)
            const surfaceGeo = new THREE.PlaneGeometry(120, 120, 16, 16);
            const surfaceMat = new THREE.MeshStandardMaterial({
                color: 0x48cae4,
                transparent: true,
                opacity: 0.10,
                roughness: 0.25,
                metalness: 0.15,
                side: THREE.DoubleSide
            });
            const surfaceMesh = new THREE.Mesh(surfaceGeo, surfaceMat);
            surfaceMesh.rotation.x = -Math.PI / 2;
            surfaceMesh.position.y = 0;
            scene.add(surfaceMesh);

            // Subtle surface boundary grid
            const surfaceGrid = new THREE.GridHelper(100, 40, 0x1d587a, 0x0e3247);
            surfaceGrid.position.y = 0;
            surfaceGrid.material.transparent = true;
            surfaceGrid.material.opacity = 0.25;
            scene.add(surfaceGrid);

            // 4. Subtle Seabed Bathymetric Floor (Y = -12m, representing 12m seabed depth)
            const seabedGeo = new THREE.PlaneGeometry(120, 120, 16, 16);
            const seabedMat = new THREE.MeshStandardMaterial({
                color: 0x051a29,
                roughness: 0.90,
                metalness: 0.10
            });
            const seabedMesh = new THREE.Mesh(seabedGeo, seabedMat);
            seabedMesh.rotation.x = -Math.PI / 2;
            seabedMesh.position.y = -12.0;
            scene.add(seabedMesh);

            const seabedGrid = new THREE.GridHelper(100, 40, 0x143c59, 0x092236);
            seabedGrid.position.y = -11.95;
            seabedGrid.material.transparent = true;
            seabedGrid.material.opacity = 0.35;
            scene.add(seabedGrid);

            // 5. Marine Snow / Micro-Particulate Suspended Matter (Living Underwater Environment)
            const particleCount = 450;
            const particleGeo = new THREE.BufferGeometry();
            const particlePositions = new Float32Array(particleCount * 3);
            for (let i = 0; i < particleCount * 3; i += 3) {
                particlePositions[i] = (Math.random() - 0.5) * 50;     // X in [-25, 25]
                particlePositions[i + 1] = -Math.random() * 12;        // Y in [-12, 0] (water column)
                particlePositions[i + 2] = (Math.random() - 0.5) * 50; // Z in [-25, 25]
            }
            particleGeo.setAttribute('position', new THREE.BufferAttribute(particlePositions, 3));

            const particleMat = new THREE.PointsMaterial({
                color: 0x72cce4,
                size: 0.035,
                transparent: true,
                opacity: 0.40,
                blending: THREE.AdditiveBlending
            });
            marineSnowPoints = new THREE.Points(particleGeo, particleMat);
            scene.add(marineSnowPoints);

            // 6. Trajectory Line
            trajectoryGeo = new THREE.BufferGeometry();
            const trajectoryMat = new THREE.LineBasicMaterial({ color: 0x00f5d4, linewidth: 2, transparent: true, opacity: 0.85 });
            trajectoryLine = new THREE.Line(trajectoryGeo, trajectoryMat);
            scene.add(trajectoryLine);

            // =========================================================================
            // Realistic Torpedo-Class Cylindrical AUV 3D Model
            // Scale: Diameter = ~0.40m (Radius = 0.20m), Total Length = ~1.75m
            // Coordinate Frame: +X = Forward (Nose), -X = Aft (Thruster),
            //                   +Y = Up (Dorsal),   -Y = Down (Keel),
            //                   +Z = Right (Starboard), -Z = Left (Port)
            // =========================================================================
            auvGroup = new THREE.Group();

            // Professional Marine Robotics Materials
            const hullMat = new THREE.MeshStandardMaterial({
                color: 0x18283b,          // Deep Tactical Navy / Blue-Gray Pressure Hull
                roughness: 0.35,
                metalness: 0.70
            });

            const darkHullMat = new THREE.MeshStandardMaterial({
                color: 0x0e1724,          // Darkened Modular Structural Sections
                roughness: 0.40,
                metalness: 0.75
            });

            const ringMat = new THREE.MeshStandardMaterial({
                color: 0x223548,          // Anodized Titanium Joint Collars / Rings
                roughness: 0.25,
                metalness: 0.85
            });

            const finMat = new THREE.MeshStandardMaterial({
                color: 0x1f2e3d,          // Muted Matte Marine Hydrofoil Composite
                roughness: 0.40,
                metalness: 0.55
            });

            const thrusterDuctMat = new THREE.MeshStandardMaterial({
                color: 0x0a1118,          // Graphite / Black Ducted Shroud
                roughness: 0.35,
                metalness: 0.80
            });

            const propMat = new THREE.MeshStandardMaterial({
                color: 0x4a5d6e,          // Metallic Alloy Propeller Hub & Blades
                roughness: 0.25,
                metalness: 0.90
            });

            const detailMat = new THREE.MeshStandardMaterial({
                color: 0x2a3d4f,          // Structural Rails, Hardware & Mounts
                roughness: 0.30,
                metalness: 0.75
            });

            const sensorLensMat = new THREE.MeshStandardMaterial({
                color: 0x00f5d4,          // Optical Sensor Lens with Cyan Glow
                emissive: 0x00a8e8,
                emissiveIntensity: 0.4,
                roughness: 0.15,
                metalness: 0.9
            });

            const glassMat = new THREE.MeshStandardMaterial({
                color: 0x030a14,          // Optical Glass Dome Port
                roughness: 0.1,
                metalness: 0.95
            });

            // 1. Main Pressure Hull Segments (Diameter = ~0.40m -> Radius = 0.20m)
            // Mid Section (X = [-0.25, +0.25], length = 0.50m)
            const midHullGeo = new THREE.CylinderGeometry(0.20, 0.20, 0.50, 36);
            midHullGeo.rotateZ(-Math.PI / 2);
            const midHull = new THREE.Mesh(midHullGeo, hullMat);
            midHull.position.set(0, 0, 0);
            auvGroup.add(midHull);

            // Fore Section (X = [+0.25, +0.50], length = 0.25m)
            const foreHullGeo = new THREE.CylinderGeometry(0.20, 0.20, 0.25, 36);
            foreHullGeo.rotateZ(-Math.PI / 2);
            const foreHull = new THREE.Mesh(foreHullGeo, darkHullMat);
            foreHull.position.set(0.375, 0, 0);
            auvGroup.add(foreHull);

            // Aft Section (X = [-0.50, -0.25], length = 0.25m)
            const aftHullGeo = new THREE.CylinderGeometry(0.20, 0.20, 0.25, 36);
            aftHullGeo.rotateZ(-Math.PI / 2);
            const aftHull = new THREE.Mesh(aftHullGeo, darkHullMat);
            aftHull.position.set(-0.375, 0, 0);
            auvGroup.add(aftHull);

            // 2. Precision Joint Rings / Modular Collars
            const ringGeo = new THREE.TorusGeometry(0.202, 0.006, 8, 36);
            ringGeo.rotateY(Math.PI / 2);

            const ringPositions = [0.50, 0.25, -0.25, -0.50];
            ringPositions.forEach(posX => {
                const ringMesh = new THREE.Mesh(ringGeo, ringMat);
                ringMesh.position.set(posX, 0, 0);
                auvGroup.add(ringMesh);
            });

            // 3. Streamlined Nose Cone & Optical Dome
            // Nose Transition Cone (X = [+0.50, +0.72], length = 0.22m, tapering R: 0.20 -> 0.12)
            const noseConeGeo = new THREE.CylinderGeometry(0.12, 0.20, 0.22, 36);
            noseConeGeo.rotateZ(-Math.PI / 2);
            const noseCone = new THREE.Mesh(noseConeGeo, hullMat);
            noseCone.position.set(0.61, 0, 0);
            auvGroup.add(noseCone);

            // Forward Optical Dome (X = [+0.72, +0.84], Radius = 0.12m)
            const noseDomeGeo = new THREE.SphereGeometry(0.12, 28, 20, 0, Math.PI * 2, 0, Math.PI / 2);
            noseDomeGeo.rotateZ(-Math.PI / 2);
            const noseDome = new THREE.Mesh(noseDomeGeo, glassMat);
            noseDome.position.set(0.72, 0, 0);
            auvGroup.add(noseDome);

            // Forward Camera Sensor Lens Eye
            const lensGeo = new THREE.CylinderGeometry(0.04, 0.04, 0.02, 24);
            lensGeo.rotateZ(-Math.PI / 2);
            const lensMesh = new THREE.Mesh(lensGeo, sensorLensMat);
            lensMesh.position.set(0.835, 0, 0);
            auvGroup.add(lensMesh);

            // Twin LED Inspection Illuminators
            const lightHousingGeo = new THREE.CylinderGeometry(0.022, 0.022, 0.035, 20);
            lightHousingGeo.rotateZ(-Math.PI / 2);
            const lightLensGeo = new THREE.CircleGeometry(0.018, 20);
            lightLensGeo.rotateY(Math.PI / 2);

            [-0.13, 0.13].forEach(posZ => {
                const lightHousing = new THREE.Mesh(lightHousingGeo, detailMat);
                lightHousing.position.set(0.66, 0.04, posZ);
                auvGroup.add(lightHousing);

                const lightLens = new THREE.Mesh(lightLensGeo, sensorLensMat);
                lightLens.position.set(0.678, 0.04, posZ);
                auvGroup.add(lightLens);
            });

            // 4. Aft Boat-tail Transition (X = [-0.78, -0.50], length = 0.28m, tapering R: 0.20 -> 0.10)
            const boatTailGeo = new THREE.CylinderGeometry(0.20, 0.10, 0.28, 36);
            boatTailGeo.rotateZ(-Math.PI / 2);
            const boatTail = new THREE.Mesh(boatTailGeo, hullMat);
            boatTail.position.set(-0.64, 0, 0);
            auvGroup.add(boatTail);

            // 5. Four Symmetrical Control Fins (Cruciform Aft Empennage: Top, Bottom, Right, Left)
            // Vertical Rudder Geometry (Top & Bottom)
            const vFinShape = new THREE.Shape();
            vFinShape.moveTo(-0.12, 0);     // Root Aft
            vFinShape.lineTo(0.12, 0);      // Root Fore
            vFinShape.lineTo(0.05, 0.22);   // Tip Fore (Swept Back)
            vFinShape.lineTo(-0.07, 0.22);  // Tip Aft
            vFinShape.closePath();

            const finExtrudeSettings = {
                depth: 0.018,
                bevelEnabled: true,
                bevelSegments: 2,
                steps: 1,
                bevelSize: 0.003,
                bevelThickness: 0.003
            };

            const vFinGeo = new THREE.ExtrudeGeometry(vFinShape, finExtrudeSettings);
            vFinGeo.translate(0, 0, -0.009); // Center on thickness axis

            // Top Rudder Fin Group (Hinge at X = -0.65, Y = 0.18)
            finTopGroup = new THREE.Group();
            finTopGroup.position.set(-0.65, 0.18, 0);
            const finTopMesh = new THREE.Mesh(vFinGeo, finMat);
            finTopGroup.add(finTopMesh);
            auvGroup.add(finTopGroup);

            // Bottom Rudder Fin Group (Hinge at X = -0.65, Y = -0.18)
            finBottomGroup = new THREE.Group();
            finBottomGroup.position.set(-0.65, -0.18, 0);
            const finBottomMesh = new THREE.Mesh(vFinGeo, finMat);
            finBottomMesh.rotation.z = Math.PI; // Symmetrical downward extension
            finBottomGroup.add(finBottomMesh);
            auvGroup.add(finBottomGroup);

            // Horizontal Elevator Geometry (Right & Left)
            // Dimensions matching rudder: Root Chord = 0.24m, Tip Chord = 0.12m, Span = 0.26m
            const hFinShape = new THREE.Shape();
            hFinShape.moveTo(-0.12, 0);     // Root Aft
            hFinShape.lineTo(0.12, 0);      // Root Fore
            hFinShape.lineTo(0.05, 0.26);   // Tip Fore (Swept Back)
            hFinShape.lineTo(-0.07, 0.26);  // Tip Aft
            hFinShape.closePath();

            // Starboard Elevator (extends horizontally outward along +Z)
            const hFinGeoRight = new THREE.ExtrudeGeometry(hFinShape, finExtrudeSettings);
            hFinGeoRight.rotateX(Math.PI / 2);   // Rotates shape into horizontal plane: y -> +z (outward along +Z)
            hFinGeoRight.translate(0, 0.009, 0); // Center on Y thickness axis

            // Starboard Elevator Group (Mounted at outer hull surface X = -0.65, Z = +0.16)
            finRightGroup = new THREE.Group();
            finRightGroup.position.set(-0.65, 0, 0.16);
            const finRightMesh = new THREE.Mesh(hFinGeoRight, finMat);
            finRightGroup.add(finRightMesh);
            auvGroup.add(finRightGroup);

            // Port Elevator (extends horizontally outward along -Z)
            const hFinGeoLeft = new THREE.ExtrudeGeometry(hFinShape, finExtrudeSettings);
            hFinGeoLeft.rotateX(-Math.PI / 2);  // Rotates shape into horizontal plane: y -> -z (outward along -Z)
            hFinGeoLeft.translate(0, -0.009, 0); // Center on Y thickness axis

            // Port Elevator Group (Mounted at outer hull surface X = -0.65, Z = -0.16)
            finLeftGroup = new THREE.Group();
            finLeftGroup.position.set(-0.65, 0, -0.16);
            const finLeftMesh = new THREE.Mesh(hFinGeoLeft, finMat);
            finLeftGroup.add(finLeftMesh);
            auvGroup.add(finLeftGroup);

            // 6. Rear Ducted Propulsion System & Rotating Propeller
            // Circular Kort Nozzle Thruster Duct Shroud
            const ductOuterGeo = new THREE.CylinderGeometry(0.125, 0.135, 0.15, 32, 1, true);
            ductOuterGeo.rotateZ(-Math.PI / 2);
            const ductOuter = new THREE.Mesh(ductOuterGeo, thrusterDuctMat);
            ductOuter.position.set(-0.88, 0, 0);
            auvGroup.add(ductOuter);

            const ductInnerGeo = new THREE.CylinderGeometry(0.112, 0.122, 0.15, 32, 1, true);
            ductInnerGeo.rotateZ(-Math.PI / 2);
            const ductInnerMat = new THREE.MeshStandardMaterial({ color: 0x080e14, roughness: 0.5, metalness: 0.8, side: THREE.BackSide });
            const ductInner = new THREE.Mesh(ductInnerGeo, ductInnerMat);
            ductInner.position.set(-0.88, 0, 0);
            auvGroup.add(ductInner);

            // Protective Shroud Rims
            const ductRimGeo = new THREE.TorusGeometry(0.128, 0.006, 8, 32);
            ductRimGeo.rotateY(Math.PI / 2);

            const ductRimFore = new THREE.Mesh(ductRimGeo, ringMat);
            ductRimFore.position.set(-0.805, 0, 0);
            auvGroup.add(ductRimFore);

            const ductRimAft = new THREE.Mesh(ductRimGeo, ringMat);
            ductRimAft.position.set(-0.955, 0, 0);
            auvGroup.add(ductRimAft);

            // 4 Radial Duct Support Stators
            const statorGeo = new THREE.BoxGeometry(0.022, 0.075, 0.006);
            statorGeo.translate(0, 0.075, 0);
            for (let i = 0; i < 4; i++) {
                const angle = (i * Math.PI) / 2 + Math.PI / 4;
                const stator = new THREE.Mesh(statorGeo, detailMat);
                stator.position.set(-0.83, 0, 0);
                stator.rotation.x = angle;
                auvGroup.add(stator);
            }

            // Rotating Propeller Group
            propellerGroup = new THREE.Group();
            propellerGroup.position.set(-0.88, 0, 0);

            // Central Propeller Hub Spinner
            const hubGeo = new THREE.ConeGeometry(0.035, 0.12, 20);
            hubGeo.rotateZ(Math.PI / 2); // Cone apex points aft (-X)
            const hubMesh = new THREE.Mesh(hubGeo, propMat);
            hubMesh.position.set(-0.02, 0, 0);
            propellerGroup.add(hubMesh);

            // 4 Propeller Blades
            const bladeGeo = new THREE.BoxGeometry(0.006, 0.065, 0.022);
            bladeGeo.translate(0, 0.040, 0);
            for (let i = 0; i < 4; i++) {
                const angle = (i * Math.PI) / 2;
                const blade = new THREE.Mesh(bladeGeo, propMat);
                blade.rotation.x = angle;
                blade.rotation.y = 0.38; // Blade pitch angle
                propellerGroup.add(blade);
            }
            auvGroup.add(propellerGroup);

            // 7. Sensor Details, Mounting Rails & Deck Features
            // Side Protection / Guide Rails
            const railGeo = new THREE.BoxGeometry(0.75, 0.018, 0.010);
            [-0.202, 0.202].forEach(posZ => {
                const rail = new THREE.Mesh(railGeo, detailMat);
                rail.position.set(0.02, 0, posZ);
                auvGroup.add(rail);
            });

            // Bottom DVL 4-Transducer Acoustic Array
            const dvlBaseGeo = new THREE.CylinderGeometry(0.06, 0.06, 0.015, 24);
            const dvlBase = new THREE.Mesh(dvlBaseGeo, detailMat);
            dvlBase.position.set(0.12, -0.198, 0);
            auvGroup.add(dvlBase);

            const transducerGeo = new THREE.CircleGeometry(0.016, 16);
            transducerGeo.rotateX(Math.PI / 2);
            const dvlTransducerMat = new THREE.MeshStandardMaterial({ color: 0x050c14, roughness: 0.2, metalness: 0.9 });

            [[-0.024, -0.024], [0.024, -0.024], [-0.024, 0.024], [0.024, 0.024]].forEach(([dx, dz]) => {
                const transducer = new THREE.Mesh(transducerGeo, dvlTransducerMat);
                transducer.position.set(0.12 + dx, -0.206, dz);
                auvGroup.add(transducer);
            });

            // Top Navigation Antenna / Strobe Mast
            const mastShape = new THREE.Shape();
            mastShape.moveTo(-0.06, 0);
            mastShape.lineTo(0.06, 0);
            mastShape.lineTo(0.03, 0.05);
            mastShape.lineTo(-0.04, 0.05);
            mastShape.closePath();

            const mastGeo = new THREE.ExtrudeGeometry(mastShape, { depth: 0.014, bevelEnabled: true, bevelSize: 0.002, bevelThickness: 0.002 });
            mastGeo.translate(0, 0, -0.007);
            const mast = new THREE.Mesh(mastGeo, detailMat);
            mast.position.set(-0.15, 0.198, 0);
            auvGroup.add(mast);

            // Top Strobe Beacon LED
            const beaconGeo = new THREE.SphereGeometry(0.008, 12, 12);
            const beacon = new THREE.Mesh(beaconGeo, sensorLensMat);
            beacon.position.set(-0.15, 0.252, 0);
            auvGroup.add(beacon);

            // Side Transducer Ports (Port & Starboard)
            const sidePortGeo = new THREE.RingGeometry(0.014, 0.028, 20);
            const sidePortMat = new THREE.MeshStandardMaterial({ color: 0x223548, roughness: 0.3, metalness: 0.8, side: THREE.DoubleSide });

            [-0.201, 0.201].forEach(posZ => {
                const sidePort = new THREE.Mesh(sidePortGeo, sidePortMat);
                sidePort.position.set(0.42, -0.03, posZ);
                sidePort.rotation.y = posZ > 0 ? 0 : Math.PI;
                auvGroup.add(sidePort);
            });

            // 8. 3-Axis Body Frame Orientation Indicator (X=Forward, Y=Right/Starboard, Z=Down)
            const axisOrigin = new THREE.Vector3(0.85, 0, 0);

            // X-Axis: Forward (Bright Cyan)
            const dirX = new THREE.Vector3(1, 0, 0);
            const arrowX = new THREE.ArrowHelper(dirX, axisOrigin, 0.65, 0x00f5d4, 0.16, 0.07);
            auvGroup.add(arrowX);

            // Y-Axis: Right / Starboard (Aqua Sky)
            const dirY = new THREE.Vector3(0, 0, 1);
            const arrowY = new THREE.ArrowHelper(dirY, axisOrigin, 0.45, 0x48cae4, 0.12, 0.06);
            auvGroup.add(arrowY);

            // Z-Axis: Down (Tactical Blue-Cyan)
            const dirZ = new THREE.Vector3(0, -1, 0);
            const arrowZ = new THREE.ArrowHelper(dirZ, axisOrigin, 0.45, 0x00a8e8, 0.12, 0.06);
            auvGroup.add(arrowZ);

            scene.add(auvGroup);

            window.addEventListener('resize', onWindowResize);
            animate();
        }

        function onWindowResize() {
            camera.aspect = window.innerWidth / window.innerHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight);
        }

        function setCameraMode(mode) {
            cameraMode = mode;
            if (!latestState) return;

            const posX = latestState.position.x;
            const posY = latestState.position.y;
            const posZ = latestState.position.z;
            const target3js = new THREE.Vector3(posX, -posZ, posY);

            if (mode === 'top') {
                camera.position.set(target3js.x, target3js.y + 15, target3js.z);
                controls.target.copy(target3js);
            } else if (mode === 'side') {
                camera.position.set(target3js.x, target3js.y, target3js.z + 12);
                controls.target.copy(target3js);
            } else if (mode === 'follow') {
                camera.position.set(target3js.x - 4, target3js.y + 2, target3js.z + 4);
                controls.target.copy(target3js);
            }
            controls.update();
        }

        function updateHUD(data) {
            latestState = data;

            // 1. Arming Status
            const isArmed = data.status.armed;
            const dotArmed = document.getElementById('dot-armed');
            const txtArmed = document.getElementById('txt-armed');
            if (isArmed) {
                dotArmed.className = 'dot-indicator dot-armed';
                txtArmed.innerText = 'ARMED';
            } else {
                dotArmed.className = 'dot-indicator dot-disarmed';
                txtArmed.innerText = 'DISARMED';
            }

            // 2. Gain
            const gainPct = data.status.gain || 75;
            document.getElementById('txt-gain').innerText = `${gainPct}%`;
            document.getElementById('bar-gain').style.width = `${gainPct}%`;

            // 3. Inspection Mission Telemetry & Status
            const aut = data.autonomy || {};
            const insp = aut.inspection_mission || {};
            const phase = insp.status || (data.status ? data.status.mission : 'IDLE') || 'IDLE';
            const dhStatus = data.status ? (data.status.depth_hold || 'OFF') : 'OFF';
            const hhStatus = data.status ? (data.status.heading_hold || 'OFF') : 'OFF';

            const statusBadge = document.getElementById('txt-mission-status');
            statusBadge.innerText = phase;
            if (phase === 'DESCENDING') {
                statusBadge.style.color = '#00a8e8';
                statusBadge.style.borderColor = '#00a8e8';
            } else if (phase === 'INSPECTION') {
                statusBadge.style.color = '#00f5d4';
                statusBadge.style.borderColor = '#00f5d4';
            } else if (phase === 'SURFACING') {
                statusBadge.style.color = '#ffb703';
                statusBadge.style.borderColor = '#ffb703';
            } else if (phase === 'MISSION COMPLETE' || phase === 'SURFACE REACHED') {
                statusBadge.style.color = '#52b788';
                statusBadge.style.borderColor = '#52b788';
            } else {
                statusBadge.style.color = '#00f5d4';
                statusBadge.style.borderColor = '#00f5d4';
            }

            // Mode Badge
            const badgeMode = document.getElementById('badge-mode');
            if (phase === 'IDLE' || phase === 'DISARMED') {
                badgeMode.innerText = isArmed ? 'MANUAL' : 'DISARMED';
                badgeMode.className = 'badge-box';
            } else {
                badgeMode.innerText = `AUTO: ${phase}`;
                badgeMode.className = 'badge-box badge-enabled';
            }

            // Top Status Bar Badges
            const isDepthHoldActive = (dhStatus.startsWith('ON:') || dhStatus === 'ACTIVE');
            const bDH = document.getElementById('badge-depth-hold');
            if (isDepthHoldActive) {
                bDH.className = 'badge-box badge-enabled';
                bDH.innerText = `DEPTH HOLD: ${dhStatus.replace('ON:', '')}m`;
            } else {
                bDH.className = 'badge-box badge-disabled';
                bDH.innerText = 'DEPTH HOLD: OFF';
            }

            const isHeadingHoldActive = (hhStatus.startsWith('ON:') || hhStatus === 'ACTIVE');
            const bHH = document.getElementById('badge-heading-hold');
            if (isHeadingHoldActive) {
                bHH.className = 'badge-box badge-enabled';
                bHH.innerText = `HEADING HOLD: ${hhStatus.replace('ON:', '')}\u00b0`;
            } else {
                bHH.className = 'badge-box badge-disabled';
                bHH.innerText = 'HEADING HOLD: OFF';
            }

            const rollStabStatus = data.status ? (data.status.roll_stabilizer || 'ACTIVE') : 'ACTIVE';
            const bRoll = document.getElementById('badge-roll-stab');
            if (rollStabStatus === 'ACTIVE') {
                bRoll.className = 'badge-box badge-enabled';
                bRoll.innerText = 'ROLL STABILIZER: ACTIVE';
            } else {
                bRoll.className = 'badge-box badge-disabled';
                bRoll.innerText = 'ROLL STABILIZER: OFF';
            }

            // Mission inputs from autonomy state if active
            const activeDepthTgt = aut.active_depth_target !== undefined ? aut.active_depth_target : (aut.target_depth || 0.0);
            const activeHeadingTgt = aut.active_heading_target !== undefined ? aut.active_heading_target : (aut.target_heading || 0.0);
            const reqSpd = insp.target_speed || aut.throttle_cmd || 0.0;
            const cmdSpd = aut.throttle_cmd || 0.0;
            const actSpd = data.velocities ? (data.velocities.vx || 0.0) : 0.0;

            // 4. Left Telemetry Data Panel
            const orient = data.orientation || { roll: 0.0, pitch: 0.0, yaw: 0.0 };
            const roll = orient.roll || 0.0;
            const pitch = orient.pitch || 0.0;
            const yaw = orient.yaw || 0.0;

            document.getElementById('val-roll').innerText = `${roll >= 0 ? '+' : ''}${roll.toFixed(1)}\u00b0`;
            document.getElementById('val-pitch').innerText = `${pitch >= 0 ? '+' : ''}${pitch.toFixed(1)}\u00b0`;
            document.getElementById('val-yaw').innerText = `${yaw >= 0 ? '+' : ''}${yaw.toFixed(1)}\u00b0`;

            // Compass Dial & Digital Heading Display
            const compassDial = document.getElementById('compass-dial');
            if (compassDial) {
                compassDial.style.transform = `rotate(${-yaw}deg)`;
            }
            const compassVal = document.getElementById('compass-val');
            if (compassVal) {
                const normHdg = ((Math.round(yaw) % 360) + 360) % 360;
                compassVal.textContent = `${normHdg.toString().padStart(3, '0')}°`;
            }

            const vels = data.velocities || { vx: 0.0, vy: 0.0, vz: 0.0 };
            const vx = vels.vx || 0.0;
            const vy = vels.vy || 0.0;
            const vz = vels.vz || 0.0;

            document.getElementById('val-vx').innerText = `${vx >= 0 ? '+' : ''}${vx.toFixed(2)} m/s`;
            document.getElementById('val-vy').innerText = `${vy >= 0 ? '+' : ''}${vy.toFixed(2)} m/s`;
            document.getElementById('val-vz').innerText = `${vz >= 0 ? '+' : ''}${vz.toFixed(2)} m/s`;

            const pos = data.position || { x: 0.0, y: 0.0, z: 0.5 };
            const depth = pos.z !== undefined ? pos.z : 0.5;
            document.getElementById('val-depth').innerText = `${depth.toFixed(2)} m`;
            document.getElementById('val-lin-vel').innerText = `[${vx >= 0 ? '+' : ''}${vx.toFixed(1)}, ${vy >= 0 ? '+' : ''}${vy.toFixed(1)}, ${vz >= 0 ? '+' : ''}${vz.toFixed(1)}] m/s`;

            const ax = data.accel ? data.accel.ax : 0.0;
            const ay = data.accel ? data.accel.ay : 0.0;
            const az = data.accel ? data.accel.az : -9.81;
            document.getElementById('val-imu-accel').innerText = `[${ax >= 0 ? '+' : ''}${ax.toFixed(1)}, ${ay >= 0 ? '+' : ''}${ay.toFixed(1)}, ${az.toFixed(1)}] m/s²`;

            document.getElementById('txt-packets').innerText = `${data.packets || 0} pkts`;
            document.getElementById('val-motion').innerText = data.world_motion || 'NONE';

            // 5. Unified Telemetry Panel — Active Targets
            const curDepth = depth;
            const depthErr = isDepthHoldActive ? (activeDepthTgt - curDepth) : 0.0;
            const depthCmd = aut.depth_cmd || 0.0;

            const curHead = yaw;
            let headErr = activeHeadingTgt - curHead;
            while (headErr > 180) headErr -= 360;
            while (headErr < -180) headErr += 360;
            if (!isHeadingHoldActive) headErr = 0.0;
            const rudCmd = aut.rudder_cmd || 0.0;

            const acts = data.actual_actuators || data.actuators || { main_thruster: 1500, elevator_left: 1500, elevator_right: 1500, rudder_left: 1500, rudder_right: 1500 };
            const thrPWM = acts.main_thruster !== undefined ? acts.main_thruster : 1500;
            const elevLPWM = acts.elevator_left !== undefined ? acts.elevator_left : 1500;
            const elevRPWM = acts.elevator_right !== undefined ? acts.elevator_right : elevLPWM;
            const rudLPWM = acts.rudder_left !== undefined ? acts.rudder_left : 1500;
            const rudRPWM = acts.rudder_right !== undefined ? acts.rudder_right : rudLPWM;

            // Actuator Target Deflections & Thrust
            currentThrustNorm = (thrPWM - 1500) / 400.0;

            const el_l_norm = (elevLPWM - 1500) / 400.0;
            const el_r_norm = (elevRPWM - 1500) / 400.0;

            // Decompose into common pitch component and differential roll component
            const pitch_component = (el_l_norm - el_r_norm) / 2.0;
            const roll_component = (el_l_norm + el_r_norm) / 2.0;

            const maxFinAngle = 0.436; // 25 degrees in radians
            const pitch_deflection = pitch_component * maxFinAngle;
            const roll_deflection = roll_component * maxFinAngle;

            // Normal Pitch: Left = Right (matched pair)
            // Roll Stabilizer: Left = -Right (differential pair)
            targetElevLAngle = pitch_deflection + roll_deflection;
            targetElevRAngle = pitch_deflection - roll_deflection;

            targetRudAngle = ((rudLPWM - 1500) / 400.0) * maxFinAngle;

            // Mission section
            document.getElementById('ut-phase').innerText = phase;
            const mElapsed = insp.elapsed_time || 0;
            const mTotal = (insp.duration_min || 30) * 60;
            const fmtTime = (sec) => {
                const m = Math.floor(sec / 60).toString().padStart(2, '0');
                const s = Math.floor(sec % 60).toString().padStart(2, '0');
                return `${m}:${s}`;
            };
            document.getElementById('ut-time').innerText = `${fmtTime(mElapsed)} / ${fmtTime(mTotal)}`;

            // Depth section
            document.getElementById('ut-depth-tgt').innerText = `${activeDepthTgt.toFixed(2)} m`;
            document.getElementById('ut-depth-cur').innerText = `${curDepth.toFixed(2)} m`;
            document.getElementById('ut-depth-err').innerText = `${depthErr >= 0 ? '+' : ''}${depthErr.toFixed(2)} m`;
            document.getElementById('ut-depth-out').innerText = `${depthCmd >= 0 ? '+' : ''}${depthCmd.toFixed(1)}`;

            // Heading section
            document.getElementById('ut-head-tgt').innerText = `${activeHeadingTgt.toFixed(1)}\u00b0`;
            document.getElementById('ut-head-cur').innerText = `${curHead.toFixed(1)}\u00b0`;
            document.getElementById('ut-head-err').innerText = `${headErr >= 0 ? '+' : ''}${headErr.toFixed(1)}\u00b0`;
            document.getElementById('ut-head-out').innerText = `${rudCmd >= 0 ? '+' : ''}${rudCmd.toFixed(1)}`;

            // Speed section
            document.getElementById('ut-spd-tgt').innerText = `${reqSpd.toFixed(2)} m/s`;
            document.getElementById('ut-spd-cmd').innerText = `${cmdSpd.toFixed(2)} m/s`;
            document.getElementById('ut-spd-act').innerText = `${actSpd.toFixed(2)} m/s`;

            // Actuator section
            document.getElementById('ut-act-thr').innerText = `${thrPWM} us`;
            document.getElementById('ut-act-elev').innerText = `${elevLPWM} us`;
            document.getElementById('ut-act-rud').innerText = `${rudLPWM} us`;

            // Physical state section
            document.getElementById('ut-phys-fwd').innerText = `${vx.toFixed(2)} m/s`;
            document.getElementById('ut-phys-vz').innerText = `${vz.toFixed(2)} m/s`;
            document.getElementById('ut-phys-pitch').innerText = `${pitch >= 0 ? '+' : ''}${pitch.toFixed(1)}\u00b0`;
            document.getElementById('ut-phys-yaw').innerText = `${yaw >= 0 ? '+' : ''}${yaw.toFixed(1)}\u00b0`;
            document.getElementById('ut-phys-roll').innerText = `${roll >= 0 ? '+' : ''}${roll.toFixed(1)}\u00b0`;
            document.getElementById('ut-phys-depth').innerText = `${curDepth.toFixed(2)} m`;

            // 6. Physics Debug Data Panel
            const phys = data.physics || {};
            const buoyF = phys.buoyancy_force !== undefined ? phys.buoyancy_force : 12.3;
            const thrF = phys.thrust_force || 0.0;
            const fwdDrag = phys.forward_drag || 0.0;
            const qDyn = phys.dynamic_pressure || 0.0;
            const mPitch = phys.pitch_moment || 0.0;
            const nYaw = phys.yaw_moment || 0.0;
            const lRoll = phys.roll_moment || 0.0;

            document.getElementById('phys-buoyancy').innerText = `${buoyF >= 0 ? '+' : ''}${buoyF.toFixed(1)} N`;
            document.getElementById('phys-thrust').innerText = `${thrF >= 0 ? '+' : ''}${thrF.toFixed(1)} N`;
            document.getElementById('phys-drag').innerText = `${fwdDrag.toFixed(1)} N`;
            document.getElementById('phys-qdyn').innerText = `${qDyn.toFixed(1)} Pa`;
            document.getElementById('phys-mpitch').innerText = `${mPitch.toFixed(2)} Nm`;
            document.getElementById('phys-nyaw').innerText = `${nYaw.toFixed(2)} Nm`;
            document.getElementById('phys-lroll').innerText = `${lRoll.toFixed(2)} Nm`;

            // Actuator PWM Bars
            const thrPct = Math.round(((thrPWM - 1500) / 400) * 100);
            const elevPct = Math.round(((elevLPWM - 1500) / 400) * 100);
            const rudPct = Math.round(((rudLPWM - 1500) / 400) * 100);

            document.getElementById('val-thr-pwm').innerText = `${thrPWM} us (${thrPct >= 0 ? '+' : ''}${thrPct}%)`;
            document.getElementById('fill-thr-pwm').style.width = `${((thrPWM - 1100) / 800) * 100}%`;

            document.getElementById('val-elev-pwm').innerText = `${elevLPWM} us (${elevPct >= 0 ? '+' : ''}${elevPct}%)`;
            document.getElementById('fill-elev-pwm').style.width = `${((elevLPWM - 1100) / 800) * 100}%`;

            document.getElementById('val-rud-pwm').innerText = `${rudLPWM} us (${rudPct >= 0 ? '+' : ''}${rudPct}%)`;
            document.getElementById('fill-rud-pwm').style.width = `${((rudLPWM - 1100) / 800) * 100}%`;

            // 7. Update 3D AUV Pose in Three.js
            const posX = pos.x !== undefined ? pos.x : 0.0;
            const posY = pos.y !== undefined ? pos.y : 0.0;
            const posZ = pos.z !== undefined ? pos.z : 0.5;

            const tX = posX;
            const tY = -posZ;
            const tZ = posY;

            auvGroup.position.set(tX, tY, tZ);

            const rollRad = (roll * Math.PI) / 180.0;
            const pitchRad = (pitch * Math.PI) / 180.0;
            const yawRad = (yaw * Math.PI) / 180.0;

            auvGroup.rotation.set(rollRad, -yawRad, pitchRad, 'YXZ');

            // Trajectory
            trajectoryPoints.push(new THREE.Vector3(tX, tY, tZ));
            if (trajectoryPoints.length > maxTrajectoryPoints) {
                trajectoryPoints.shift();
            }
            trajectoryGeo.setFromPoints(trajectoryPoints);
        }

        function animate() {
            requestAnimationFrame(animate);
            controls.update();

            // 1. Propeller Continuous Visual Rotation based on Thrust
            if (propellerGroup && Math.abs(currentThrustNorm) > 0.01) {
                // Rotation speed proportional to normalized thrust
                propRotationAngle += currentThrustNorm * 0.40;
                propellerGroup.rotation.x = propRotationAngle;
            }

            // 2. Smooth Fin Deflection Animation (Lerp to Target Angle)
            const finLerpFactor = 0.15;
            currentRudAngle += (targetRudAngle - currentRudAngle) * finLerpFactor;
            currentElevLAngle += (targetElevLAngle - currentElevLAngle) * finLerpFactor;
            currentElevRAngle += (targetElevRAngle - currentElevRAngle) * finLerpFactor;

            if (finTopGroup) finTopGroup.rotation.y = currentRudAngle;
            if (finBottomGroup) finBottomGroup.rotation.y = currentRudAngle;
            if (finRightGroup) finRightGroup.rotation.z = currentElevRAngle;
            if (finLeftGroup) finLeftGroup.rotation.z = currentElevLAngle;

            // 3. Marine Snow Gentle Oceanic Drift
            if (marineSnowPoints) {
                const posArr = marineSnowPoints.geometry.attributes.position.array;
                for (let i = 1; i < posArr.length; i += 3) {
                    posArr[i] -= 0.004; // Gentle downward settling
                    if (posArr[i] < -12.0) posArr[i] = 0.0; // Wrap back to surface
                }
                marineSnowPoints.geometry.attributes.position.needsUpdate = true;
            }

            renderer.render(scene, camera);
        }

        function startSSE() {
            const evtSource = new EventSource('/events');
            evtSource.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    updateHUD(data);
                } catch (err) {
                    console.error('JSON error:', err);
                }
            };
        }

        // Real-Time WebSockets Joystick Streaming
        let joyWS = null;
        function initJoyWS() {
            try {
                const wsProto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
                joyWS = new WebSocket(wsProto + '//' + window.location.host + '/ws');
                joyWS.onclose = () => { setTimeout(initJoyWS, 1000); };
                joyWS.onerror = () => { try { joyWS.close(); } catch(e){} };
            } catch(e){}
        }

        let lastJoyTime = 0;
        const keysDown = {};

        window.addEventListener('keydown', (e) => {
            if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
            keysDown[e.code] = true;
            if (e.code === 'Space') {
                e.preventDefault();
                toggleArm();
            }
        });

        window.addEventListener('keyup', (e) => {
            if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
            keysDown[e.code] = false;
        });

        function toggleArm() {
            const isCurrentlyArmed = latestState && latestState.status && latestState.status.armed;
            const nextArmed = !isCurrentlyArmed;
            fetch('/api/arm', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ armed: nextArmed })
            }).catch(err => console.error('Arming toggle error:', err));
        }

        function pollGamepad() {
            const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
            let activeGp = null;
            for (let i = 0; i < gamepads.length; i++) {
                if (gamepads[i] && gamepads[i].connected) {
                    activeGp = gamepads[i];
                    break;
                }
            }

            const joyBadge = document.getElementById('badge-joy');
            let axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let buttons = [0, 0, 0, 0, 0, 0, 0, 0];
            let hasInput = false;

            if (activeGp) {
                joyBadge.innerText = 'JOYSTICK: CONNECTED';
                joyBadge.className = 'badge-box badge-enabled';
                axes = Array.from(activeGp.axes).map((v, idx) => {
                    let val = Math.abs(v) < 0.05 ? 0.0 : v;
                    if (idx === 1 || idx === 3 || idx === 5) {
                        val = -val;
                    }
                    return val;
                });
                buttons = Array.from(activeGp.buttons).map(b => b.pressed ? 1 : 0);
                hasInput = true;
            } else {
                // Keyboard Teleop Fallback:
                // W/S = Thruster (+1 / -1 on axis 3)
                // A/D = Rudder (-1 / +1 on axis 0)
                // Up/Down = Elevator (+1 / -1 on axis 1)
                // X = Depth hold (button 2)
                // R/H = Heading hold (button 5)
                // Y/T = Gain up/down (buttons 3 / 0)
                if (keysDown['KeyW']) { axes[3] = 1.0; hasInput = true; }
                if (keysDown['KeyS']) { axes[3] = -1.0; hasInput = true; }
                if (keysDown['KeyA']) { axes[0] = -1.0; hasInput = true; }
                if (keysDown['KeyD']) { axes[0] = 1.0; hasInput = true; }
                if (keysDown['ArrowUp']) { axes[1] = 1.0; hasInput = true; }
                if (keysDown['ArrowDown']) { axes[1] = -1.0; hasInput = true; }

                if (keysDown['KeyX']) { buttons[2] = 1; hasInput = true; }
                if (keysDown['KeyB']) { buttons[1] = 1; hasInput = true; }
                if (keysDown['KeyR'] || keysDown['KeyH']) { buttons[5] = 1; hasInput = true; }
                if (keysDown['KeyL']) { buttons[4] = 1; hasInput = true; }
                if (keysDown['KeyY']) { buttons[3] = 1; hasInput = true; }
                if (keysDown['KeyT']) { buttons[0] = 1; hasInput = true; }

                const isUsingKeyboard = Object.values(keysDown).some(v => v === true);
                if (isUsingKeyboard) {
                    joyBadge.innerText = 'KEYBOARD TELEOP';
                    joyBadge.className = 'badge-box badge-enabled';
                } else {
                    joyBadge.innerText = 'JOYSTICK: DISCONNECTED';
                    joyBadge.className = 'badge-box badge-disabled';
                }
            }

            const now = performance.now();
            if ((activeGp || hasInput) && (now - lastJoyTime >= 25)) { // ~40 Hz streaming
                lastJoyTime = now;
                const payload = JSON.stringify({ axes: axes, buttons: buttons });

                if (joyWS && joyWS.readyState === WebSocket.OPEN) {
                    joyWS.send(payload);
                } else {
                    fetch('/api/joy', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: payload
                    }).catch(() => {});
                }
            }
        }

        function startMission() {
            const depth = parseFloat(document.getElementById('input-depth').value) || 20.0;
            const speed = parseFloat(document.getElementById('input-speed').value) || 1.0;
            const heading = parseFloat(document.getElementById('input-heading').value) || 90.0;
            const duration = parseFloat(document.getElementById('input-duration').value) || 30.0;

            fetch('/api/mission/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ depth: depth, speed: speed, heading: heading, duration: duration })
            }).catch(err => console.error('Mission start error:', err));
        }

        function stopMission() {
            fetch('/api/mission/stop', { method: 'POST' })
            .catch(err => console.error('Mission stop error:', err));
        }

        window.onload = () => {
            init3D();
            startSSE();
            initJoyWS();
            setInterval(pollGamepad, 25);
        };
    </script>
</body>
</html>
"""


def handle_websocket_connection(sock, node_instance):
    sock.settimeout(None)
    while True:
        try:
            head = sock.recv(2)
            if not head or len(head) < 2:
                break
            b1, b2 = head[0], head[1]
            opcode = b1 & 0x0f
            if opcode == 0x8:  # Close frame
                break
            
            has_mask = bool(b2 & 0x80)
            length = b2 & 0x7f
            if length == 126:
                len_bytes = sock.recv(2)
                if len(len_bytes) < 2: break
                length = struct.unpack(">H", len_bytes)[0]
            elif length == 127:
                len_bytes = sock.recv(8)
                if len(len_bytes) < 8: break
                length = struct.unpack(">Q", len_bytes)[0]

            masks = sock.recv(4) if has_mask else None
            payload = b""
            while len(payload) < length:
                chunk = sock.recv(length - len(payload))
                if not chunk: break
                payload += chunk
            if len(payload) < length:
                break

            if has_mask and masks:
                payload = bytes(b ^ masks[i % 4] for i, b in enumerate(payload))

            data = json.loads(payload.decode('utf-8'))
            axes = data.get('axes', [])
            buttons = data.get('buttons', [])
            if node_instance is not None:
                node_instance.publish_joy(axes, buttons)
        except Exception:
            break


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class WebHTTPRequestHandler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        pass

    def do_HEAD(self):
        if self.path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
        else:
            self.send_error(404, "File Not Found")

    def do_GET(self):
        # WebSocket Handshake Upgrade
        if self.headers.get('Upgrade', '').lower() == 'websocket' or self.path == '/ws':
            key = self.headers.get('Sec-WebSocket-Key', '')
            accept_key = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
            self.send_response(101, "Switching Protocols")
            self.send_header('Upgrade', 'websocket')
            self.send_header('Connection', 'Upgrade')
            self.send_header('Sec-WebSocket-Accept', accept_key)
            self.end_headers()
            handle_websocket_connection(self.connection, ros_node_instance)
            return

        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode('utf-8'))
        elif self.path == '/api/state':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(simulation_state).encode('utf-8'))
        elif self.path == '/events':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    data = json.dumps(simulation_state)
                    self.wfile.write(f"data: {data}\n\n".encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(0.033)  # ~30 FPS
            except (ConnectionResetError, BrokenPipeError):
                pass
        else:
            self.send_error(404, "File Not Found")

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode('utf-8') if content_length > 0 else '{}'
        
        try:
            data = json.loads(body)
        except Exception:
            data = {}

        if self.path == '/api/joy':
            if ros_node_instance is not None:
                axes = data.get('axes', [])
                buttons = data.get('buttons', [])
                ros_node_instance.publish_joy(axes, buttons)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')

        elif self.path == '/api/mission/start':
            if ros_node_instance is not None:
                depth = float(data.get('depth', 20.0))
                speed = float(data.get('speed', 1.0))
                heading = float(data.get('heading', 90.0))
                duration = float(data.get('duration', 30.0))
                ros_node_instance.start_mission(depth, heading, 0.0, speed, duration)
            self.send_response(200)

            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status":"started"}')

        elif self.path == '/api/arm':
            if ros_node_instance is not None:
                armed_val = bool(data.get('armed', True))
                arm_msg = Bool()
                arm_msg.data = armed_val
                ros_node_instance.arm_pub.publish(arm_msg)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')

        elif self.path == '/api/mission/stop':
            if ros_node_instance is not None:
                ros_node_instance.stop_mission()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status":"stopped"}')
        else:
            self.send_error(404, "Endpoint Not Found")


class WebVisualizerNode(Node):

    def __init__(self):
        super().__init__('web_visualizer')
        global ros_node_instance
        ros_node_instance = self

        self.packet_counter = 0
        self.last_time = time.time()
        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_vz = 0.0

        # ROS Publishers (depth=1 KEEP_LAST QoS)
        self.joy_pub = self.create_publisher(Joy, '/joy', 1)
        self.arm_pub = self.create_publisher(Bool, '/auv/arm_cmd', 10)
        self.mission_pub = self.create_publisher(Mission, '/auv/mission', 10)
        self.desired_distance_pub = self.create_publisher(Float32, '/auv/desired_distance', 1)
        self.desired_depth_pub = self.create_publisher(Float32, '/auv/desired_depth', 1)
        self.desired_heading_pub = self.create_publisher(Float32, '/auv/desired_heading', 1)
        self.mission_status_pub = self.create_publisher(String, '/auv/mission_status', 10)

        # Service Client for Simulation Reset
        self.reset_client = self.create_client(Trigger, '/auv/sim/reset')

        # Subscriptions
        self.create_subscription(Odometry, '/auv/ideal_state', self.ideal_state_cb, 1)
        self.create_subscription(Orientation, '/auv/orientation', self.orientation_cb, 1)
        self.create_subscription(ActuatorCmds, '/auv/actual_actuator_cmds', self.actual_actuator_cb, 1)
        self.create_subscription(ActuatorCmds, '/auv/actuator_cmds', self.actuator_cb, 1)

        self.create_subscription(Float32, '/auv/thruster_gain', self.gain_cb, 10)
        self.create_subscription(Bool, '/auv/armed_status', self.armed_cb, 10)

        self.create_subscription(String, '/auv/heading_hold_status', self.hh_cb, 10)
        self.create_subscription(String, '/auv/depth_hold_status', self.dh_cb, 10)
        self.create_subscription(String, '/auv/distance_hold_status', self.dist_cb, 10)
        self.create_subscription(String, '/auv/mission_status', self.mission_cb, 10)

        # Autonomy PID Signal Chain Subscriptions (Requirement 12)
        self.create_subscription(Float32, '/auv/depth_cmd', self.depth_cmd_cb, 10)
        self.create_subscription(Float32, '/auv/rudder_cmd', self.rudder_cmd_cb, 10)
        self.create_subscription(Float32, '/auv/throttle_cmd', self.throttle_cmd_cb, 10)
        self.create_subscription(Float32, '/auv/desired_depth', self.target_depth_cb, 10)
        self.create_subscription(Float32, '/auv/desired_heading', self.target_heading_cb, 10)
        self.create_subscription(Float32, '/auv/desired_distance', self.target_distance_cb, 10)
        self.create_subscription(String, '/auv/physics_telemetry', self.physics_cb, 1)
        self.create_subscription(String, '/auv/distance_telemetry', self.dist_telem_cb, 10)
        self.create_subscription(Float32, '/auv/distance_remaining', self.dist_rem_cb, 10)
        self.create_subscription(Float32, '/auv/distance_travelled', self.dist_trav_cb, 10)
        self.create_subscription(String, '/auv/inspection_mission_telemetry', self.inspection_telem_cb, 10)

        # Active controller target subscriptions for unified HUD
        self.create_subscription(Float32, '/auv/active_depth_target', self.active_depth_target_cb, 10)
        self.create_subscription(Float32, '/auv/active_heading_target', self.active_heading_target_cb, 10)
        self.create_subscription(String, '/auv/depth_telemetry', self.depth_telem_cb, 10)
        self.create_subscription(String, '/auv/heading_telemetry', self.heading_telem_cb, 10)

    def publish_joy(self, axes, buttons):
        msg = Joy()
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        self.joy_pub.publish(msg)

    def inspection_telem_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            simulation_state["autonomy"]["inspection_mission"] = data
        except Exception:
            pass

    def start_mission(self, depth, heading, distance, speed=1.0, duration=30.0):
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

        msg = Mission()
        msg.depth = float(depth)
        msg.heading = float(heading)
        msg.distance = float(distance)
        msg.inspection_speed = float(speed)
        msg.duration = float(duration)
        msg.start = True
        msg.cancel = False
        self.mission_pub.publish(msg)
        self.get_logger().info(f"ARMED & MISSION STARTED from Web UI: Depth={depth}m, Heading={heading}deg, Speed={speed}m/s, Duration={duration}min")

    def stop_mission(self):
        msg = Mission()
        msg.cancel = True
        msg.start = False
        self.mission_pub.publish(msg)

        dist_msg = Float32()
        dist_msg.data = 0.0
        self.desired_distance_pub.publish(dist_msg)

        self.get_logger().info("MISSION STOP COMMAND SENT from Web UI.")

    def reset_simulation(self):
        if self.reset_client.service_is_ready():
            req = Trigger.Request()
            self.reset_client.call_async(req)
        
        status_msg = String()
        status_msg.data = "IDLE"
        self.mission_status_pub.publish(status_msg)

        dist_msg = Float32()
        dist_msg.data = 0.0
        self.desired_distance_pub.publish(dist_msg)

        simulation_state["autonomy"]["target_distance"] = 0.0
        simulation_state["autonomy"]["current_distance"] = 0.0
        simulation_state["autonomy"]["distance_remaining"] = 0.0
        simulation_state["autonomy"]["throttle_cmd"] = 0.0

        self.get_logger().info("SIMULATION RESET triggered from Web UI.")

    def physics_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            simulation_state["physics"].update(data)
        except Exception:
            pass

    def dist_rem_cb(self, msg: Float32):
        simulation_state["autonomy"]["distance_remaining"] = float(msg.data)

    def dist_trav_cb(self, msg: Float32):
        simulation_state["autonomy"]["current_distance"] = float(msg.data)

    def dist_telem_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            simulation_state["autonomy"]["target_distance"] = float(data.get("target", 0.0))
            simulation_state["autonomy"]["current_distance"] = float(data.get("travelled", 0.0))
            simulation_state["autonomy"]["distance_remaining"] = float(data.get("remaining", 0.0))
            simulation_state["autonomy"]["throttle_cmd"] = float(data.get("output", 0.0))
        except Exception:
            pass

    def depth_cmd_cb(self, msg: Float32):
        simulation_state["autonomy"]["depth_cmd"] = float(msg.data)

    def rudder_cmd_cb(self, msg: Float32):
        simulation_state["autonomy"]["rudder_cmd"] = float(msg.data)

    def throttle_cmd_cb(self, msg: Float32):
        simulation_state["autonomy"]["throttle_cmd"] = float(msg.data)

    def target_depth_cb(self, msg: Float32):
        simulation_state["autonomy"]["target_depth"] = float(msg.data)
        simulation_state["autonomy"]["active_depth_target"] = float(msg.data)

    def target_heading_cb(self, msg: Float32):
        simulation_state["autonomy"]["target_heading"] = float(msg.data)
        simulation_state["autonomy"]["active_heading_target"] = float(msg.data)

    def target_distance_cb(self, msg: Float32):
        simulation_state["autonomy"]["target_distance"] = float(msg.data)

    def active_depth_target_cb(self, msg: Float32):
        simulation_state["autonomy"]["active_depth_target"] = float(msg.data)
        simulation_state["autonomy"]["target_depth"] = float(msg.data)

    def active_heading_target_cb(self, msg: Float32):
        simulation_state["autonomy"]["active_heading_target"] = float(msg.data)
        simulation_state["autonomy"]["target_heading"] = float(msg.data)

    def depth_telem_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            tgt = float(data.get("target", 0.0))
            simulation_state["autonomy"]["active_depth_target"] = tgt
            simulation_state["autonomy"]["target_depth"] = tgt
            simulation_state["autonomy"]["depth_cmd"] = float(data.get("output", 0.0))
            simulation_state["autonomy"]["depth_error"] = float(data.get("error", 0.0))
        except Exception:
            pass

    def heading_telem_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            tgt = float(data.get("target", 0.0))
            simulation_state["autonomy"]["active_heading_target"] = tgt
            simulation_state["autonomy"]["target_heading"] = tgt
            simulation_state["autonomy"]["rudder_cmd"] = float(data.get("output", 0.0))
            simulation_state["autonomy"]["heading_error"] = float(data.get("error", 0.0))
        except Exception:
            pass

    def ideal_state_cb(self, msg: Odometry):
        self.packet_counter += 1
        now = time.time()
        dt = max(0.001, now - self.last_time)
        self.last_time = now

        simulation_state["packets"] = self.packet_counter

        simulation_state["position"]["x"] = float(msg.x)
        simulation_state["position"]["y"] = float(msg.y)
        simulation_state["position"]["z"] = float(msg.z)

        simulation_state["orientation"]["roll"] = float(msg.roll)
        simulation_state["orientation"]["pitch"] = float(msg.pitch)
        simulation_state["orientation"]["yaw"] = float(msg.yaw)

        simulation_state["autonomy"]["current_depth"] = float(msg.z)
        simulation_state["autonomy"]["current_heading"] = float(msg.yaw)

        vx = float(msg.vx)
        vy = float(msg.vy)
        vz = float(msg.vz)

        simulation_state["velocities"]["vx"] = vx
        simulation_state["velocities"]["vy"] = vy
        simulation_state["velocities"]["vz"] = vz

        # Calculate IMU Acceleration
        ax = (vx - self.prev_vx) / dt
        ay = (vy - self.prev_vy) / dt
        az = ((vz - self.prev_vz) / dt) - 9.81

        self.prev_vx = vx
        self.prev_vy = vy
        self.prev_vz = vz

        simulation_state["accel"]["ax"] = ax
        simulation_state["accel"]["ay"] = ay
        simulation_state["accel"]["az"] = az

        # Determine World Motion Status
        if vx > 0.05:
            simulation_state["world_motion"] = "FORWARD"
        elif vx < -0.05:
            simulation_state["world_motion"] = "REVERSE"
        elif abs(vy) > 0.05:
            simulation_state["world_motion"] = "SWAY"
        elif vz > 0.05:
            simulation_state["world_motion"] = "DIVING"
        elif vz < -0.05:
            simulation_state["world_motion"] = "ASCENDING"
        else:
            simulation_state["world_motion"] = "NONE"

    def orientation_cb(self, msg: Orientation):
        simulation_state["orientation"]["roll"] = float(msg.roll)
        simulation_state["orientation"]["pitch"] = float(msg.pitch)
        simulation_state["orientation"]["yaw"] = float(msg.yaw)
        simulation_state["autonomy"]["current_heading"] = float(msg.yaw)

    def actuator_cb(self, msg: ActuatorCmds):
        simulation_state["actuators"]["elevator_left"] = int(msg.elevator_left)
        simulation_state["actuators"]["elevator_right"] = int(msg.elevator_right)
        simulation_state["actuators"]["rudder_left"] = int(msg.rudder_left)
        simulation_state["actuators"]["rudder_right"] = int(msg.rudder_right)
        simulation_state["actuators"]["main_thruster"] = int(msg.main_thruster)

    def actual_actuator_cb(self, msg: ActuatorCmds):
        simulation_state["actual_actuators"]["elevator_left"] = int(msg.elevator_left)
        simulation_state["actual_actuators"]["elevator_right"] = int(msg.elevator_right)
        simulation_state["actual_actuators"]["rudder_left"] = int(msg.rudder_left)
        simulation_state["actual_actuators"]["rudder_right"] = int(msg.rudder_right)
        simulation_state["actual_actuators"]["main_thruster"] = int(msg.main_thruster)

    def gain_cb(self, msg: Float32):
        simulation_state["status"]["gain"] = int(round(msg.data * 100))

    def armed_cb(self, msg: Bool):
        simulation_state["status"]["armed"] = msg.data

    def hh_cb(self, msg: String):
        simulation_state["status"]["heading_hold"] = msg.data
        if msg.data.startswith("ON:"):
            try:
                tgt = float(msg.data.split(":")[1])
                simulation_state["autonomy"]["active_heading_target"] = tgt
                simulation_state["autonomy"]["target_heading"] = tgt
            except Exception:
                pass

    def dh_cb(self, msg: String):
        simulation_state["status"]["depth_hold"] = msg.data
        if msg.data.startswith("ON:"):
            try:
                tgt = float(msg.data.split(":")[1])
                simulation_state["autonomy"]["active_depth_target"] = tgt
                simulation_state["autonomy"]["target_depth"] = tgt
            except Exception:
                pass

    def dist_cb(self, msg: String):
        simulation_state["status"]["distance_hold"] = msg.data

    def mission_cb(self, msg: String):
        simulation_state["status"]["mission"] = msg.data


def run_http_server():
    try:
        server_address = ('0.0.0.0', 8080)
        httpd = ThreadedHTTPServer(server_address, WebHTTPRequestHandler)
        httpd.serve_forever()
    except Exception as e:
        print(f"[web_visualizer] HTTP server error: {e}", file=sys.stderr, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = WebVisualizerNode()

    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()

    msg_1 = "[web_visualizer] AUV 3D Web Viewer & Joystick Bridge Started"
    msg_2 = "[web_visualizer] Open browser:"
    msg_3 = "http://localhost:8080"

    print(msg_1, flush=True)
    print(msg_2, flush=True)
    print(msg_3, flush=True)

    node.get_logger().info(msg_1)
    node.get_logger().info(msg_2)
    node.get_logger().info(msg_3)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

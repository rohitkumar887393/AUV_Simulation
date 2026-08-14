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
        "throttle_cmd": 0.0
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

        /* Top HUD Bar */
        #top-hud-bar {
            position: absolute;
            top: 0;
            left: 0;
            width: 100vw;
            height: 48px;
            background: rgba(3, 14, 28, 0.92);
            border-bottom: 2px solid #00a8e8;
            box-shadow: 0 4px 20px rgba(0, 168, 232, 0.25);
            display: flex;
            align-items: center;
            padding: 0 20px;
            gap: 14px;
            z-index: 10;
            font-size: 13px;
            letter-spacing: 1px;
        }

        .hud-pill {
            display: flex;
            align-items: center;
            gap: 8px;
            padding: 4px 12px;
            border-radius: 14px;
            border: 1px solid #1c3d5a;
            background: rgba(10, 25, 47, 0.7);
        }

        .dot-indicator {
            width: 9px;
            height: 9px;
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
            width: 70px;
            height: 8px;
            background: #0d2238;
            border-radius: 4px;
            overflow: hidden;
            border: 1px solid #00a8e8;
        }

        .gain-bar-fill {
            height: 100%;
            width: 75%;
            background: linear-gradient(90deg, #0077b6, #00f5d4);
            box-shadow: 0 0 8px #00f5d4;
        }

        .badge-box {
            padding: 3px 10px;
            border-radius: 4px;
            border: 1px solid #00a8e8;
            background: rgba(0, 168, 232, 0.15);
            color: #00f5d4;
            font-weight: bold;
        }

        .badge-disabled {
            border-color: #3a506b;
            color: #4a6572;
            background: rgba(13, 27, 42, 0.5);
        }

        .badge-enabled {
            border-color: #00f5d4;
            color: #00f5d4;
            background: rgba(0, 245, 212, 0.18);
            box-shadow: 0 0 8px rgba(0, 245, 212, 0.3);
        }

        /* Left HUD Telemetry Panel */
        #telemetry-panel {
            position: absolute;
            top: 60px;
            left: 15px;
            width: 320px;
            background: rgba(3, 14, 28, 0.92);
            border: 1px solid #00a8e8;
            border-radius: 6px;
            padding: 12px 14px;
            box-shadow: 0 0 25px rgba(0, 0, 0, 0.7);
            z-index: 10;
        }

        .panel-header {
            font-size: 13px;
            color: #00f5d4;
            border-bottom: 1px solid #1c3d5a;
            padding-bottom: 5px;
            margin-bottom: 8px;
            letter-spacing: 1.5px;
            display: flex;
            justify-content: space-between;
        }

        .telem-row {
            display: flex;
            justify-content: space-between;
            margin-bottom: 4px;
            font-size: 12px;
        }

        .telem-label { color: #5c7d99; }
        .telem-val { color: #ffffff; font-weight: bold; }
        .telem-val-highlight { color: #ffb703; font-weight: bold; }

        /* Crosshair Pitch/Roll Dial */
        #crosshair-box {
            position: absolute;
            top: 15px;
            right: 12px;
            width: 60px;
            height: 60px;
            border: 1px solid #00a8e8;
            border-radius: 50%;
            background: rgba(10, 25, 47, 0.8);
            display: flex;
            align-items: center;
            justify-content: center;
        }

        #crosshair-dot {
            width: 7px;
            height: 7px;
            background: #ef233c;
            border-radius: 50%;
            box-shadow: 0 0 8px #ef233c;
            position: absolute;
        }

        /* Autonomy Closed-Loop PID HUD Panel (Requirement 12) */
        #autonomy-pid-panel {
            position: absolute;
            top: 360px;
            left: 15px;
            width: 330px;
            background: rgba(3, 14, 28, 0.94);
            border: 1.5px solid #00f5d4;
            border-radius: 6px;
            padding: 10px 14px;
            box-shadow: 0 0 25px rgba(0, 245, 212, 0.2);
            z-index: 10;
            font-size: 12px;
        }

        .hud-section-header {
            color: #00f5d4;
            font-size: 11px;
            font-weight: bold;
            margin-top: 6px;
            margin-bottom: 3px;
            letter-spacing: 1px;
        }

        .hud-divider-line {
            border-bottom: 1px solid #1c3d5a;
            margin: 5px 0;
        }

        /* Physics Debug Data Panel (Requirement 13) */
        #physics-debug-panel {
            position: absolute;
            top: 290px;
            right: 15px;
            width: 310px;
            background: rgba(3, 14, 28, 0.94);
            border: 1px solid #ffb703;
            border-radius: 6px;
            padding: 10px 14px;
            box-shadow: 0 0 25px rgba(255, 183, 3, 0.2);
            z-index: 10;
            font-size: 12px;
        }

        /* Bottom Propulsion Panel */
        #propulsion-panel {
            position: absolute;
            bottom: 15px;
            left: 15px;
            width: 330px;
            background: rgba(3, 14, 28, 0.92);
            border: 1px solid #00a8e8;
            border-radius: 6px;
            padding: 10px 14px;
            box-shadow: 0 0 25px rgba(0, 0, 0, 0.7);
            z-index: 10;
        }

        .pwm-bar-wrapper {
            margin-bottom: 6px;
        }

        .pwm-label-row {
            display: flex;
            justify-content: space-between;
            font-size: 11px;
            margin-bottom: 2px;
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
        }

        /* Mission Control Panel */
        #mission-panel {
            position: absolute;
            top: 60px;
            right: 15px;
            width: 310px;
            background: rgba(3, 14, 28, 0.92);
            border: 1px solid #00a8e8;
            border-radius: 6px;
            padding: 12px 14px;
            box-shadow: 0 0 25px rgba(0, 0, 0, 0.7);
            z-index: 10;
        }

        .mission-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
            font-size: 12px;
        }

        .mission-input {
            width: 90px;
            background: #0a192f;
            border: 1px solid #00a8e8;
            color: #00f5d4;
            padding: 4px 8px;
            border-radius: 4px;
            font-family: inherit;
            font-size: 12px;
            text-align: right;
        }

        .mission-btn-row {
            display: flex;
            gap: 6px;
            margin-top: 10px;
            margin-bottom: 10px;
        }

        .mission-btn {
            flex: 1;
            padding: 8px 4px;
            border-radius: 4px;
            font-family: inherit;
            font-size: 11px;
            font-weight: bold;
            cursor: pointer;
            border: 1px solid transparent;
            transition: all 0.2s;
        }

        .btn-start {
            background: rgba(0, 245, 212, 0.2);
            color: #00f5d4;
            border-color: #00f5d4;
        }
        .btn-start:hover {
            background: #00f5d4;
            color: #030a16;
            box-shadow: 0 0 10px #00f5d4;
        }

        .btn-stop {
            background: rgba(217, 4, 41, 0.2);
            color: #ef233c;
            border-color: #d90429;
        }
        .btn-stop:hover {
            background: #d90429;
            color: #ffffff;
            box-shadow: 0 0 10px #d90429;
        }

        .btn-reset {
            background: rgba(255, 183, 3, 0.2);
            color: #ffb703;
            border-color: #ffb703;
        }
        .btn-reset:hover {
            background: #ffb703;
            color: #030a16;
            box-shadow: 0 0 10px #ffb703;
        }

        .mission-status-row {
            display: flex;
            justify-content: space-between;
            font-size: 12px;
            padding-top: 6px;
            border-top: 1px dashed #1c3d5a;
        }

        .mission-status-val {
            font-weight: bold;
            color: #ffb703;
        }

        /* Camera Controls */
        #cam-mode-box {
            position: absolute;
            bottom: 15px;
            right: 15px;
            background: rgba(3, 14, 28, 0.92);
            border: 1px solid #00a8e8;
            border-radius: 6px;
            padding: 8px 12px;
            display: flex;
            gap: 6px;
            z-index: 10;
        }

        .cam-btn {
            background: rgba(0, 119, 182, 0.4);
            color: #00f5d4;
            border: 1px solid #00a8e8;
            padding: 5px 10px;
            border-radius: 4px;
            cursor: pointer;
            font-family: inherit;
            font-size: 11px;
            transition: all 0.2s;
        }

        .cam-btn:hover {
            background: #00b4d8;
            color: #030a16;
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
        <div class="hud-pill">
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

        <span class="hud-divider">|</span>

        <div class="badge-box badge-disabled" id="badge-depth-hold">DEPTH HOLD: OFF</div>
        <div class="badge-box badge-disabled" id="badge-heading-hold">HEADING HOLD: OFF</div>
        <div class="badge-box badge-enabled" id="badge-roll-stab">ROLL STABILIZER: ACTIVE</div>

        <span class="hud-divider">|</span>
        <div class="badge-box badge-disabled" id="badge-joy">JOYSTICK: CONNECTED</div>
    </div>

    <!-- Left Telemetry Panel -->
    <div id="telemetry-panel">
        <div class="panel-header">
            <span>TELEMETRY DATA</span>
            <span style="font-size: 11px; color: #8d99ae;" id="txt-packets">0 pkts</span>
        </div>

        <div style="position: relative;">
            <div id="crosshair-box">
                <div id="crosshair-dot"></div>
            </div>

            <div class="telem-row"><span class="telem-label">ROLL:</span> <span class="telem-val" id="val-roll">+0.0°</span></div>
            <div class="telem-row"><span class="telem-label">PITCH:</span> <span class="telem-val" id="val-pitch">+0.0°</span></div>
            <div class="telem-row"><span class="telem-label">YAW:</span> <span class="telem-val-highlight" id="val-yaw">+0.0°</span></div>

            <div style="height: 6px;"></div>

            <div class="telem-row"><span class="telem-label">SPEED X:</span> <span class="telem-val" id="val-vx">+0.0 m/s</span></div>
            <div class="telem-row"><span class="telem-label">SPEED Y:</span> <span class="telem-val" id="val-vy">+0.0 m/s</span></div>
            <div class="telem-row"><span class="telem-label">SPEED Z:</span> <span class="telem-val" id="val-vz">+0.0 m/s</span></div>

            <div style="height: 6px;"></div>

            <div class="telem-row"><span class="telem-label">DEPTH:</span> <span class="telem-val-highlight" id="val-depth">0.50 m</span></div>
            <div class="telem-row"><span class="telem-label">LIN VELOCITY:</span> <span class="telem-val" id="val-lin-vel">[+0.0, +0.0, +0.0] m/s</span></div>
            <div class="telem-row"><span class="telem-label">IMU ACCEL:</span> <span class="telem-val" id="val-imu-accel">[+0.0, +0.0, -9.8] m/s²</span></div>
            <div class="telem-row"><span class="telem-label">WORLD MOTION:</span> <span class="telem-val-highlight" id="val-motion">NONE</span></div>
        </div>
    </div>

    <!-- Autonomy Closed-Loop PID HUD Panel (Requirement 12) -->
    <div id="autonomy-pid-panel">
        <div class="panel-header" style="color: #00f5d4; border-color: #00f5d4; font-size: 13px;">AUTONOMY CLOSED-LOOP SIGNAL CHAIN</div>
        
        <div class="hud-section-header">DEPTH</div>
        <div class="telem-row"><span class="telem-label">TARGET:</span> <span class="telem-val" id="pid-depth-tgt">2.00 m</span></div>
        <div class="telem-row"><span class="telem-label">CURRENT:</span> <span class="telem-val" id="pid-depth-cur">0.50 m</span></div>
        <div class="telem-row"><span class="telem-label">ERROR:</span> <span class="telem-val-highlight" id="pid-depth-err">+1.50 m</span></div>
        <div class="telem-row"><span class="telem-label">PID OUTPUT:</span> <span class="telem-val" id="pid-depth-out">+0.0</span></div>

        <div class="hud-divider-line"></div>

        <div class="hud-section-header">HEADING</div>
        <div class="telem-row"><span class="telem-label">TARGET:</span> <span class="telem-val" id="pid-head-tgt">90.0°</span></div>
        <div class="telem-row"><span class="telem-label">CURRENT:</span> <span class="telem-val" id="pid-head-cur">0.0°</span></div>
        <div class="telem-row"><span class="telem-label">ERROR:</span> <span class="telem-val-highlight" id="pid-head-err">+90.0°</span></div>
        <div class="telem-row"><span class="telem-label">PID OUTPUT:</span> <span class="telem-val" id="pid-head-out">+0.0</span></div>

        <div class="hud-divider-line"></div>

        <div class="hud-section-header">DISTANCE</div>
        <div class="telem-row"><span class="telem-label">TARGET:</span> <span class="telem-val" id="pid-dist-tgt">10.0 m</span></div>
        <div class="telem-row"><span class="telem-label">TRAVELLED:</span> <span class="telem-val" id="pid-dist-trav">0.0 m</span></div>
        <div class="telem-row"><span class="telem-label">REMAINING:</span> <span class="telem-val-highlight" id="pid-dist-rem">10.0 m</span></div>
        <div class="telem-row"><span class="telem-label">PID OUTPUT:</span> <span class="telem-val" id="pid-dist-out">0.00</span></div>

        <div class="hud-divider-line"></div>

        <div class="hud-section-header">ACTUATOR OUTPUT</div>
        <div class="telem-row"><span class="telem-label">THRUSTER CMD:</span> <span class="telem-val" id="act-thr-cmd">1500 us</span></div>
        <div class="telem-row"><span class="telem-label">ELEVATOR CMD:</span> <span class="telem-val" id="act-elev-cmd">1500 us</span></div>
        <div class="telem-row"><span class="telem-label">RUDDER CMD:</span> <span class="telem-val" id="act-rud-cmd">1500 us</span></div>

        <div class="hud-divider-line"></div>

        <div class="hud-section-header">PHYSICAL STATE</div>
        <div class="telem-row"><span class="telem-label">FORWARD SPEED:</span> <span class="telem-val" id="phys-fwd-speed">0.00 m/s</span></div>
        <div class="telem-row"><span class="telem-label">VERTICAL SPEED:</span> <span class="telem-val" id="phys-vert-speed">0.00 m/s</span></div>
        <div class="telem-row"><span class="telem-label">PITCH:</span> <span class="telem-val" id="phys-pitch">0.0°</span></div>
        <div class="telem-row"><span class="telem-label">YAW:</span> <span class="telem-val" id="phys-yaw">0.0°</span></div>
        <div class="telem-row"><span class="telem-label">ROLL:</span> <span class="telem-val" id="phys-roll">0.0°</span></div>
        <div class="telem-row"><span class="telem-label">DEPTH:</span> <span class="telem-val" id="phys-depth">0.50 m</span></div>

        <div class="hud-divider-line"></div>

        <div style="font-size: 10px; color: #00f5d4; text-align: center; font-weight: bold; margin-top: 4px; letter-spacing: 0.5px;">
            MISSION → TARGET → PID → ACTUATOR → VEHICLE → SENSOR → PID FEEDBACK
        </div>
    </div>

    <!-- Physics Debug Data Panel (Requirement 13) -->
    <div id="physics-debug-panel">
        <div class="panel-header" style="color: #ffb703; border-color: #ffb703; font-size: 12px;">PHYSICS DEBUG TELEMETRY</div>
        <div class="telem-row"><span class="telem-label">BUOYANCY FORCE:</span> <span class="telem-val-highlight" id="phys-buoyancy">+12.3 N</span></div>
        <div class="telem-row"><span class="telem-label">THRUST FORCE:</span> <span class="telem-val" id="phys-thrust">+0.0 N</span></div>
        <div class="telem-row"><span class="telem-label">FORWARD DRAG:</span> <span class="telem-val" id="phys-drag">-0.0 N</span></div>
        <div class="telem-row"><span class="telem-label">DYNAMIC PRESS:</span> <span class="telem-val-highlight" id="phys-qdyn">0.0 Pa</span></div>
        <div class="telem-row"><span class="telem-label">PITCH MOMENT:</span> <span class="telem-val" id="phys-mpitch">0.00 Nm</span></div>
        <div class="telem-row"><span class="telem-label">YAW MOMENT:</span> <span class="telem-val" id="phys-nyaw">0.00 Nm</span></div>
        <div class="telem-row"><span class="telem-label">ROLL MOMENT:</span> <span class="telem-val" id="phys-lroll">0.00 Nm</span></div>
    </div>

    <!-- Bottom Propulsion Panel -->
    <div id="propulsion-panel">
        <div class="panel-header" style="font-size: 12px;">PROPULSION & CONTROL SURFACES</div>

        <div class="pwm-bar-wrapper">
            <div class="pwm-label-row">
                <span class="telem-label">MAIN THRUSTER PWM:</span>
                <span class="telem-val" id="val-thr-pwm">1500 us (+0%)</span>
            </div>
            <div class="pwm-track"><div class="pwm-fill" id="fill-thr-pwm"></div></div>
        </div>

        <div class="pwm-bar-wrapper">
            <div class="pwm-label-row">
                <span class="telem-label">ELEVATOR PWM:</span>
                <span class="telem-val" id="val-elev-pwm">1500 us (+0%)</span>
            </div>
            <div class="pwm-track"><div class="pwm-fill" id="fill-elev-pwm"></div></div>
        </div>

        <div class="pwm-bar-wrapper">
            <div class="pwm-label-row">
                <span class="telem-label">RUDDER PWM:</span>
                <span class="telem-val" id="val-rud-pwm">1500 us (+0%)</span>
            </div>
            <div class="pwm-track"><div class="pwm-fill" id="fill-rud-pwm"></div></div>
        </div>
    </div>

    <!-- Mission Control Panel -->
    <div id="mission-panel">
        <div class="panel-header">MISSION CONTROL</div>

        <div class="mission-row">
            <span class="telem-label">Target Depth:</span>
            <span><input type="number" id="input-depth" step="0.1" value="2.00" class="mission-input"> m</span>
        </div>
        <div class="mission-row">
            <span class="telem-label">Target Heading:</span>
            <span><input type="number" id="input-heading" step="1.0" value="90.0" class="mission-input"> deg</span>
        </div>
        <div class="mission-row">
            <span class="telem-label">Target Distance:</span>
            <span><input type="number" id="input-distance" step="0.5" value="5.00" class="mission-input"> m</span>
        </div>

        <div class="mission-btn-row">
            <button class="mission-btn btn-start" onclick="startMission()">START MISSION</button>
            <button class="mission-btn btn-stop" onclick="stopMission()">STOP MISSION</button>
            <button class="mission-btn btn-reset" onclick="resetSim()">RESET</button>
        </div>

        <div class="mission-status-row">
            <span class="telem-label">Mission Status:</span>
            <span id="txt-mission-status" class="mission-status-val">IDLE</span>
        </div>
    </div>

    <!-- Camera Mode Buttons (Non-control) -->
    <div id="cam-mode-box">
        <button class="cam-btn" onclick="setCameraMode('orbit')">Free Orbit</button>
        <button class="cam-btn" onclick="setCameraMode('follow')">Follow AUV</button>
        <button class="cam-btn" onclick="setCameraMode('top')">Top View</button>
        <button class="cam-btn" onclick="setCameraMode('side')">Side View</button>
    </div>

    <script>
        let scene, camera, renderer, controls;
        let auvGroup, trajectoryLine, trajectoryGeo;
        let trajectoryPoints = [];
        let maxTrajectoryPoints = 600;
        let cameraMode = 'orbit';
        let latestState = null;

        function init3D() {
            const container = document.getElementById('canvas-container');

            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x030a16);
            scene.fog = new THREE.FogExp2(0x030a16, 0.02);

            camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
            camera.position.set(8, 5, 8);

            renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.setSize(window.innerWidth, window.innerHeight);
            renderer.setPixelRatio(window.devicePixelRatio);
            container.appendChild(renderer.domElement);

            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.05;

            // Lights
            const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
            scene.add(ambientLight);
            const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
            dirLight.position.set(10, 20, 10);
            scene.add(dirLight);

            // Water Surface Plane (Depth Z=0m -> Three.js Y=0)
            const surfaceGeo = new THREE.GridHelper(100, 50, 0x00a8e8, 0x002244);
            surfaceGeo.position.y = 0;
            scene.add(surfaceGeo);

            // Seabed Plane (Depth Z=20m -> Three.js Y=-20)
            const seabedGeo = new THREE.GridHelper(100, 50, 0x8b5a2b, 0x3d2314);
            seabedGeo.position.y = -20;
            scene.add(seabedGeo);

            // Origin Marker (Surface origin)
            const originGeo = new THREE.SphereGeometry(0.15, 16, 16);
            const originMat = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
            scene.add(new THREE.Mesh(originGeo, originMat));

            // Trajectory Line
            trajectoryGeo = new THREE.BufferGeometry();
            const trajectoryMat = new THREE.LineBasicMaterial({ color: 0x00f5d4, linewidth: 2 });
            trajectoryLine = new THREE.Line(trajectoryGeo, trajectoryMat);
            scene.add(trajectoryLine);

            // Create 3D AUV Mesh Group
            auvGroup = new THREE.Group();

            // Main Cylindrical Body
            const bodyGeo = new THREE.CylinderGeometry(0.18, 0.18, 1.2, 16);
            bodyGeo.rotateZ(-Math.PI / 2);
            const bodyMat = new THREE.MeshStandardMaterial({ color: 0xd90429, roughness: 0.3 });
            auvGroup.add(new THREE.Mesh(bodyGeo, bodyMat));

            // Nose Cone (Front +X)
            const noseGeo = new THREE.ConeGeometry(0.18, 0.4, 16);
            noseGeo.rotateZ(-Math.PI / 2);
            noseGeo.translate(0.8, 0, 0);
            const noseMat = new THREE.MeshStandardMaterial({ color: 0xef233c, roughness: 0.3 });
            auvGroup.add(new THREE.Mesh(noseGeo, noseMat));

            // Rear Tail Section
            const tailGeo = new THREE.ConeGeometry(0.18, 0.3, 16);
            tailGeo.rotateZ(Math.PI / 2);
            tailGeo.translate(-0.75, 0, 0);
            const tailMat = new THREE.MeshStandardMaterial({ color: 0x1c3d5a, roughness: 0.5 });
            auvGroup.add(new THREE.Mesh(tailGeo, tailMat));

            // Four Fins
            const finMat = new THREE.MeshStandardMaterial({ color: 0xffb703, side: THREE.DoubleSide });

            const elevLeftGeo = new THREE.PlaneGeometry(0.25, 0.3);
            elevLeftGeo.translate(-0.6, 0.25, 0);
            auvGroup.add(new THREE.Mesh(elevLeftGeo, finMat));

            const elevRightGeo = new THREE.PlaneGeometry(0.25, 0.3);
            elevRightGeo.translate(-0.6, -0.25, 0);
            auvGroup.add(new THREE.Mesh(elevRightGeo, finMat));

            const rudTopGeo = new THREE.PlaneGeometry(0.25, 0.3);
            rudTopGeo.rotateX(Math.PI / 2);
            rudTopGeo.translate(-0.6, 0, 0.25);
            auvGroup.add(new THREE.Mesh(rudTopGeo, finMat));

            const rudBottomGeo = new THREE.PlaneGeometry(0.25, 0.3);
            rudBottomGeo.rotateX(Math.PI / 2);
            rudBottomGeo.translate(-0.6, 0, -0.25);
            auvGroup.add(new THREE.Mesh(rudBottomGeo, finMat));

            // Rear Thruster
            const thrusterGeo = new THREE.CylinderGeometry(0.08, 0.08, 0.15, 12);
            thrusterGeo.rotateZ(-Math.PI / 2);
            thrusterGeo.translate(-0.9, 0, 0);
            const thrusterMat = new THREE.MeshStandardMaterial({ color: 0x8d99ae });
            auvGroup.add(new THREE.Mesh(thrusterGeo, thrusterMat));

            // Heading Vector Indicator (Cyan Arrow)
            const arrowHelper = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(1.0, 0, 0), 0.8, 0x00f5d4, 0.2, 0.1);
            auvGroup.add(arrowHelper);

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

            // 3. Mode Badges
            const hhStatus = data.status.heading_hold;
            const dhStatus = data.status.depth_hold;
            const distStatus = data.status.distance_hold;

            const badgeMode = document.getElementById('badge-mode');
            if (hhStatus !== 'OFF' || dhStatus !== 'OFF' || distStatus !== 'OFF' || data.status.mission !== 'IDLE') {
                badgeMode.innerText = 'AUTONOMY';
            } else {
                badgeMode.innerText = 'MANUAL';
            }

            const badgeDH = document.getElementById('badge-depth-hold');
            if (dhStatus !== 'OFF') {
                badgeDH.className = 'badge-box badge-enabled';
                badgeDH.innerText = `DEPTH HOLD: ${dhStatus}`;
            } else {
                badgeDH.className = 'badge-box badge-disabled';
                badgeDH.innerText = 'DEPTH HOLD: OFF';
            }

            const badgeHH = document.getElementById('badge-heading-hold');
            if (hhStatus !== 'OFF') {
                badgeHH.className = 'badge-box badge-enabled';
                badgeHH.innerText = `HEADING HOLD: ${hhStatus}`;
            } else {
                badgeHH.className = 'badge-box badge-disabled';
                badgeHH.innerText = 'HEADING HOLD: OFF';
            }

            document.getElementById('txt-mission-status').innerText = data.status.mission || 'IDLE';

            // 4. Telemetry Data
            const roll = data.orientation.roll;
            const pitch = data.orientation.pitch;
            const yaw = data.orientation.yaw;

            document.getElementById('val-roll').innerText = `${roll >= 0 ? '+' : ''}${roll.toFixed(1)}°`;
            document.getElementById('val-pitch').innerText = `${pitch >= 0 ? '+' : ''}${pitch.toFixed(1)}°`;
            document.getElementById('val-yaw').innerText = `${yaw >= 0 ? '+' : ''}${yaw.toFixed(1)}°`;

            const vx = data.velocities.vx;
            const vy = data.velocities.vy;
            const vz = data.velocities.vz;

            document.getElementById('val-vx').innerText = `${vx >= 0 ? '+' : ''}${vx.toFixed(2)} m/s`;
            document.getElementById('val-vy').innerText = `${vy >= 0 ? '+' : ''}${vy.toFixed(2)} m/s`;
            document.getElementById('val-vz').innerText = `${vz >= 0 ? '+' : ''}${vz.toFixed(2)} m/s`;

            const depth = data.position.z;
            document.getElementById('val-depth').innerText = `${depth.toFixed(2)} m`;
            document.getElementById('val-lin-vel').innerText = `[${vx >= 0 ? '+' : ''}${vx.toFixed(1)}, ${vy >= 0 ? '+' : ''}${vy.toFixed(1)}, ${vz >= 0 ? '+' : ''}${vz.toFixed(1)}] m/s`;

            const ax = data.accel ? data.accel.ax : 0.0;
            const ay = data.accel ? data.accel.ay : 0.0;
            const az = data.accel ? data.accel.az : -9.81;
            document.getElementById('val-imu-accel').innerText = `[${ax >= 0 ? '+' : ''}${ax.toFixed(1)}, ${ay >= 0 ? '+' : ''}${ay.toFixed(1)}, ${az.toFixed(1)}] m/s²`;

            document.getElementById('txt-packets').innerText = `${data.packets || 0} pkts`;
            document.getElementById('val-motion').innerText = data.world_motion || 'NONE';

            // 5. Autonomy Closed-Loop PID HUD (Requirement 12)
            const aut = data.autonomy || {};
            const tgtDepth = aut.target_depth || 0.0;
            const curDepth = depth;
            const depthErr = tgtDepth - curDepth;
            const depthCmd = aut.depth_cmd || 0.0;

            const tgtHead = aut.target_heading || 0.0;
            const curHead = yaw;
            let headErr = tgtHead - curHead;
            while (headErr > 180) headErr -= 360;
            while (headErr < -180) headErr += 360;
            const rudCmd = aut.rudder_cmd || 0.0;

            const tgtDist = aut.target_distance || 0.0;
            const distTrav = aut.current_distance !== undefined ? aut.current_distance : 0.0;
            const distRem = aut.distance_remaining !== undefined ? aut.distance_remaining : Math.max(0.0, tgtDist - distTrav);
            const thrCmd = aut.throttle_cmd || 0.0;

            const thrPWM = data.actuators.main_thruster;
            const elevPWM = data.actuators.elevator_left;
            const rudPWM = data.actuators.rudder_left;

            document.getElementById('pid-depth-tgt').innerText = `${tgtDepth.toFixed(2)} m`;
            document.getElementById('pid-depth-cur').innerText = `${curDepth.toFixed(2)} m`;
            document.getElementById('pid-depth-err').innerText = `${depthErr >= 0 ? '+' : ''}${depthErr.toFixed(2)} m`;
            document.getElementById('pid-depth-out').innerText = `${depthCmd >= 0 ? '+' : ''}${depthCmd.toFixed(1)}`;

            document.getElementById('pid-head-tgt').innerText = `${tgtHead.toFixed(1)}°`;
            document.getElementById('pid-head-cur').innerText = `${curHead.toFixed(1)}°`;
            document.getElementById('pid-head-err').innerText = `${headErr >= 0 ? '+' : ''}${headErr.toFixed(1)}°`;
            document.getElementById('pid-head-out').innerText = `${rudCmd >= 0 ? '+' : ''}${rudCmd.toFixed(1)}`;

            document.getElementById('pid-dist-tgt').innerText = `${tgtDist.toFixed(1)} m`;
            document.getElementById('pid-dist-trav').innerText = `${distTrav.toFixed(1)} m`;
            document.getElementById('pid-dist-rem').innerText = `${distRem.toFixed(1)} m`;
            document.getElementById('pid-dist-out').innerText = `${thrCmd.toFixed(2)}`;

            document.getElementById('act-thr-cmd').innerText = `${thrPWM} us`;
            document.getElementById('act-elev-cmd').innerText = `${elevPWM} us`;
            document.getElementById('act-rud-cmd').innerText = `${rudPWM} us`;

            document.getElementById('phys-fwd-speed').innerText = `${vx.toFixed(2)} m/s`;
            document.getElementById('phys-vert-speed').innerText = `${vz.toFixed(2)} m/s`;
            document.getElementById('phys-pitch').innerText = `${pitch >= 0 ? '+' : ''}${pitch.toFixed(1)}°`;
            document.getElementById('phys-yaw').innerText = `${yaw >= 0 ? '+' : ''}${yaw.toFixed(1)}°`;
            document.getElementById('phys-roll').innerText = `${roll >= 0 ? '+' : ''}${roll.toFixed(1)}°`;
            document.getElementById('phys-depth').innerText = `${curDepth.toFixed(2)} m`;

            // 6. Physics Debug Data Panel (Requirement 13)
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

            // Pitch/Roll Crosshair Widget Dot
            const dot = document.getElementById('crosshair-dot');
            const offsetX = Math.max(-25, Math.min(25, roll * 1.2));
            const offsetY = Math.max(-25, Math.min(25, pitch * 1.2));
            dot.style.transform = `translate(${offsetX}px, ${offsetY}px)`;

            // Actuator PWM Bars
            const thrPct = Math.round(((thrPWM - 1500) / 400) * 100);
            const elevPct = Math.round(((elevPWM - 1500) / 400) * 100);
            const rudPct = Math.round(((rudPWM - 1500) / 400) * 100);

            document.getElementById('val-thr-pwm').innerText = `${thrPWM} us (${thrPct >= 0 ? '+' : ''}${thrPct}%)`;
            document.getElementById('fill-thr-pwm').style.width = `${((thrPWM - 1100) / 800) * 100}%`;

            document.getElementById('val-elev-pwm').innerText = `${elevPWM} us (${elevPct >= 0 ? '+' : ''}${elevPct}%)`;
            document.getElementById('fill-elev-pwm').style.width = `${((elevPWM - 1100) / 800) * 100}%`;

            document.getElementById('val-rud-pwm').innerText = `${rudPWM} us (${rudPct >= 0 ? '+' : ''}${rudPct}%)`;
            document.getElementById('fill-rud-pwm').style.width = `${((rudPWM - 1100) / 800) * 100}%`;

            // 7. Update 3D AUV Pose in Three.js (Aligned 3D pitch rotation sign)
            const posX = data.position.x;
            const posY = data.position.y;
            const posZ = data.position.z;

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

            if (cameraMode === 'follow') {
                controls.target.set(tX, tY, tZ);
                controls.update();
            }
        }

        function animate() {
            requestAnimationFrame(animate);
            controls.update();
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

        // Real-Time WebSockets Joystick Streaming (Windows Gamepad -> ROS /joy)
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
            if (activeGp) {
                joyBadge.innerText = 'JOYSTICK: CONNECTED';
                joyBadge.className = 'badge-box badge-enabled';

                const now = performance.now();
                if (now - lastJoyTime >= 25) { // ~40 Hz streaming
                    lastJoyTime = now;
                    const axes = Array.from(activeGp.axes).map((v, idx) => {
                        let val = Math.abs(v) < 0.05 ? 0.0 : v;
                        // Invert Y axes (axis 1, axis 3, axis 5) to match ROS sensor_msgs/Joy (+1.0 UP, -1.0 DOWN)
                        if (idx === 1 || idx === 3 || idx === 5) {
                            val = -val;
                        }
                        return val;
                    });
                    const buttons = Array.from(activeGp.buttons).map(b => b.pressed ? 1 : 0);
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
            } else {
                joyBadge.innerText = 'JOYSTICK: DISCONNECTED';
                joyBadge.className = 'badge-box badge-disabled';
            }
        }

        function startMission() {
            const depth = parseFloat(document.getElementById('input-depth').value) || 0.0;
            const heading = parseFloat(document.getElementById('input-heading').value) || 0.0;
            const distance = parseFloat(document.getElementById('input-distance').value) || 0.0;

            fetch('/api/mission/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ depth: depth, heading: heading, distance: distance })
            }).catch(err => console.error('Mission start error:', err));
        }

        function stopMission() {
            fetch('/api/mission/stop', { method: 'POST' })
            .catch(err => console.error('Mission stop error:', err));
        }

        function resetSim() {
            fetch('/api/mission/reset', { method: 'POST' })
            .catch(err => console.error('Mission reset error:', err));
        }

        window.onload = () => {
            init3D();
            startSSE();
            initJoyWS();
            setInterval(pollGamepad, 25); // 40 Hz Gamepad Polling & ROS /joy streaming
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
                depth = float(data.get('depth', 2.0))
                heading = float(data.get('heading', 90.0))
                distance = float(data.get('distance', 5.0))
                ros_node_instance.start_mission(depth, heading, distance)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status":"started"}')

        elif self.path == '/api/mission/stop':
            if ros_node_instance is not None:
                ros_node_instance.stop_mission()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status":"stopped"}')

        elif self.path == '/api/mission/reset':
            if ros_node_instance is not None:
                ros_node_instance.reset_simulation()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'{"status":"reset"}')
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

    def publish_joy(self, axes, buttons):
        msg = Joy()
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        self.joy_pub.publish(msg)

    def start_mission(self, depth, heading, distance):
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

        msg = Mission()
        msg.depth = float(depth)
        msg.heading = float(heading)
        msg.distance = float(distance)
        self.mission_pub.publish(msg)
        self.get_logger().info(f"ARMED & MISSION STARTED from Web UI: Depth={depth}m, Heading={heading}deg, Distance={distance}m")

    def stop_mission(self):
        dist_msg = Float32()
        dist_msg.data = 0.0
        self.desired_distance_pub.publish(dist_msg)

        status_msg = String()
        status_msg.data = "ABORTED"
        self.mission_status_pub.publish(status_msg)
        self.get_logger().info("MISSION STOPPED from Web UI: Thruster neutral, Mission ABORTED.")

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

    def target_heading_cb(self, msg: Float32):
        simulation_state["autonomy"]["target_heading"] = float(msg.data)

    def target_distance_cb(self, msg: Float32):
        simulation_state["autonomy"]["target_distance"] = float(msg.data)

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

    def dh_cb(self, msg: String):
        simulation_state["status"]["depth_hold"] = msg.data

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

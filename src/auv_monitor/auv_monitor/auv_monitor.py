#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node

from auv_interfaces.msg import Orientation
from auv_interfaces.msg import Depth
from auv_interfaces.msg import Battery
from auv_interfaces.msg import Leak
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from auv_interfaces.msg import Odometry


class AUVMonitor(Node):

    def __init__(self):
        super().__init__('auv_monitor')

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.depth = 0.0
        self.pressure = 0.0
        self.temperature = 0.0

        self.voltage = 0.0
        self.current = 0.0
        self.percentage = 0.0

        self.gain = 1.0
        self.depth_hold = False
        self.target_depth = 0.0


        self.leak = False

        # DVL
        self.dvl_status = "UNKNOWN"
        self.altitude = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.dvl_x = 0.0
        self.dvl_y = 0.0
        self.dvl_z = 0.0


        # AUTONOMY

        self.target_heading = 0.0
        self.target_depth = 0.0
        self.target_distance = 0.0

        self.heading_hold_enabled = False
        self.distance_hold_enabled = False

        self.rudder_cmd = 0.0
        self.depth_cmd = 0.0
        self.throttle_cmd = 0.0

        self.mission_status = "IDLE"


        self.create_subscription(
            Orientation,
            '/auv/orientation',
            self.orientation_callback,
            10
        )

        self.create_subscription(
            Depth,
            '/auv/depth',
            self.depth_callback,
            10
        )

        self.create_subscription(
            Battery,
            '/auv/battery',
            self.battery_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/thruster_gain',
            self.gain_callback,
            10
        )

        self.create_subscription(
            String,
            '/auv/depth_hold_status',
            self.depth_hold_callback,
            10
        )


        self.create_subscription(
            Leak,
            '/auv/leak',
            self.leak_callback,
            10
        )

        self.create_subscription(
            String,
            '/auv/dvl/status',
            self.dvl_status_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/dvl/altitude',
            self.altitude_callback,
            10
        )

        self.create_subscription(
            TwistStamped,
            '/auv/dvl/velocity',
            self.velocity_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/auv/dvl/position',
            self.position_callback,
            10
        )


        self.create_subscription(
            Float32,
            '/auv/desired_heading',
            self.target_heading_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/desired_depth',
            self.target_depth_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/desired_distance',
            self.target_distance_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/heading_hold_enabled',
            self.heading_hold_state_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/distance_hold_enabled',
            self.distance_hold_state_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/rudder_cmd',
            self.rudder_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/depth_cmd',
            self.depth_cmd_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/throttle_cmd',
            self.throttle_callback,
            10
        )


        self.create_subscription(
            String,
            '/auv/mission_status',
            self.mission_status_callback,
            10
        )




        self.create_timer(0.1, self.display)

        self.get_logger().info('AUV Monitor Started')

    def orientation_callback(self, msg):
        self.roll = msg.roll
        self.pitch = msg.pitch
        self.yaw = msg.yaw

    def depth_callback(self, msg):
        self.depth = msg.depth
        self.pressure = msg.pressure
        self.temperature = msg.temperature

    def battery_callback(self, msg):
        self.voltage = msg.voltage
        self.current = msg.current
        self.percentage = msg.percentage

    def gain_callback(self, msg):

        self.gain = msg.data


    def depth_hold_callback(self, msg):

        if msg.data == "OFF":

            self.depth_hold = False

        else:

            self.depth_hold = True

            self.target_depth = float(
                msg.data.split(":")[1]
            )



    def leak_callback(self, msg):
        self.leak = msg.leak_detected

    def dvl_status_callback(self, msg):

        self.dvl_status = msg.data


    def altitude_callback(self, msg):

        self.altitude = msg.data


    def velocity_callback(self, msg):

        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.vz = msg.twist.linear.z


    def position_callback(self, msg):

        self.dvl_x = msg.x
        self.dvl_y = msg.y
        self.dvl_z = msg.z


    def target_heading_callback(self, msg):

        self.target_heading = msg.data


    def target_depth_callback(self, msg):

        self.target_depth = msg.data


    def target_distance_callback(self, msg):

        self.target_distance = msg.data


    def heading_hold_state_callback(self, msg):

        self.heading_hold_enabled = bool(msg.data)


    def distance_hold_state_callback(self, msg):

        self.distance_hold_enabled = bool(msg.data)


    def rudder_callback(self, msg):

        self.rudder_cmd = msg.data


    def depth_cmd_callback(self, msg):

        self.depth_cmd = msg.data


    def throttle_callback(self, msg):

        self.throttle_cmd = msg.data


    def mission_status_callback(self, msg):

        self.mission_status = msg.data


    def display(self):

        os.system('clear')

        print("=" * 50)

        print("AUV STATUS")

        print("=" * 50)

        print(
            f"ROLL  : {self.roll:.2f} deg\n"
            f"PITCH : {self.pitch:.2f} deg\n"
            f"YAW   : {self.yaw:.2f} deg"
        )

        print()

        print(
            f"DEPTH       : {self.depth:.2f} m\n"
            f"PRESSURE    : {self.pressure:.2f} mbar\n"
            f"TEMPERATURE : {self.temperature:.2f} C"
        )

        print()

        print(
            f"VOLTAGE : {self.voltage:.2f} V\n"
            f"CURRENT : {self.current:.2f} A\n"
            f"BATTERY : {self.percentage:.1f} %"
        )

        print()


        print(
              f"THRUSTER GAIN : {self.gain*100:.0f}%"
        )

        if self.depth_hold:

            print(
                f"DEPTH HOLD    : ON @ {self.target_depth:.2f} m"
            )

        else:

            print("DEPTH HOLD    : OFF")

        print()



        if self.leak:
            print("LEAK STATUS : DETECTED")
        else:
            print("LEAK STATUS : SAFE")

        print("=" * 50)

        
        print()

        print("DVL")

        print(
            f"STATUS   : {self.dvl_status}\n"
            f"ALTITUDE : {self.altitude:.2f} m"
        )

        print()

        print(
            f"VX : {self.vx:.3f} m/s\n"
            f"VY : {self.vy:.3f} m/s\n"
            f"VZ : {self.vz:.3f} m/s"
        )

        print()

        print(
            f"X : {self.dvl_x:.2f}\n"
            f"Y : {self.dvl_y:.2f}\n"
            f"Z : {self.dvl_z:.2f}"
        )

        print()

        print("=" * 50)

        print("AUTONOMY")

        print("=" * 50)

        print()

        print(
            f"HEADING HOLD  : {'ON' if self.heading_hold_enabled else 'OFF'}"
        )

        print(
            f"DEPTH HOLD    : {'ON' if self.depth_hold else 'OFF'}"
        )

        print(
            f"DISTANCE HOLD : {'ON' if self.distance_hold_enabled else 'OFF'}"
        )

        print()

        print("CURRENT")

        print(
            f"HEADING  : {self.yaw:.2f} deg\n"
            f"DEPTH    : {self.depth:.2f} m"
        )

        print()

        print("TARGET")

        print(
            f"HEADING  : {self.target_heading:.2f} deg\n"
            f"DEPTH    : {self.target_depth:.2f} m\n"
            f"DISTANCE : {self.target_distance:.2f} m"
        )

        print()

        print("COMMANDS")

        print(
            f"RUDDER   : {self.rudder_cmd:.1f}\n"
            f"ELEVATOR : {self.depth_cmd:.1f}\n"
            f"THROTTLE : {self.throttle_cmd:.2f}"
        )

        print()

        if self.distance_hold_enabled:

            print("MISSION STATUS : RUNNING")

        else:

            print("MISSION STATUS : IDLE")

        print()

        print(f"MISSION STATUS : {self.mission_status}")


        print()

def main(args=None):

    rclpy.init(args=args)

    node = AUVMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

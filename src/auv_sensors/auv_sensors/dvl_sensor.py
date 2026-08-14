#!/usr/bin/env python3

import socket
import json
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, String
from auv_interfaces.msg import Odometry

class DvlSensor(Node):

    def __init__(self):
        super().__init__('dvl_sensor')

        # Water Linked DVL A50
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('dvl_ip', '192.168.194.95')
        self.declare_parameter('dvl_port', 16171)
        self.declare_parameter('publish_rate', 10.0)

        self.sim_mode = self.get_parameter('sim_mode').value
        self.dvl_ip = self.get_parameter('dvl_ip').value
        self.dvl_port = self.get_parameter('dvl_port').value
        rate = self.get_parameter('publish_rate').value

        # Publishers
        self.vel_pub = self.create_publisher(
            TwistStamped,
            '/auv/dvl/velocity',
            10
        )

        self.pos_pub = self.create_publisher(
            Odometry,
            '/auv/dvl/position',
            10
        )

        self.alt_pub = self.create_publisher(
            Float32,
            '/auv/dvl/altitude',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/auv/dvl/status',
            10
        )

        # Velocity data
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.dvl_roll = 0.0
        self.dvl_pitch = 0.0
        self.dvl_yaw = 0.0

        # Position data from DVL
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.altitude = -1.0
        self.valid = False

        self.running = True

        self.get_logger().info(
            f'Connecting to DVL {self.dvl_ip}:{self.dvl_port}'
        )

        self.tcp_thread = threading.Thread(
            target=self.tcp_client_loop,
            daemon=True
        )
        self.tcp_thread.start()

        self.create_timer(
            1.0 / rate,
            self.publish_data
        )

    def tcp_client_loop(self):

        while self.running:

            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            sock.settimeout(5.0)

            try:

                sock.connect(
                    (self.dvl_ip, self.dvl_port)
                )

                self.get_logger().info(
                    f'Connected to DVL at {self.dvl_ip}:{self.dvl_port}'
                )

                buffer = ""

                while self.running:

                    data = sock.recv(4096)

                    if not data:
                        break

                    buffer += data.decode(
                        'utf-8',
                        errors='ignore'
                    )

                    while '\n' in buffer:

                        line, buffer = buffer.split(
                            '\n',
                            1
                        )

                        if line.strip():
                            self.parse_dvl_json(line)

            except Exception as e:

                self.valid = False

                self.get_logger().warn(
                    f'DVL connection failed: {e}'
                )

                time.sleep(3)

            finally:
                sock.close()

    def parse_dvl_json(self, line):

        try:

            msg = json.loads(line)

            msg_type = msg.get("type", "")

            # Velocity packet
            if msg_type == "velocity":

                self.vx = float(
                    msg.get("vx", 0.0)
                )

                self.vy = float(
                    msg.get("vy", 0.0)
                )

                self.vz = float(
                    msg.get("vz", 0.0)
                )

                self.altitude = float(
                    msg.get("altitude", -1.0)
                )

                self.valid = bool(
                    msg.get("velocity_valid", False)
                )

            # Position packet
            elif msg_type == "position_local":

                self.x = float(msg.get("x", 0.0))
                self.y = float(msg.get("y", 0.0))
                self.z = float(msg.get("z", 0.0))

                self.dvl_roll = float(msg.get("roll", 0.0))
                self.dvl_pitch = float(msg.get("pitch", 0.0))
                self.dvl_yaw = float(msg.get("yaw", 0.0))

        except Exception as e:

            self.get_logger().error(
                f'JSON parse error: {e}'
            )

    def publish_data(self):

        vel_msg = TwistStamped()

        vel_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        vel_msg.header.frame_id = 'dvl_link'

        vel_msg.twist.linear.x = self.vx
        vel_msg.twist.linear.y = self.vy
        vel_msg.twist.linear.z = self.vz

        self.vel_pub.publish(
            vel_msg
        )

        alt_msg = Float32()

        alt_msg.data = float(
            self.altitude
        )

        self.alt_pub.publish(
            alt_msg
        )

        status_msg = String()

        if self.valid:
            status_msg.data = "VALID"
        else:
            status_msg.data = "INVALID_OR_NO_LOCK"

        self.status_pub.publish(
            status_msg
        )
 
        pos_msg = Odometry()

        pos_msg.x = self.x
        pos_msg.y = self.y
        pos_msg.z = self.z

        pos_msg.roll = self.dvl_roll
        pos_msg.pitch = self.dvl_pitch
        pos_msg.yaw = self.dvl_yaw

        pos_msg.vx = self.vx
        pos_msg.vy = self.vy
        pos_msg.vz = self.vz

        self.pos_pub.publish(pos_msg)

    def destroy_node(self):

        self.running = False

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = DvlSensor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

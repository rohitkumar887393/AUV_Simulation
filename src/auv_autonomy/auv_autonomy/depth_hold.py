import json
import rclpy

from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from auv_interfaces.msg import Depth
from std_msgs.msg import String

from rclpy.node import Node
from .pid import PID

class DepthHold(Node):

    def __init__(self):

        super().__init__('depth_hold')

        self.depth_hold_enabled = False
        self.target_depth = 0.0
        self.current_depth = 0.0

        # Target source: 'NONE', 'JOYSTICK', or 'EXTERNAL'
        self.target_source = 'NONE'

        self.prev_x = 0
        self.prev_b = 0

        self.pid = PID(
            150.0,
            2.0,
            30.0,
            -300.0,
            300.0
        )

        self.prev_time = self.get_clock().now()

        self.create_subscription(
            Depth,
            '/auv/depth',
            self.depth_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/desired_depth',
            self.desired_depth_callback,
            10
        )

        self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.depth_pub = self.create_publisher(
            Float32,
            '/auv/depth_cmd',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/auv/depth_hold_status',
            10
        )

        self.state_pub = self.create_publisher(
            Float32,
            '/auv/depth_hold_enabled',
            10
        )

        # Common desired depth publisher — notifies all nodes of target updates
        self.target_pub = self.create_publisher(
            Float32,
            '/auv/desired_depth',
            10
        )

        # Active target publisher — always reflects the real controller target
        self.active_target_pub = self.create_publisher(
            Float32,
            '/auv/active_depth_target',
            10
        )

        # Depth telemetry publisher for unified HUD
        self.telemetry_pub = self.create_publisher(
            String,
            '/auv/depth_telemetry',
            10
        )

        self.get_logger().info(
            'Depth Hold Started'
        )

    def desired_depth_callback(self, msg):
        """Accept external/mission depth target."""
        new_target = float(msg.data)
        # Avoid re-triggering integral reset if the target is already identical
        if abs(self.target_depth - new_target) < 1e-4 and self.depth_hold_enabled:
            return

        self.target_depth = new_target
        self.target_source = 'EXTERNAL'
        self.depth_hold_enabled = True

        self.pid.integral = 0.0
        self.pid.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        state = Float32()
        state.data = 1.0
        self.state_pub.publish(state)

        status = String()
        status.data = f"ON:{self.target_depth:.2f}"
        self.status_pub.publish(status)

        active = Float32()
        active.data = self.target_depth
        self.active_target_pub.publish(active)

        self.get_logger().info(f'DESIRED DEPTH SET (EXTERNAL): {self.target_depth:.2f} m')

    def joy_callback(self, msg):

        x_btn = msg.buttons[2] if len(msg.buttons) > 2 else 0
        b_btn = msg.buttons[1] if len(msg.buttons) > 1 else 0

        # X Button -> Turn ON Depth Hold at CURRENT actual sensor depth
        if x_btn == 1 and self.prev_x == 0:
            self.target_depth = float(self.current_depth)
            self.target_source = 'JOYSTICK'
            self.depth_hold_enabled = True

            self.pid.integral = 0.0
            self.pid.prev_error = 0.0
            self.prev_time = self.get_clock().now()

            # 1. State: Depth Hold Enabled
            msg_state = Float32()
            msg_state.data = 1.0
            self.state_pub.publish(msg_state)

            # 2. Status string containing target depth
            status = String()
            status.data = f"ON:{self.target_depth:.2f}"
            self.status_pub.publish(status)

            # 3. Publish to common /auv/desired_depth
            target_msg = Float32()
            target_msg.data = self.target_depth
            self.target_pub.publish(target_msg)

            # 4. Publish active target for HUD
            self.active_target_pub.publish(target_msg)

            # 5. Immediate telemetry
            telem_dict = {
                "target": self.target_depth,
                "current": self.current_depth,
                "error": 0.0,
                "output": 0.0,
                "source": "JOYSTICK"
            }
            telem_msg = String()
            telem_msg.data = json.dumps(telem_dict)
            self.telemetry_pub.publish(telem_msg)

            self.get_logger().info(
                f'DEPTH HOLD ON (JOYSTICK CAPTURED): target_depth = {self.target_depth:.2f} m (current_depth = {self.current_depth:.2f} m)'
            )

        # B Button -> Turn OFF Depth Hold
        if b_btn == 1 and self.prev_b == 0:
            self.depth_hold_enabled = False
            self.target_source = 'NONE'

            msg_state = Float32()
            msg_state.data = 0.0
            self.state_pub.publish(msg_state)

            status = String()
            status.data = "OFF"
            self.status_pub.publish(status)

            self.get_logger().info('DEPTH HOLD OFF')

        self.prev_x = x_btn
        self.prev_b = b_btn

    def depth_callback(self, msg):

        self.current_depth = float(msg.depth)

        if not self.depth_hold_enabled:
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9

        if dt <= 0.0:
            return

        error = self.target_depth - self.current_depth
        output = self.pid.update(error, dt)

        cmd = Float32()
        cmd.data = float(output)
        self.depth_pub.publish(cmd)

        self.prev_time = now

        # Publish active target for HUD
        active = Float32()
        active.data = self.target_depth
        self.active_target_pub.publish(active)

        # Publish depth telemetry for unified HUD
        telem_dict = {
            "target": self.target_depth,
            "current": self.current_depth,
            "error": error,
            "output": float(output),
            "source": self.target_source
        }
        telem_msg = String()
        telem_msg.data = json.dumps(telem_dict)
        self.telemetry_pub.publish(telem_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

        self.get_logger().info(
            'Depth Hold Started'
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



        self.target_pub = self.create_publisher(
            Float32,
            '/auv/desired_depth',
            10
        )

    def desired_depth_callback(self, msg):
        self.target_depth = float(msg.data)
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

        self.get_logger().info(f'DESIRED DEPTH SET : {self.target_depth:.2f} m')





    def joy_callback(self, msg):

        x_btn = msg.buttons[2]

        b_btn = msg.buttons[1]

        if x_btn == 1 and self.prev_x == 0:
  
            self.target_depth = self.current_depth

            target = Float32()
            target.data = self.target_depth

            self.target_pub.publish(target)




            self.depth_hold_enabled = True


            msg = Float32()
            msg.data = 1.0

            self.state_pub.publish(msg)



            status = String()

            status.data = (
                f"ON:{self.target_depth:.2f}"
            )

            self.status_pub.publish(status)



            self.get_logger().info(
                f'DEPTH HOLD ON : {self.target_depth:.2f} m'
            )

        if b_btn == 1 and self.prev_b == 0:

            self.depth_hold_enabled = False


            msg = Float32()
            msg.data = 0.0

            self.state_pub.publish(msg)





            status = String()

            status.data = "OFF"

            self.status_pub.publish(status)


            self.get_logger().info(
                'DEPTH HOLD OFF'
            )

        self.prev_x = x_btn

        self.prev_b = b_btn





    def depth_callback(self, msg):

        self.current_depth = msg.depth

        if not self.depth_hold_enabled:
            return

        now = self.get_clock().now()

        dt = (
            now - self.prev_time
        ).nanoseconds / 1e9

        error = (
            self.target_depth -
            self.current_depth
        )

        output = self.pid.update(
            error,
            dt
        )

        cmd = Float32()

        cmd.data = output

        self.depth_pub.publish(cmd)

        self.prev_time = now





 

def main(args=None):

    rclpy.init(args=args)

    node = DepthHold()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

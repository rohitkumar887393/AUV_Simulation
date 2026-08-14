import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool
from auv_interfaces.msg import ActuatorCmds

from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import CommandBool


class AUVControl(Node):

    def __init__(self):
        super().__init__('auv_control')

        use_sim_param = self.declare_parameter('use_sim', False).value
        if isinstance(use_sim_param, str):
            self.use_sim = use_sim_param.lower() in ('true', '1', 'yes')
        else:
            self.use_sim = bool(use_sim_param)

        self.armed = False
        self.depth_hold_enabled = False

        self.depth_cmd = 0.0
        self.roll_cmd = 0.0

        self.prev_start = 0
        self.prev_back = 0

        self.prev_y = 0
        self.prev_a = 0

        self.gain_levels = [0.25, 0.50, 0.75, 1.00]
        self.gain_index = 2

        self.prev_pwm_elevator_left = 1500
        self.prev_pwm_elevator_right = 1500

        self.prev_pwm_rudder_left = 1500
        self.prev_pwm_rudder_right = 1500

        self.prev_pwm_thruster = 1500

        self.heading_hold_enabled = False
        self.rudder_cmd = 0.0

        self.distance_hold_enabled = False
        self.throttle_cmd = 0.0

        self.create_subscription(
            Float32,
            '/auv/rudder_cmd',
            self.rudder_callback,
            1
        )

        self.create_subscription(
            Float32,
            '/auv/heading_hold_enabled',
            self.heading_hold_state_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/throttle_cmd',
            self.throttle_callback,
            1
        )

        self.create_subscription(
            Float32,
            '/auv/distance_hold_enabled',
            self.distance_hold_state_callback,
            10
        )

        self.arm_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )

        self.servo_client = self.create_client(
            CommandLong,
            '/mavros/cmd/command'
        )

        self.gain_pub = self.create_publisher(
            Float32,
            '/auv/thruster_gain',
            10
        )

        self.actuator_pub = self.create_publisher(
            ActuatorCmds,
            '/auv/actuator_cmds',
            1
        )

        self.armed_pub = self.create_publisher(
            Bool,
            '/auv/armed_status',
            10
        )

        self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            1
        )

        self.create_subscription(
            Float32,
            '/auv/roll_cmd',
            self.roll_callback,
            1
        )

        self.create_subscription(
            Float32,
            '/auv/depth_cmd',
            self.depth_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/auv/depth_hold_enabled',
            self.depth_hold_state_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/auv/arm_cmd',
            self.arm_cmd_callback,
            10
        )

        self.last_axis_1 = 0.0
        self.last_axis_0 = 0.0
        self.last_axis_3 = 0.0

        self.control_timer = self.create_timer(0.02, self.update_control_loop)

        self.publish_gain()
        self.publish_armed()

        self.get_logger().info(f'AUV Control Started (use_sim={self.use_sim})')

    def publish_gain(self):
        msg = Float32()
        msg.data = float(self.gain_levels[self.gain_index])
        self.gain_pub.publish(msg)

    def publish_armed(self):
        msg = Bool()
        msg.data = self.armed
        self.armed_pub.publish(msg)

    def depth_callback(self, msg):
        self.depth_cmd = -msg.data

    def depth_hold_state_callback(self, msg):
        self.depth_hold_enabled = bool(msg.data)

    def throttle_callback(self, msg):
        self.throttle_cmd = msg.data

    def distance_hold_state_callback(self, msg):
        self.distance_hold_enabled = bool(msg.data)

    def roll_callback(self, msg):
        self.roll_cmd = -msg.data

    def arm_vehicle(self):
        if not self.use_sim:
            req = CommandBool.Request()
            req.value = True
            self.arm_client.call_async(req)

        self.armed = True
        self.publish_armed()
        self.get_logger().info('VEHICLE ARMED')

    def disarm_vehicle(self):
        if not self.use_sim:
            req = CommandBool.Request()
            req.value = False
            self.arm_client.call_async(req)

        self.armed = False
        self.publish_armed()
        self.stop_all()
        self.get_logger().info('VEHICLE DISARMED')

    def arm_cmd_callback(self, msg):
        if msg.data:
            self.arm_vehicle()
        else:
            self.disarm_vehicle()

    def set_servo(self, output, pwm):
        req = CommandLong.Request()
        req.command = 183
        req.param1 = float(output)
        req.param2 = float(pwm)
        self.servo_client.call_async(req)

    def set_actuators(self, el_l, el_r, rud_l, rud_r, thruster):
        self.cmd_elevator_left = int(el_l)
        self.cmd_elevator_right = int(el_r)
        self.cmd_rudder_left = int(rud_l)
        self.cmd_rudder_right = int(rud_r)
        self.cmd_main_thruster = int(thruster)

        if self.use_sim:
            msg = ActuatorCmds()
            msg.elevator_left = int(el_l)
            msg.elevator_right = int(el_r)
            msg.rudder_left = int(rud_l)
            msg.rudder_right = int(rud_r)
            msg.main_thruster = int(thruster)
            self.actuator_pub.publish(msg)

        if not self.use_sim:
            if el_l != self.prev_pwm_elevator_left:
                self.set_servo(1, el_l)
                self.prev_pwm_elevator_left = el_l
            if el_r != self.prev_pwm_elevator_right:
                self.set_servo(2, el_r)
                self.prev_pwm_elevator_right = el_r
            if rud_l != self.prev_pwm_rudder_left:
                self.set_servo(3, rud_l)
                self.prev_pwm_rudder_left = rud_l
            if rud_r != self.prev_pwm_rudder_right:
                self.set_servo(4, rud_r)
                self.prev_pwm_rudder_right = rud_r
            if thruster != self.prev_pwm_thruster:
                self.set_servo(5, thruster)
                self.prev_pwm_thruster = thruster

    def stop_all(self):
        self.set_actuators(1500, 1500, 1500, 1500, 1500)

    def rudder_callback(self, msg):
        self.rudder_cmd = msg.data

    def heading_hold_state_callback(self, msg):
        self.heading_hold_enabled = bool(msg.data)

    def joy_callback(self, msg):
        start_btn = msg.buttons[7] if len(msg.buttons) > 7 else 0
        back_btn = msg.buttons[6] if len(msg.buttons) > 6 else 0

        if start_btn == 1 and self.prev_start == 0:
            self.arm_vehicle()

        if back_btn == 1 and self.prev_back == 0:
            self.disarm_vehicle()

        self.prev_start = start_btn
        self.prev_back = back_btn

        btn_y = msg.buttons[3] if len(msg.buttons) > 3 else 0
        btn_a = msg.buttons[0] if len(msg.buttons) > 0 else 0

        # Y button -> gain up
        if btn_y == 1 and self.prev_y == 0:
            if self.gain_index < len(self.gain_levels) - 1:
                self.gain_index += 1
                self.publish_gain()
            self.get_logger().info(
                f"Thruster Gain = {self.gain_levels[self.gain_index]*100:.0f}%"
            )

        # A button -> gain down
        if btn_a == 1 and self.prev_a == 0:
            if self.gain_index > 0:
                self.gain_index -= 1
                self.publish_gain()
            self.get_logger().info(
                f"Thruster Gain = {self.gain_levels[self.gain_index]*100:.0f}%"
            )

        self.prev_y = btn_y
        self.prev_a = btn_a

        if len(msg.axes) > 1:
            self.last_axis_1 = msg.axes[1]
        if len(msg.axes) > 0:
            self.last_axis_0 = msg.axes[0]
        if len(msg.axes) > 3:
            self.last_axis_3 = msg.axes[3]
        elif len(msg.axes) > 5:
            self.last_axis_3 = msg.axes[5]

    def update_control_loop(self):
        if not self.armed:
            self.stop_all()
            return

        # Left stick vertical (axis 1): UP is +1.0 in ROS Joy
        axis_1 = self.last_axis_1
        if self.depth_hold_enabled:
            elevator = 0.0
        else:
            elevator = -axis_1

        # Left stick horizontal (axis 0)
        axis_0 = self.last_axis_0
        if self.heading_hold_enabled:
            rudder = self.rudder_cmd / 400.0
        else:
            rudder = axis_0

        # Right stick vertical / Thruster (axis 3)
        axis_3 = self.last_axis_3
        if self.distance_hold_enabled:
            thruster = self.throttle_cmd * self.gain_levels[self.gain_index]
        else:
            thruster = axis_3 * self.gain_levels[self.gain_index]

        if self.depth_hold_enabled:
            pitch_pwm = self.depth_cmd
        else:
            pitch_pwm = elevator * 400.0

        pwm_elevator_left = int(1500 + pitch_pwm + self.roll_cmd)
        pwm_elevator_right = int(1500 - pitch_pwm + self.roll_cmd)
        pwm_rudder_left = int(1500 + rudder * 400.0)
        pwm_rudder_right = int(1500 - rudder * 400.0)
        pwm_thruster = int(1500 + thruster * 400.0)

        pwm_elevator_left = max(1100, min(1900, pwm_elevator_left))
        pwm_elevator_right = max(1100, min(1900, pwm_elevator_right))
        pwm_rudder_left = max(1100, min(1900, pwm_rudder_left))
        pwm_rudder_right = max(1100, min(1900, pwm_rudder_right))
        pwm_thruster = max(1100, min(1900, pwm_thruster))

        if abs(axis_1) > 0.05:
            self.get_logger().info(
                f"JOY AXIS LEFT_VERTICAL={axis_1:.2f} | NORMALIZED COMMAND={elevator:.2f} | "
                f"ELEVATOR COMMAND={pitch_pwm:.1f} | ELEVATOR LEFT PWM={pwm_elevator_left} | ELEVATOR RIGHT PWM={pwm_elevator_right}"
            )

        self.set_actuators(
            pwm_elevator_left,
            pwm_elevator_right,
            pwm_rudder_left,
            pwm_rudder_right,
            pwm_thruster
        )


def main(args=None):
    rclpy.init(args=args)
    node = AUVControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_all()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

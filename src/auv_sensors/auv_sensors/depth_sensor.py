#!/usr/bin/env python3

import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from mavros_msgs.msg import Mavlink
from auv_interfaces.msg import Depth


class DepthSensor(Node):

    def __init__(self):
        super().__init__('depth_sensor')

        # Declare parameters for calibration and environment
        self.declare_parameter('fluid_density', 1000.0)  # kg/m^3 (use 1024.0 for saltwater)
        self.declare_parameter('gravity', 9.80665)       # m/s^2

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(
            Mavlink,
            '/uas1/mavlink_source',
            self.mavlink_callback,
            qos
        )

        self.depth_pub = self.create_publisher(
            Depth,
            '/auv/depth',
            10
        )

        # Service to trigger manual tare/re-calibration at the surface
        self.tare_srv = self.create_service(
            Trigger,
            '/auv/tare_depth',
            self.tare_callback
        )

        self.surface_pressure = None
        self.last_update_ns = 0
        self.bar30_count = 0

        self.get_logger().info(
            'Depth Sensor Node Started. Ready.'
        )

    def tare_callback(self, request, response):
        self.surface_pressure = None
        self.get_logger().info('Tare requested. Recalibrating surface pressure on next reading...')
        response.success = True
        response.message = "Recalibrating surface pressure."
        return response

    def mavlink_callback(self, msg):
        try:
            # BAR30 = SCALED_PRESSURE2 (msgid 137)
            if msg.msgid != 137:
                return

            now_ns = self.get_clock().now().nanoseconds

            # 20 Hz limit
            if now_ns - self.last_update_ns < 20000000:
                return

            self.last_update_ns = now_ns
            self.bar30_count += 1

            # Reconstruct byte payload from uint64 array
            payload = b''.join(
                struct.pack('<Q', word)
                for word in msg.payload64
            )
            payload = payload[:msg.len]

            # MAVLink 2 zero-truncation: pad the payload if truncated
            expected_len = 14
            if len(payload) < expected_len:
                payload = payload.ljust(expected_len, b'\x00')

            (
                time_boot_ms,
                press_abs,     # Can be hPa or Pa depending on MAVLink/autopilot version
                press_diff,
                temperature
            ) = struct.unpack(
                '<Iffh',
                payload
            )

            # Auto-detect pressure units and convert to Pascals (Pa)
            if press_abs > 50000.0:
                # Value is in Pascals (e.g. ~101325 Pa)
                pressure_pa = press_abs
            elif press_abs > 5000.0:
                # Value is in decapascal (e.g. ~10132.5 daPa)
                pressure_pa = press_abs * 10.0
            else:
                # Value is in hectopascals/mbar (e.g. ~1013.25 hPa)
                pressure_pa = press_abs * 100.0

            # Calibrate surface pressure on the first valid sample
            if self.surface_pressure is None:
                # Sane boundary check to ignore garbage/zero readings on startup
                if pressure_pa > 50000.0 and pressure_pa < 150000.0:
                    self.surface_pressure = pressure_pa
                    self.get_logger().info(
                        f"Calibrated surface pressure to: {self.surface_pressure / 100.0:.2f} hPa (mbar)"
                    )
                else:
                    # Publish 0.0 depth while waiting for a valid calibration pressure
                    self.publish_depth(0.0, pressure_pa, temperature)
                    return

            # Retrieve parameters
            fluid_density = self.get_parameter('fluid_density').value
            g = self.get_parameter('gravity').value

            # Calculate depth (m)
            # depth = (P_abs - P_surface) / (rho * g)
            depth = (pressure_pa - self.surface_pressure) / (fluid_density * g)

            # Note: We do NOT clamp depth to 0.0 here so that the user can see 
            # sub-centimeter level fluctuations and negative drift on their monitor screen.
            self.publish_depth(depth, pressure_pa, temperature)

        except Exception as e:
            self.get_logger().error(f"Error in depth sensor callback: {e}")

    def publish_depth(self, depth, pressure_pa, temperature_cdeg):
        depth_msg = Depth()
        depth_msg.depth = float(depth)
        depth_msg.pressure = float(pressure_pa / 100.0)  # Convert back to mbar/hPa for consistency
        depth_msg.temperature = float(temperature_cdeg) / 100.0
        self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

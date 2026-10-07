#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.pub = self.create_publisher(Imu, "/imu", 10)
        
        # Constants
        self.GRAVITY = 9.80665
        self.DEG_TO_RAD = math.pi / 180.0
        
        # Calibration variables
        self.gyro_z_offset = 0.0
        self.calibration_samples = 100
        self.samples_taken = 0
        self.is_calibrated = False

        # Initialize hardware
        try:
            self.sensor = mpu6050(0x68)
            self.get_logger().info("MPU6050 initialized. Keep robot still for calibration...")
        except Exception as e:
            self.get_logger().error(f"Hardware initialization failed: {e}")
            raise e

        # timer at 20Hz (0.05s)
        self.timer = self.create_timer(0.05, self.loop)

    def loop(self):
        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()

            # 1. Handle Calibration (To stop the slow spinning in RViz)
            if not self.is_calibrated:
                self.gyro_z_offset += gyro["z"]
                self.samples_taken += 1
                if self.samples_taken >= self.calibration_samples:
                    self.gyro_z_offset /= self.calibration_samples
                    self.is_calibrated = True
                    self.get_logger().info(f"Calibration complete! Offset: {self.gyro_z_offset:.4f}")
                return # Don't publish until calibrated

            # 2. Apply Offset and Convert to Radians/sec
            # If robot still spins, change '-' to '+' below to invert axis
            gz = (gyro["z"] - self.gyro_z_offset) * self.DEG_TO_RAD
            gx = gyro["x"] * self.DEG_TO_RAD
            gy = gyro["y"] * self.DEG_TO_RAD

            # 3. Small Deadzone to kill remaining jitter
            if abs(gz) < 0.001:
                gz = 0.0

            # 4. Create ROS 2 Message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link" # Matches your URDF

            # Angular Velocity (rad/s)
            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)

            # Linear Acceleration (m/s^2)
            msg.linear_acceleration.x = float(accel["x"] * self.GRAVITY)
            msg.linear_acceleration.y = float(accel["y"] * self.GRAVITY)
            msg.linear_acceleration.z = float(accel["z"] * self.GRAVITY)

            # Tell EKF we don't have orientation (it will calculate it from gyro)
            msg.orientation_covariance[0] = -1.0 

            self.pub.publish(msg)

        except OSError:
            self.get_logger().warn("I2C Bus busy or dropped. Skipping frame.")
        except Exception as e:
            self.get_logger().error(f"Loop error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
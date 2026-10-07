import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor
from signal import SIGINT, SIGTERM, signal
from rclpy.executors import ExternalShutdownException

# ==== GPIO PINS (BCM) ====
# Matches your previous setup
LEFT_PWM_PIN = 18
LEFT_DIR_PIN = 23
LEFT_BACK_PIN = 24 # For backward direction control if needed
RIGHT_PWM_PIN = 19
RIGHT_DIR_PIN = 6
RIGHT_BACK_PIN = 5 # For backward direction control if needed
left_motor_calibration_factor = 1.1 # Adjust as needed
right_motor_calibration_factor = 1 # Adjust as needed
speed_cal=1.0 # Max speed (0.0 to 1.0)
min_speed=0.1 # Minimum speed to overcome motor deadzone (0.0 to 1.0)
max_angular=0.2 # Max angular velocity in radians (tune as needed) 
angular_importance = 0.5

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # PhaseEnableMotor(phase, enable) 
        # phase = Direction, enable = PWM/Speed
        self.left_motor = Motor(forward=LEFT_DIR_PIN, backward=LEFT_BACK_PIN, enable=LEFT_PWM_PIN)
        self.right_motor = Motor(forward=RIGHT_DIR_PIN, backward=RIGHT_BACK_PIN, enable=RIGHT_PWM_PIN)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        self.get_logger().info("Motor Driver Node Online (Using GPIO Zero)")

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        
        if  linear!=0.0:
            angular=angular*angular_importance
        
        # Differential drive math
        left_speed = linear - angular
        right_speed = linear + angular
        left_speed = left_speed * left_motor_calibration_factor
        right_speed = right_speed * right_motor_calibration_factor

        # Clamp and move left motor
        self.move_motor(self.left_motor, left_speed)
        # Clamp and move right motor
        self.move_motor(self.right_motor, right_speed)

    def move_motor(self, motor_obj, speed):
        # Ensure speed is between -1.0 and 1.0 or between the set calibration limits
        speed = max(min(speed, speed_cal),-speed_cal)
        speed=speed*-1.0
        
        if speed > 0:
            motor_obj.forward(speed)
        elif speed < 0:
            motor_obj.backward(abs(speed))
        else:
            motor_obj.stop()
        if abs(speed) < min_speed and speed != 0:
            speed = min_speed if speed > 0 else -min_speed

    def stop_motors(self):
        self.get_logger().info("Stopping motors and releasing GPIO...")
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.close()
        self.right_motor.close()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # We don't need to do anything here, just catch it to exit the spin
        pass
    finally:
        # 1. Print to the terminal using standard python print 
        # (Since ROS logging might be dead/invalid at this point)
        print("[Motor_node]: Shutting down... Stopping motors.")
        
        # 2. Stop the hardware first
        node.left_motor.close()
        node.right_motor.close()
        
        # 3. Clean up ROS
        node.destroy_node()
        
        # 4. Only shutdown if rclpy hasn't done it automatically
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
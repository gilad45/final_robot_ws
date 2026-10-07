import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int8
import json
import serial

class SensorInputUart(Node):
    def __init__(self):
        super().__init__('sensor_input_uart')
        # Open serial port with error handling
        try:
            self.ser = serial.Serial(
                port='/dev/ttyAMA0',
                baudrate=115200,
                timeout=1
            )

        except Exception as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            raise SystemExit

        self.microphone = self.create_publisher(Bool,'/mic',10)
        self.ultrasonic = self.create_publisher(Float32,'/ultrasonic',10)
        self.flame_sensor = self.create_publisher(Int8,'/flame_sensor',10)
        self.fire_sensor = self.create_publisher(Bool,'/fire_sensor',10)
        self.timer = self.create_timer(0.05 ,self.read_uart)
        self.get_logger().info('UART reader has started')
        

    def read_uart(self):
        try:
        
            if self.ser.in_waiting > 0:

                line = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Raw UART: {line}")

                

                if not line:
                    return
                data = json.loads(line)

                # Ultrasonic
                msg_ultrasonic = Float32()
                msg_ultrasonic.data = float(data.get('distance', 0.0))
                self.ultrasonic.publish(msg_ultrasonic)

                # Microphone
                msg_microphone = Bool()
                msg_microphone.data = bool(data.get('microphone', False))
                self.microphone.publish(msg_microphone)

                # Flame sensor
                msg_flame_sensor = Int8()
                msg_flame_sensor.data = int((data.get('flame_sensor', 1))) # Invert the flame sensor reading
                self.flame_sensor.publish(msg_flame_sensor)

                # Fire sensor
                msg_fire_sensor = Bool()
                msg_fire_sensor.data = bool(data.get('fire_sensor', False))
                self.fire_sensor.publish(msg_fire_sensor)

        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON: {line}")

        except Exception as e:
            self.get_logger().error(f"Error reading UART: {e}")


def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorInputUart()
    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            sensor_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()




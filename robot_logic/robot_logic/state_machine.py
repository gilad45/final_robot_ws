import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32, Bool, Int8
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from gpiozero import OutputDevice


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.fire_sub = self.create_subscription(Bool, '/fire_sensor', self.fire_callback, 10)
        self.flame_sub = self.create_subscription(Int8, '/flame_sensor', self.flame_callback, 10)
        self.ultrasonic_sub = self.create_subscription(Float32, '/ultrasonic_sensor', self.ultrasonic_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.fire = False
        self.flame = 0
        self.distance = 0.0

        # Tracks the single in-flight Nav2 goal so we can cancel it without
        # needing to know its UUID up front.
        self.goal_handle = None
        self.nav_result_ready = False
        self.nav_succeeded = False

        self.relay = OutputDevice(17)  # GPIO 17

        self.state = 'Navigating'
        self.timer = self.create_timer(0.1, self.decision_logic)

        self.get_logger().info('state machine has started')

        # Kick off the initial navigation goal here, e.g.:
        # self.send_goal(1.0, 2.0)

    # ---------------- Nav2 goal handling ----------------

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_result_ready = False
        self.nav_succeeded = False

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('NavigateToPose action server not available')
            return

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2')
            self.goal_handle = None
            return

        self.get_logger().info('Goal accepted by Nav2')
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.nav_result_ready = True
        status = future.result().status
        self.nav_succeeded = (status == 4)  # STATUS_SUCCEEDED
        self.goal_handle = None  # goal has finished, nothing left to cancel
        self.get_logger().info(f'Nav2 finished with status: {status}')

    def stop_properly(self):
        # Cancel whichever goal we're actually tracking right now.
        if self.goal_handle is not None:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            self.get_logger().info('Fire found, sent cancel request to Nav2')
        else:
            self.get_logger().info('No active Nav2 goal to cancel')

    def cancel_done_callback(self, future):
        self.get_logger().info('Nav2 goal cancel confirmed')
        self.goal_handle = None

    def at_goal(self):
        return self.nav_result_ready and self.nav_succeeded

    # ---------------- state machine ----------------

    def decision_logic(self):
        if self.state == 'Navigating':
            self.navigation()
        elif self.state == 'fire_detected':
            self.fire_detected()
        elif self.state == 'extinguishing':
            self.extinguishing()
        elif self.state == 'returning_to_base':
            self.returning_to_base()

    def fire_callback(self, msg):
        self.fire = msg.data

    def flame_callback(self, msg):
        self.flame = msg.data

    def ultrasonic_callback(self, msg):
        self.distance = msg.data

    def navigation(self):
        if self.fire:
            self.stop_properly()
            self.state = 'fire_detected'

    def fire_detected(self):
        self.send_cmd_vel(linear=0.0, angular=1.0)
        self.get_logger().info('spinning to find the flame')
        if self.flame == 0:
            self.state = 'extinguishing'

    def extinguishing(self):
        if self.distance > 15 and self.flame == 0 and self.fire:
            self.send_cmd_vel(linear=0.7, angular=0.0)
            self.get_logger().info('Getting close to the fire')
        else:
            self.set_relay(True)
            if self.flame == 1:
                self.set_relay(False)
                self.state = 'returning_to_base'
                self.send_goal(0.0, 0.0)  # fire goal once, not every tick

    def returning_to_base(self):
        if self.at_goal():
            self.get_logger().info('lets goooooo')

    def send_cmd_vel(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def set_relay(self, state):
        self.relay.on() if state else self.relay.off()

    def destroy_node(self):
        self.relay.off()
        self.relay.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    state_node = StateMachineNode()
    try:
        rclpy.spin(state_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            state_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
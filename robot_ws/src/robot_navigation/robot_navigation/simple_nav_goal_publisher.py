import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SimpleNavGoalPublisher(Node):

    def __init__(self):
        super().__init__('simple_nav_goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info('Initialized SimpleNavGoalPublisher node.')

        # Optional: Action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def publish_goal(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.z = float(yaw) # For 2D navigation, yaw is typically around Z-axis
        goal_pose.pose.orientation.w = 1.0 # Assuming no roll/pitch, identity quaternion part

        self.publisher_.publish(goal_pose)
        self.get_logger().info(f'Published 2D Nav Goal: x={x}, y={y}, yaw={yaw}')

    def send_goal_action(self, x, y, yaw):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = float(yaw)
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal action: x={x}, y={y}, yaw={yaw}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.total_time.sec} seconds')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavGoalPublisher()
    # Example: Publish a goal using the topic interface
    # node.publish_goal(x=2.0, y=0.0, yaw=0.0) # Move to x=2, y=0

    # Example: Send a goal using the action interface (preferred for Nav2)
    node.send_goal_action(x=2.0, y=0.0, yaw=0.0) # Move to x=2, y=0

    rclpy.spin(node)
    # rclpy.shutdown() # Shutdown handled in get_result_callback for action

if __name__ == '__main__':
    main()

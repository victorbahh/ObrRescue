from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MissionNode(Node):
    def __init__(self):
        super().__init__('rescue_mission')
        self.map_ready = False
        self.localized = False
        self.goal_sent = False
        self.last_initial_time = None
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/mcl/pose', self.pose_callback, 10)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(1.0, self.start_when_ready)

    def map_callback(self, message):
        self.map_ready = bool(message.info.width and message.info.height)

    def pose_callback(self, message):
        if message.header.frame_id == 'map':
            self.localized = True

    def start_when_ready(self):
        if not self.map_ready or self.goal_sent:
            return
        now = self.get_clock().now()
        if not self.localized:
            if self.last_initial_time is None or (now - self.last_initial_time).nanoseconds >= 3000000000:
                initial = PoseWithCovarianceStamped()
                initial.header.frame_id = 'map'
                initial.header.stamp = now.to_msg()
                initial.pose.pose.position.x = -0.65
                initial.pose.pose.position.y = -0.325
                initial.pose.pose.orientation.w = 1.0
                initial.pose.covariance[0] = 0.0004
                initial.pose.covariance[7] = 0.0004
                initial.pose.covariance[35] = 0.01
                self.publisher.publish(initial)
                self.last_initial_time = now
                self.get_logger().info('Initializing MCL at the entrance')
            return
        if not self.client.server_is_ready():
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = now.to_msg()
        goal.pose.pose.position.x = 0.65
        goal.pose.pose.position.y = 0.325
        goal.pose.pose.orientation.w = 1.0
        self.goal_sent = True
        self.client.send_goal_async(goal).add_done_callback(self.goal_response)
        self.get_logger().info('Navigating to the exit')

    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Navigation goal was rejected')
            return
        handle.get_result_async().add_done_callback(self.goal_result)

    def goal_result(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Exit reached')
        else:
            self.get_logger().error(f'Navigation did not reach the exit (status {result.status})')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
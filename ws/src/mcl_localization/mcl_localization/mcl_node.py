import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import math


# Mapa da sala OBR (area_resgate_obr.sdf): 4 paredes, sala 0.94 x 0.94 m centrada em (0,0).
WALLS = [
    (-0.47,  0.47,  0.47,  0.47),  # Norte
    (-0.47, -0.47,  0.47, -0.47),  # Sul
    (-0.47, -0.47, -0.47,  0.47),  # Oeste
    ( 0.47, -0.47,  0.47,  0.47),  # Leste
]


def quaternion_from_yaw(yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MapAndGroundTruthNode(Node):
    def __init__(self):
        super().__init__('map_and_ground_truth_node')

        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_odom)

        self.gt_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/ground_truth_pose', 10)

        self.map_pub = self.create_publisher(
            MarkerArray, '/map', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_yaw = None

        # Publica o mapa estaticamente uma vez
        self.publish_map()

        self.get_logger().info('Map and ground truth node iniciado.')

    def publish_map(self):
        """Publica as 4 paredes da sala como markers para visualizacao no RViz."""
        marker_array = MarkerArray()

        for i, (x1, y1, x2, y2) in enumerate(WALLS):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'walls'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01  # espessura da linha
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.color.a = 1.0

            p1 = Point(x=x1, y=y1, z=0.0)
            p2 = Point(x=x2, y=y2, z=0.0)
            marker.points = [p1, p2]

            marker_array.markers.append(marker)

        self.map_pub.publish(marker_array)

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # Publica pose verdadeira (ground truth)
        gt_msg = PoseWithCovarianceStamped()
        gt_msg.header.stamp = self.get_clock().now().to_msg()
        gt_msg.header.frame_id = 'map'
        gt_msg.pose.pose.position = Point(x=x, y=y, z=0.012)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        gt_msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        gt_msg.pose.covariance = [0.0] * 36
        self.gt_pub.publish(gt_msg)

        # Publica tf map -> odom (origem do odom no mapa)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapAndGroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

from collections import deque
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mcl_localization.mcl_node import yaw_of
from mcl_localization.particle_filter import OccupancyMap
from mcl_localization.victim_detection import CandidateTracker, find_candidates


class VictimNode(Node):
    def __init__(self):
        super().__init__('victim_detector')
        self.grid = None
        self.tracker = CandidateTracker()
        self.scans = deque(maxlen=100)
        self.visible_ids = set()
        self.last_stamp = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.create_subscription(LaserScan, '/laser', self.scan_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(MarkerArray, '/victims/markers', 10)
        self.create_timer(0.1, self.process_scans)

    def scan_callback(self, scan):
        self.scans.append(scan)

    def map_callback(self, message):
        if self.grid is not None:
            return
        origin = message.info.origin
        try:
            self.grid = OccupancyMap(message.info.width, message.info.height, message.info.resolution,
                                     (origin.position.x, origin.position.y, yaw_of(origin.orientation)),
                                     message.data)
        except ValueError as error:
            self.get_logger().error(f'Cannot use map for detection: {error}')

    def process_scans(self):
        if self.grid is None:
            return
        scan = None
        while self.scans:
            candidate = self.scans[0]
            if (self.get_clock().now() - Time.from_msg(candidate.header.stamp)).nanoseconds < 150000000:
                break
            scan = self.scans.popleft()
        if scan is None:
            return
        stamp = Time.from_msg(scan.header.stamp)
        if self.last_stamp is not None and (stamp - self.last_stamp).nanoseconds < 200000000:
            return
        try:
            transform = self.tf_buffer.lookup_transform('map', scan.header.frame_id, stamp).transform
        except TransformException as error:
            self.get_logger().warning(f'Waiting for laser TF: {error}', throttle_duration_sec=5.0)
            return
        yaw = yaw_of(transform.rotation)
        points = []
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance < scan.range_max:
                points.append(None)
                continue
            angle = yaw + scan.angle_min + index * scan.angle_increment
            points.append((transform.translation.x + distance * math.cos(angle),
                           transform.translation.y + distance * math.sin(angle)))
        tracks = self.tracker.update(find_candidates(points, self.grid), stamp.nanoseconds / 1e9)
        markers = MarkerArray()
        current_ids = {track['id'] for track in tracks}
        for marker_id in self.visible_ids - current_ids:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = scan.header.stamp
            marker.ns = 'possible_victims'
            marker.id = marker_id
            marker.action = Marker.DELETE
            markers.markers.append(marker)
        self.visible_ids = current_ids
        for track in tracks:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = scan.header.stamp
            marker.ns = 'possible_victims'
            marker.id = track['id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = track['x']
            marker.pose.position.y = track['y']
            marker.pose.position.z = 0.0225
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.045
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.75, 0.0, 0.9
            marker.lifetime = Duration(seconds=1.0).to_msg()
            markers.markers.append(marker)
        self.publisher.publish(markers)
        self.last_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = VictimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
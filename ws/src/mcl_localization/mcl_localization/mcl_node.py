from collections import deque
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from mcl_localization.particle_filter import OccupancyMap, ParticleFilter, wrap


def yaw_of(quaternion):
	return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
					  1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))


def set_yaw(quaternion, yaw):
	quaternion.z = math.sin(yaw / 2.0)
	quaternion.w = math.cos(yaw / 2.0)


class MclNode(Node):
	def __init__(self):
		super().__init__('mcl_node')
		self.declare_parameter('particle_count', 300)
		self.declare_parameter('max_beams', 40)
		self.declare_parameter('max_initial_std_xy', 0.1)
		self.declare_parameter('max_initial_std_yaw', 0.5)
		self.declare_parameter('ground_truth_frame', 'ground_truth_world')
		self.particle_count = self.get_parameter('particle_count').value
		self.max_beams = self.get_parameter('max_beams').value
		if self.particle_count <= 0 or self.max_beams <= 0:
			raise ValueError('particle_count and max_beams must be positive')
		self.filter = None
		self.previous_odom = None
		self.last_scan_time = None
		self.map_to_odom = None
		self.pending_scans = deque(maxlen=100)
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.tf_broadcaster = TransformBroadcaster(self)
		map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
							 reliability=ReliabilityPolicy.RELIABLE)
		self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
		self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_callback, 10)
		self.create_subscription(LaserScan, '/laser', self.scan_callback, qos_profile_sensor_data)
		self.create_subscription(PoseStamped, '/ground_truth/pose', self.ground_truth_callback, 10)
		self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/mcl/pose', 10)
		self.particle_publisher = self.create_publisher(PoseArray, '/mcl/particles', 10)
		self.ground_truth_publisher = self.create_publisher(PoseStamped, '/ground_truth/rviz', 10)
		self.create_timer(0.05, self.publish_transform)
		self.create_timer(0.05, self.process_scans)
		self.get_logger().info('Waiting for /map and an initial pose from RViz')

	def map_callback(self, message):
		if self.filter is not None:
			return
		origin = message.info.origin
		try:
			grid = OccupancyMap(message.info.width, message.info.height, message.info.resolution,
								(origin.position.x, origin.position.y, yaw_of(origin.orientation)),
								message.data)
			self.filter = ParticleFilter(grid, self.particle_count)
		except ValueError as error:
			self.get_logger().error(f'Cannot use map: {error}')
			return
		self.get_logger().info('Map received; select 2D Pose Estimate in RViz')

	def initial_callback(self, message):
		if self.filter is None:
			self.get_logger().warning('Cannot initialize before receiving /map')
			return
		if message.header.frame_id != 'map':
			self.get_logger().warning('Initial pose must be expressed in map')
			return
		pose = message.pose.pose
		covariance = message.pose.covariance
		max_xy = max(0.02, self.get_parameter('max_initial_std_xy').value)
		max_yaw = max(0.05, self.get_parameter('max_initial_std_yaw').value)
		deviations = (min(max_xy, max(0.02, math.sqrt(max(0.0, covariance[0])))),
			      min(max_xy, max(0.02, math.sqrt(max(0.0, covariance[7])))),
			      min(max_yaw, max(0.05, math.sqrt(max(0.0, covariance[35])))))
		self.filter.initialize((pose.position.x, pose.position.y, yaw_of(pose.orientation)), deviations)
		self.previous_odom = None
		self.last_scan_time = None
		self.map_to_odom = None
		self.pending_scans.clear()
		self.get_logger().info('Particle filter initialized')

	def scan_callback(self, scan):
		if self.filter is not None and self.filter.particles:
			self.pending_scans.append(scan)

	def process_scans(self):
		scan = None
		now = self.get_clock().now()
		while self.pending_scans:
			candidate = self.pending_scans[0]
			age = (now - Time.from_msg(candidate.header.stamp)).nanoseconds
			if age < 150000000:
				break
			scan = self.pending_scans.popleft()
		if scan is not None:
			self.process_scan(scan)

	def process_scan(self, scan):
		if self.filter is None or not self.filter.particles:
			return
		stamp = Time.from_msg(scan.header.stamp)
		if self.last_scan_time is not None and (stamp - self.last_scan_time).nanoseconds < 200000000:
			return
		try:
			odom_transform = self.tf_buffer.lookup_transform('odom', 'base_link', stamp).transform
			laser_transform = self.tf_buffer.lookup_transform('base_link', scan.header.frame_id, stamp).transform
		except TransformException as error:
			self.get_logger().warning(f'Waiting for laser/odometry TF: {error}', throttle_duration_sec=5.0)
			return
		odom = (odom_transform.translation.x, odom_transform.translation.y, yaw_of(odom_transform.rotation))
		if self.previous_odom is not None:
			distance = math.hypot(odom[0] - self.previous_odom[0], odom[1] - self.previous_odom[1])
			rotation = abs(wrap(odom[2] - self.previous_odom[2]))
			if distance < 0.01 and rotation < 0.04:
				return
		step = max(1, math.ceil(len(scan.ranges) / self.max_beams))
		beams = [(scan.angle_min + index * scan.angle_increment, distance)
				 for index, distance in enumerate(scan.ranges) if index % step == 0
				 and math.isfinite(distance) and scan.range_min <= distance < scan.range_max]
		if not beams:
			self.get_logger().warning('Laser scan contains no usable returns', throttle_duration_sec=5.0)
			return
		if self.previous_odom is not None:
			self.filter.move(self.previous_odom, odom)
		laser_pose = (laser_transform.translation.x, laser_transform.translation.y,
					  yaw_of(laser_transform.rotation))
		self.filter.update(beams, laser_pose)
		estimate, variances = self.filter.estimate()
		self.publish_estimate(estimate, variances, scan.header.stamp)
		self.filter.resample_if_needed()
		self.publish_particles(scan.header.stamp)
		yaw = wrap(estimate[2] - odom[2])
		cosine, sine = math.cos(yaw), math.sin(yaw)
		self.map_to_odom = (estimate[0] - cosine * odom[0] + sine * odom[1],
							estimate[1] - sine * odom[0] - cosine * odom[1], yaw)
		self.previous_odom = odom
		self.last_scan_time = stamp

	def publish_estimate(self, estimate, variances, stamp):
		message = PoseWithCovarianceStamped()
		message.header.frame_id = 'map'
		message.header.stamp = stamp
		message.pose.pose.position.x, message.pose.pose.position.y = estimate[:2]
		set_yaw(message.pose.pose.orientation, estimate[2])
		message.pose.covariance[0] = variances[0]
		message.pose.covariance[7] = variances[1]
		message.pose.covariance[35] = variances[2]
		self.pose_publisher.publish(message)

	def publish_particles(self, stamp):
		message = PoseArray()
		message.header.frame_id = 'map'
		message.header.stamp = stamp
		for x, y, yaw, _ in self.filter.particles:
			pose = Pose()
			pose.position.x, pose.position.y = x, y
			set_yaw(pose.orientation, yaw)
			message.poses.append(pose)
		self.particle_publisher.publish(message)

	def publish_transform(self):
		if self.map_to_odom is None:
			return
		transform = TransformStamped()
		transform.header.stamp = (self.get_clock().now() + Duration(seconds=0.1)).to_msg()
		transform.header.frame_id = 'map'
		transform.child_frame_id = 'odom'
		transform.transform.translation.x, transform.transform.translation.y = self.map_to_odom[:2]
		set_yaw(transform.transform.rotation, self.map_to_odom[2])
		self.tf_broadcaster.sendTransform(transform)

	def ground_truth_callback(self, message):
		message.header.frame_id = self.get_parameter('ground_truth_frame').value
		self.ground_truth_publisher.publish(message)


def main(args=None):
	rclpy.init(args=args)
	node = MclNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

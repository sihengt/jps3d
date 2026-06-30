#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from waypoint_sequencer_logic import compute_next_target


class WaypointSequencer(Node):
    """Walks a fixed list of waypoints, replanning toward the current one on
    a timer (same mechanism as rviz_goal_client.py) and advancing to the next
    waypoint once within switch_radius. This keeps every leg fully reactive
    to obstacles while never asking droan to converge on a goal that's
    already close enough to trigger its near-goal oscillation (the goal is
    swapped out before that regime is reached).
    """

    def __init__(self):
        super().__init__('waypoint_sequencer')

        self.declare_parameter('odom_topic',
                               '/px4vision_custom_0/mavros/local_position/odom')
        self.declare_parameter('plan_srv_topic', '/plan')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('replan_period', 0.5)
        self.declare_parameter('switch_radius', 0.5)
        self.declare_parameter('tolerance', 0.)
        self.declare_parameter('markers_topic', 'waypoint_markers')
        self.declare_parameter('markers_period', 1.0)
        # Flat list [x0,y0,z0, x1,y1,z1, ...]; must be a multiple of 3 long.
        self.declare_parameter('waypoints', [
            0.0, -5.0, 1.5,
            9.0, -10.0, 2.0,
            1.3, -12.0, 2.3,
            # -0.72, -12.67, 1.1,
            -3.84, -11.95, 1.1,
            0.0, -16.0, 1.3])

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        plan_srv_topic = self.get_parameter('plan_srv_topic').get_parameter_value().string_value
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        replan_period = self.get_parameter('replan_period').get_parameter_value().double_value
        self._switch_radius = self.get_parameter('switch_radius').get_parameter_value().double_value
        self._tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        markers_topic = self.get_parameter('markers_topic').get_parameter_value().string_value
        markers_period = self.get_parameter('markers_period').get_parameter_value().double_value

        flat = list(self.get_parameter('waypoints').get_parameter_value().double_array_value)
        if len(flat) % 3 != 0 or len(flat) == 0:
            raise ValueError(
                f'waypoints param must be a non-empty flat list of x,y,z triples, got {len(flat)} values')
        self._waypoints = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        self._current_index = 0
        self._mission_complete = False
        self._latest_odom = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Odometry, odom_topic, self._odom_cb, sensor_qos)
        self._plan_client = self.create_client(GetPlan, plan_srv_topic)
        self._timer = self.create_timer(replan_period, self._tick)

        self._markers_pub = self.create_publisher(MarkerArray, markers_topic, 1)
        self._markers_timer = self.create_timer(markers_period, self._publish_markers)
        self._publish_markers()

        self.get_logger().info(
            f'Waypoint sequencer ready: {len(self._waypoints)} waypoints, '
            f'switch_radius={self._switch_radius:.2f}m, replan_period={replan_period:.2f}s, '
            f'markers on {markers_topic}')

    def _odom_cb(self, msg):
        self._latest_odom = msg

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        array = MarkerArray()

        path = Marker()
        path.header.frame_id = self._frame_id
        path.header.stamp = now
        path.ns = 'waypoints'
        path.id = 0
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.pose.orientation.w = 1.0
        path.scale.x = 0.05
        path.color.r, path.color.g, path.color.b, path.color.a = 0.0, 0.6, 1.0, 0.8
        path.points = [Point(x=x, y=y, z=z) for x, y, z in self._waypoints]
        array.markers.append(path)

        for i, (x, y, z) in enumerate(self._waypoints):
            if i < self._current_index:
                color = (0.5, 0.5, 0.5, 0.6)  # visited
            elif i == self._current_index:
                color = (0.0, 1.0, 0.0, 1.0)  # current target
            else:
                color = (1.0, 0.6, 0.0, 0.9)  # upcoming

            sphere = Marker()
            sphere.header.frame_id = self._frame_id
            sphere.header.stamp = now
            sphere.ns = 'waypoints'
            sphere.id = i + 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = x, y, z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
            array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = self._frame_id
            label.header.stamp = now
            label.ns = 'waypoint_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, z + 0.5
            label.pose.orientation.w = 1.0
            label.scale.z = 0.3
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
            label.text = str(i)
            array.markers.append(label)

        self._markers_pub.publish(array)

    def _tick(self):
        if self._mission_complete:
            return
        if self._latest_odom is None:
            return
        if not self._plan_client.service_is_ready():
            self.get_logger().warn('Plan service not ready — is jps3d_node running?', once=True)
            return

        p = self._latest_odom.pose.pose.position
        position = (p.x, p.y, p.z)

        next_index = compute_next_target(
            self._current_index, position, self._waypoints, self._switch_radius)

        if next_index is None:
            self._mission_complete = True
            self.get_logger().info(
                f'Mission complete: reached final waypoint {self._waypoints[self._current_index]}')
            return

        if next_index != self._current_index:
            self.get_logger().info(
                f'Reached waypoint {self._current_index} '
                f'({self._waypoints[self._current_index]}), advancing to {next_index}')
            self._current_index = next_index

        self._send_plan_request(position)

    def _send_plan_request(self, position):
        now = self.get_clock().now().to_msg()

        start = PoseStamped()
        start.header.frame_id = self._frame_id
        start.header.stamp = now
        start.pose = self._latest_odom.pose.pose

        goal = PoseStamped()
        goal.header.frame_id = self._frame_id
        goal.header.stamp = now
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = \
            self._waypoints[self._current_index]
        goal.pose.orientation.w = 1.0

        req = GetPlan.Request()
        req.start = start
        req.goal = goal
        req.tolerance = float(self._tolerance)

        future = self._plan_client.call_async(req)
        future.add_done_callback(self._plan_done_cb)

    def _plan_done_cb(self, future):
        try:
            result = future.result()
            if len(result.plan.poses) == 0:
                self.get_logger().warn('Plan returned empty path — waypoint may be unreachable.')
        except Exception as e:
            self.get_logger().error(f'Plan service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSequencer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

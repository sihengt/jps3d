#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, Marker,
)


class InteractiveGoalClient(Node):
    def __init__(self):
        super().__init__('rviz_goal_client')

        self.declare_parameter('odom_topic',
                               '/px4vision_custom_0/mavros/local_position/odom')
        self.declare_parameter('plan_srv_topic', '/plan')
        self.declare_parameter('tolerance', 0.5)
        self.declare_parameter('marker_frame', 'map')
        self.declare_parameter('replan_period', 1.0)
        # This topic is for setting a goal from a ros2 topic.
        self.declare_parameter('goal_xyz_topic', '/goal_xyz')

        odom_topic      = self.get_parameter('odom_topic').get_parameter_value().string_value
        plan_srv_topic  = self.get_parameter('plan_srv_topic').get_parameter_value().string_value
        self._frame     = self.get_parameter('marker_frame').get_parameter_value().string_value
        self._replan_period = self.get_parameter('replan_period').get_parameter_value().double_value
        goal_xyz_topic  = self.get_parameter('goal_xyz_topic').get_parameter_value().string_value
        
        self._latest_odom = None
        self._active = False
        self._first_request_logged = False
        self._first_reply_logged = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # default reliable qos, 10 is the depth
        self.create_subscription(PoseStamped, goal_xyz_topic, self._manual_goal_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, sensor_qos)
        self._plan_client = self.create_client(GetPlan, plan_srv_topic)

        # Interactive marker server
        self._server = InteractiveMarkerServer(self, 'goal_marker')

        # Right click menu that comes out whenever clicked.
        self._menu = MenuHandler()
        self._menu.insert('Plan to here', callback=self._menu_plan_cb)
        self._menu.insert('Move marker to drone', callback=self._menu_reset_cb)
        self._menu.insert('Stop replanning', callback=self._menu_stop_replan_cb)

        # Start marker at a convenient position; user can drag it anywhere
        self._marker_pose = PoseStamped()
        self._marker_pose.header.frame_id = self._frame
        self._marker_pose.pose.position.x = 5.0
        self._marker_pose.pose.position.y = 0.0
        self._marker_pose.pose.position.z = 3.0
        self._marker_pose.pose.orientation.w = 1.0

        self._make_marker()
        self._server.applyChanges()

        self.get_logger().info(
            'Interactive goal client ready.\n'
            '  Drag the sphere in RViz to your goal, then right-click → "Plan to here".\n'
            '  Add topic: /goal_marker/update  (InteractiveMarkers display).')

        if self._replan_period > 0.0:
            self._replan_timer = self.create_timer(self._replan_period, self._replan_tick)
            self.get_logger().info(
                f'Periodic re-plan enabled at {1.0 / self._replan_period:.2f} Hz '
                f'(period {self._replan_period:.2f} s). '
                'First plan must still be triggered with right-click → "Plan to here".')
        else:
            self._replan_timer = None
            self.get_logger().info('Periodic re-plan disabled (replan_period <= 0).')

    # ------------------------------------------------------------------
    def _make_marker(self):
        im = InteractiveMarker()
        im.header.frame_id = self._frame
        im.name = 'goal'
        im.description = 'Drag to goal\nRight-click to plan'
        im.pose = self._marker_pose.pose
        im.scale = 1.0

        # Visible sphere
        sphere_ctrl = InteractiveMarkerControl()
        sphere_ctrl.always_visible = True
        sphere_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_3D

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
        sphere.color.r = 1.0
        sphere.color.g = 0.5
        sphere.color.b = 0.0
        sphere.color.a = 0.85
        sphere_ctrl.markers.append(sphere)
        im.controls.append(sphere_ctrl)

        # XY plane movement (drag in ground plane)
        xy_ctrl = InteractiveMarkerControl()
        xy_ctrl.name = 'move_xy'
        xy_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        xy_ctrl.orientation.w = 1.0
        xy_ctrl.orientation.y = 1.0  # normal = Y axis → XZ plane... we want XY
        im.controls.append(xy_ctrl)

        # Z axis movement (drag up/down)
        z_ctrl = InteractiveMarkerControl()
        z_ctrl.name = 'move_z'
        z_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        z_ctrl.orientation.w = 1.0
        z_ctrl.orientation.z = 1.0  # along Z
        im.controls.append(z_ctrl)

        self._server.insert(im, feedback_callback=self._feedback_cb)
        self._menu.apply(self._server, 'goal')

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg

    def _feedback_cb(self, feedback):
        self._marker_pose.pose = feedback.pose

    def _menu_stop_replan_cb(self, feedback):
        if not self._active:
            self.get_logger().info('Replan already inactive.')
            return
        self._active = False
        self.get_logger().info('Replanning stopped. Right-click → "Plan to here" to resume.')

    def _menu_reset_cb(self, feedback):
        if self._latest_odom is None:
            self.get_logger().warn('No odometry yet.')
            return
        p = self._latest_odom.pose.pose.position
        self._marker_pose.pose.position.x = p.x
        self._marker_pose.pose.position.y = p.y
        self._marker_pose.pose.position.z = p.z
        self._server.setPose('goal', self._marker_pose.pose)
        self._server.applyChanges()
        self.get_logger().info(f'Marker moved to drone position ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')

    def _menu_plan_cb(self, feedback=None):
        if self._latest_odom is None:
            self.get_logger().warn('No odometry received yet — cannot plan.')
            return
        if not self._plan_client.service_is_ready():
            self.get_logger().warn('Plan service not ready — is jps3d_node running?')
            return

        tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        now = self.get_clock().now().to_msg()

        start = PoseStamped()
        start.header.frame_id = self._frame
        start.header.stamp = now
        start.pose = self._latest_odom.pose.pose

        goal = PoseStamped()
        goal.header.frame_id = self._frame
        goal.header.stamp = now
        goal.pose = self._marker_pose.pose
        goal.pose.orientation.w = 1.0

        req = GetPlan.Request()
        req.start = start
        req.goal = goal
        req.tolerance = float(tolerance)

        gp = goal.pose.position
        sp = start.pose.position
        msg = (f'Planning ({sp.x:.2f}, {sp.y:.2f}, {sp.z:.2f}) → '
               f'({gp.x:.2f}, {gp.y:.2f}, {gp.z:.2f})')
        if self._first_request_logged:
            self.get_logger().debug(msg)
        else:
            self.get_logger().info(msg)
            self._first_request_logged = True

        future = self._plan_client.call_async(req)
        future.add_done_callback(self._plan_done_cb)
        self._active = True

    def _plan_done_cb(self, future):
        try:
            result = future.result()
            n = len(result.plan.poses)
            if n == 0:
                self.get_logger().warn('Plan returned empty path — goal may be unreachable.')
                return
            msg = f'Plan received: {n} waypoints.'
            if self._first_reply_logged:
                self.get_logger().debug(msg)
            else:
                self.get_logger().info(msg)
                self._first_reply_logged = True
        except Exception as e:
            self.get_logger().error(f'Plan service call failed: {e}')

    def _manual_goal_cb(self, msg):
        self._marker_pose.header.frame_id = self._frame
        self._marker_pose.pose.position.x = msg.pose.position.x
        self._marker_pose.pose.position.y = msg.pose.position.y
        self._marker_pose.pose.position.z = msg.pose.position.z
        self._marker_pose.pose.orientation.w = 1.0
        self._menu_plan_cb()

    def _replan_tick(self):
        if not self._active:
            return
        if self._latest_odom is None:
            return
        if not self._plan_client.service_is_ready():
            return
        self._menu_plan_cb()


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveGoalClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

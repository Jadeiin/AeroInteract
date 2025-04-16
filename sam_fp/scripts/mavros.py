import rospy
import math
import time
import tf
from geometry_msgs.msg import (
    PoseStamped,
    Point,
    Vector3,
    TransformStamped,
    PointStamped,
)
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong


class DoorTraverseNode:
    """ROS node for autonomous door traversal using visual detection and MAVROS control.

    State Machine Flow:
    INIT -> TAKEOFF -> TRAVERSE -> LANDING

    1. INIT: Enable offboard mode and arm
    2. TAKEOFF: Rise to specified height
    3. TRAVERSE: Execute direct door traversal
    4. LANDING: Controlled descent to ground
    """

    # Configuration constants
    CONTROL_RATE = 20  # Hz
    TRAVERSE_SPEED = 20  # m/s?
    POSITIONING_THRESHOLD = 0.1  # meters
    TAKEOFF_HEIGHT = 1.0  # meters
    DOOR_END_DISTANCE = 2.0  # meters
    DOOR_DETECTION_TIMEOUT = 5.0  # seconds

    def __init__(self):
        """Initialize the door traverse node."""
        rospy.init_node("door_traverse_node")
        self._setup_state_variables()
        self._setup_ros_interface()
        self._setup_tf()
        self.rate = rospy.Rate(self.CONTROL_RATE)
        self.enable_metrics = rospy.get_param("/enable_metrics", False)

    #
    # Setup Methods
    #

    def _setup_state_variables(self):
        """Initialize internal state variables."""
        # Vehicle state
        self.current_state = State()
        self.current_pose = PoseStamped()

        # Door detection state
        self.door_center = None
        self.door_normal = None
        self.last_door_detection = None

        # Fixed door data (set after takeoff)
        self.fixed_door_center = None
        self.fixed_door_normal = None

        # Navigation state
        self.execution_state = "INIT"
        self.hover_start_time = None
        self.traverse_waypoints = None

    def _setup_ros_interface(self):
        """Setup ROS subscribers, publishers and service clients."""
        # Subscribers
        rospy.Subscriber("mavros/state", State, self._state_callback)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self._pose_callback)
        rospy.Subscriber("objects_marker", MarkerArray, self._marker_callback)

        # Publishers
        self.local_pos_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        # Publishers for visualization
        self.target_path_pub = rospy.Publisher(
            "traverse_trajectory/target", Path, queue_size=10
        )
        self.actual_path_pub = rospy.Publisher(
            "traverse_trajectory/actual", Path, queue_size=10
        )
        self.door_markers_pub = rospy.Publisher(
            "door_visualization", MarkerArray, queue_size=10
        )
        self.target_path = Path()
        self.actual_path = Path()
        self.target_path.header.frame_id = "map"
        self.actual_path.header.frame_id = "map"

        # Service clients
        rospy.wait_for_service("mavros/cmd/arming")
        rospy.wait_for_service("mavros/set_mode")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.cmd_client = rospy.ServiceProxy("mavros/cmd/command", CommandLong)

    def _setup_tf(self):
        """Setup TF listener for frame transformations."""
        # TODO: check if transfrom further needed
        # ref: https://github.com/thien94/vision_to_mavros
        self.tf_listener = tf.TransformListener()

    #
    # Callback Methods
    #

    def _state_callback(self, msg):
        """Callback for vehicle state updates."""
        self.current_state = msg

    def _pose_callback(self, msg):
        """Callback for position updates."""
        self.current_pose = msg

    def _marker_callback(self, msg):
        """Process door detection markers and transform to map frame."""
        try:
            if not msg.markers or msg.markers[0].type != msg.markers[0].ARROW:
                return

            marker = msg.markers[0]
            if len(marker.points) < 2:
                return

            # Transform points from camera_link to map frame
            try:
                point1_camera = self._create_point_stamped(
                    marker.points[0], "camera_link", marker.header.stamp
                )
                point2_camera = self._create_point_stamped(
                    marker.points[1], "camera_link", marker.header.stamp
                )

                # Wait for transform to be available
                self.tf_listener.waitForTransform(
                    "map", "camera_link", rospy.Time(), rospy.Duration(1.0)
                )

                point1_map = self.tf_listener.transformPoint("map", point1_camera)
                point2_map = self.tf_listener.transformPoint("map", point2_camera)

                self.door_center = point1_map.point
                self.door_normal = self._calculate_normal_vector(
                    point1_map.point, point2_map.point
                )
                self.last_door_detection = rospy.Time.now()

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ) as e:
                rospy.logwarn(f"TF Error: {e}")

        except Exception as e:
            rospy.logwarn(f"Error processing marker: {e}")

    #
    # Vehicle Control Methods
    #

    def _enable_offboard_mode(self):
        """Set vehicle to offboard mode."""
        if self.current_state.mode != "OFFBOARD":
            if self.set_mode_client(custom_mode="OFFBOARD"):
                rospy.loginfo("Offboard mode enabled")
                return True
            rospy.logwarn("Failed to set OFFBOARD mode")
            return False
        return True

    def _arm_vehicle(self):
        """Arm the vehicle."""
        if not self.current_state.armed:
            if self.arming_client(True):
                rospy.loginfo("Vehicle armed")
                return True
            rospy.logwarn("Failed to arm vehicle")
            return False
        return True

    def _publish_setpoint(self, position, orientation=None, stamp=None):
        """Publish a setpoint position."""
        # TODO: use setpoint_raw instead of setpoint_position
        # This will allow for more control over velocity, acceleration and yaw
        # original thought was to use setpoint_position and setpoint_velocity
        pose = PoseStamped()
        pose.header.stamp = stamp if stamp else rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = position
        pose.pose.orientation = (
            orientation if orientation else self.current_pose.pose.orientation
        )
        self.local_pos_pub.publish(pose)

    #
    # Navigation Helper Methods
    #

    def _create_point_stamped(self, point, frame_id, stamp=None):
        """Create a PointStamped message with header."""
        point_stamped = PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = stamp if stamp else rospy.Time.now()
        point_stamped.point = point
        return point_stamped

    def _calculate_normal_vector(self, point1, point2):
        """Calculate normalized vector between two points."""
        normal = Point()
        normal.x = point2.x - point1.x
        normal.y = point2.y - point1.y
        normal.z = point2.z - point1.z

        magnitude = math.sqrt(normal.x**2 + normal.y**2 + normal.z**2)
        if magnitude > 0:
            normal.x /= magnitude
            normal.y /= magnitude
            normal.z /= magnitude
        return normal

    def _calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt(
            (point1.x - point2.x) ** 2
            + (point1.y - point2.y) ** 2
            + (point1.z - point2.z) ** 2
        )

    def _calculate_door_position(self, distance, use_fixed_data=True):
        """Calculate position relative to door center using primary axis."""
        current_pos = self.current_pose.pose.position

        # Use fixed door data during traverse
        door_center = self.fixed_door_center if use_fixed_data else self.door_center
        door_normal = self.fixed_door_normal if use_fixed_data else self.door_normal

        # Calculate movement vector from quadrotor to door center
        diff_x = door_center.x - current_pos.x
        diff_y = door_center.y - current_pos.y

        # Determine primary axis based on largest positional difference
        abs_x = abs(diff_x)
        abs_y = abs(diff_y)

        point = Point()
        if abs_x > abs_y:
            # X is primary axis - maintain y coordinate
            point.x = door_center.x + door_normal.x * distance
            point.y = door_center.y
        else:
            # Y is primary axis - maintain x coordinate
            point.x = door_center.x
            point.y = door_center.y + door_normal.y * distance
        point.z = door_center.z
        return point

    def _update_trajectories(self, current_pos, target_pos):
        """Update and publish target and actual trajectory paths."""
        now = rospy.Time.now()

        # Create PoseStamped for current position
        current_pose = PoseStamped()
        current_pose.header.frame_id = "map"
        current_pose.header.stamp = now
        current_pose.pose.position = current_pos
        current_pose.pose.orientation = self.current_pose.pose.orientation

        # Create PoseStamped for target position
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = now
        target_pose.pose.position = target_pos
        target_pose.pose.orientation = self.current_pose.pose.orientation

        # Update target path (green)
        self.target_path.header.stamp = now
        self.target_path.poses.append(target_pose)

        # Update actual path (blue)
        self.actual_path.header.stamp = now
        self.actual_path.poses.append(current_pose)

        # Publish updated paths
        self.target_path_pub.publish(self.target_path)
        self.actual_path_pub.publish(self.actual_path)

    #
    # State Handler Methods
    #

    def _handle_init_state(self):
        """Initialize vehicle for autonomous operation."""
        # Publish setpoints before enabling offboard
        for _ in range(100):
            if rospy.is_shutdown():
                return
            self._publish_setpoint(self.current_pose.pose.position)
            self.rate.sleep()

        # Enable offboard mode and arm
        if self._enable_offboard_mode() and self._arm_vehicle():
            self.execution_state = "TAKEOFF"
            rospy.loginfo("Initialization complete, starting takeoff")

    def _handle_takeoff_state(self):
        """Execute takeoff sequence."""
        # TODO: use TAKEOFF command instead of manual control
        current_pos = self.current_pose.pose.position
        takeoff_pos = Point()
        takeoff_pos.x = current_pos.x
        takeoff_pos.y = current_pos.y
        takeoff_pos.z = self.TAKEOFF_HEIGHT

        # Check if we've reached takeoff height
        if abs(current_pos.z - self.TAKEOFF_HEIGHT) < self.POSITIONING_THRESHOLD:
            self.execution_state = "TRAVERSE"
            rospy.loginfo("Takeoff complete, starting door traverse")

        self._publish_setpoint(takeoff_pos)

    def _handle_traverse_state(self):
        """Execute direct door traversal."""
        if not self.door_center or not self.door_normal:
            # Hold position while waiting for door detection
            self._publish_setpoint(self.current_pose.pose.position)
            rospy.loginfo_throttle(5.0, "Waiting for door detection")
            return

        # First time in traverse with valid door data - store fixed values
        if not self.fixed_door_center:
            self.fixed_door_center = self.door_center
            self.fixed_door_normal = self.door_normal
            rospy.loginfo("Door data fixed for traverse")

            # Create markers for door center and end position
            markers = MarkerArray()

            # Door center marker (red sphere)
            center_marker = Marker()
            center_marker.header.frame_id = "map"
            center_marker.header.stamp = rospy.Time.now()
            center_marker.ns = "door_points"
            center_marker.id = 0
            center_marker.type = Marker.SPHERE
            center_marker.action = Marker.ADD
            center_marker.pose.position = self.fixed_door_center
            center_marker.pose.orientation.w = 1.0
            center_marker.scale.x = 0.1
            center_marker.scale.y = 0.1
            center_marker.scale.z = 0.1
            center_marker.color.r = 1.0
            center_marker.color.g = 0.0
            center_marker.color.b = 0.0
            center_marker.color.a = 1.0
            center_marker.lifetime = rospy.Duration()
            markers.markers.append(center_marker)

            # End position marker (blue sphere)
            end_pos = self._calculate_door_position(self.DOOR_END_DISTANCE)
            end_marker = Marker()
            end_marker.header.frame_id = "map"
            end_marker.header.stamp = rospy.Time.now()
            end_marker.ns = "door_points"
            end_marker.id = 1
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose.position = end_pos
            end_marker.pose.orientation.w = 1.0
            end_marker.scale.x = 0.1
            end_marker.scale.y = 0.1
            end_marker.scale.z = 0.1
            end_marker.color.r = 0.0
            end_marker.color.g = 0.0
            end_marker.color.b = 1.0
            end_marker.color.a = 1.0
            end_marker.lifetime = rospy.Duration()
            markers.markers.append(end_marker)

            # Publish marker array
            self.door_markers_pub.publish(markers)
            rospy.loginfo(f"Door center: {self.fixed_door_center}")
            rospy.loginfo(f"Door normal: {self.fixed_door_normal}")

        # Calculate end position beyond the door
        end_pos = self._calculate_door_position(self.DOOR_END_DISTANCE)
        traverse_height = self.fixed_door_center.z
        current_pos = self.current_pose.pose.position

        # Calculate distance to end point and normalize to get speed scale
        distance_to_end = self._calculate_distance(current_pos, end_pos)

        # Adjust speed scale based on distance to end point
        factor = 1.0
        if distance_to_end > 1.0 and distance_to_end < 2.0:
            factor = 0.5
        elif distance_to_end < 1.0:
            factor = 0.25

        # Create next setpoint
        target = Point()
        direction = self._calculate_normal_vector(current_pos, end_pos)
        step_size = factor * self.TRAVERSE_SPEED / self.CONTROL_RATE

        target.x = current_pos.x + direction.x * step_size
        target.y = current_pos.y + direction.y * step_size
        target.z = traverse_height

        # Calculate desired yaw orientation based on movement direction
        yaw = math.atan2(direction.y, direction.x)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        orientation = self.current_pose.pose.orientation
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]

        # Update and publish trajectories
        self._update_trajectories(current_pos, target)

        # Check if we've reached the end
        if distance_to_end < step_size:
            target = end_pos
            # Update trajectories with final position
            self._update_trajectories(current_pos, end_pos)
            rospy.loginfo("Traverse complete")
            self.execution_state = "LANDING"

        self._publish_setpoint(target, orientation)

    def _handle_landing_state(self):
        """Execute controlled landing sequence."""
        current_pos = self.current_pose.pose.position
        landing_pos = Point()
        landing_pos.x = current_pos.x
        landing_pos.y = current_pos.y
        landing_pos.z = 0.0  # Ground level

        # Check if we've reached ground level
        if current_pos.z < 0.1:  # 10cm threshold
            if self.current_state.armed:
                # Disarm the vehicle
                self.arming_client(False)
                rospy.loginfo_throttle(5.0, "Landing complete, vehicle disarmed")
            return

        self._publish_setpoint(landing_pos)

    def run(self):
        """Main execution loop."""
        # Wait for FCU connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        while not rospy.is_shutdown():
            try:
                # Check for door detection timeout
                if (
                    self.last_door_detection
                    and (rospy.Time.now() - self.last_door_detection).to_sec()
                    > self.DOOR_DETECTION_TIMEOUT
                ):
                    self.door_center = None
                    self.door_normal = None
                    rospy.logwarn("Door detection timeout")

                # State machine
                if self.execution_state == "INIT":
                    self._handle_init_state()
                elif self.execution_state == "TAKEOFF":
                    self._handle_takeoff_state()
                elif self.execution_state == "TRAVERSE":
                    self._handle_traverse_state()
                elif self.execution_state == "LANDING":
                    self._handle_landing_state()

            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                # Reset to safe state
                if self.execution_state not in ["INIT", "TAKEOFF", "LANDING"]:
                    self.execution_state = "LANDING"

            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = DoorTraverseNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

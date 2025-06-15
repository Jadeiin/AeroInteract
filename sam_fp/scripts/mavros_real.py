import rospy
import math
import tf
from geometry_msgs.msg import (
    PoseStamped,
    Point,
    PointStamped,
)
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from mavros_msgs.msg import State


class DoorTraverseNodeReal:
    """ROS node for real-world autonomous door traversal using visual detection and MAVROS control.

    State Machine Flow:
    WAIT_FOR_POSITION_HOLD -> TRAVERSE

    1. WAIT_FOR_POSITION_HOLD: Wait for pilot to manually enable OFFBOARD mode
    2. TRAVERSE: Execute direct door traversal after OFFBOARD mode is enabled
    """

    # Configuration constants
    CONTROL_RATE = 20  # Hz
    TRAVERSE_SPEED = 0.5  # m/s
    DOOR_END_DISTANCE = 2.0  # meters
    DOOR_DETECTION_TIMEOUT = 5.0  # seconds

    def __init__(self):
        """Initialize the door traverse node."""
        rospy.init_node("door_traverse_node_real")
        self._setup_state_variables()
        self._setup_ros_interface()
        self._setup_tf()
        self.rate = rospy.Rate(self.CONTROL_RATE)

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

        # Fixed door data (set after switching to offboard)
        self.fixed_door_center = None
        self.fixed_door_normal = None
        self.dominant_axis = None  # 'x' or 'y'
        self.initial_side = None   # True if UAV starts before door center

        # Navigation state
        self.execution_state = "WAIT_FOR_POSITION_HOLD"

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

    def _setup_tf(self):
        """Setup TF listener for frame transformations."""
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
    # Helper Methods
    #

    def _publish_setpoint(self, position, orientation=None, stamp=None):
        """Publish a setpoint position."""
        pose = PoseStamped()
        pose.header.stamp = stamp if stamp else rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = position
        pose.pose.orientation = (
            orientation if orientation else self.current_pose.pose.orientation
        )
        self.local_pos_pub.publish(pose)

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
        """Calculate target position beyond the door using stored alignment."""
        # Use fixed door center if available
        door_center = self.fixed_door_center if use_fixed_data else self.door_center

        # Initialize target at door center
        target = Point()
        target.x = door_center.x
        target.y = door_center.y
        target.z = door_center.z

        # Apply offset based on stored alignment
        if self.dominant_axis == 'x':
            target.x += distance if self.initial_side else -distance
        else:
            target.y += distance if self.initial_side else -distance

        return target

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

        # Update paths
        self.target_path.header.stamp = now
        self.target_path.poses.append(target_pose)
        self.actual_path.header.stamp = now
        self.actual_path.poses.append(current_pose)

        # Publish updated paths
        self.target_path_pub.publish(self.target_path)
        self.actual_path_pub.publish(self.actual_path)

    #
    # State Handler Methods
    #

    def _handle_position_hold_state(self):
        """Send setpoints and wait for OFFBOARD mode."""
        # Publish several setpoints at current position before switching to OFFBOARD
        # This ensures a smooth transition to OFFBOARD mode
        self._publish_setpoint(self.current_pose.pose.position)

        if self.current_state.mode == "OFFBOARD":
            self.execution_state = "TRAVERSE"
            rospy.loginfo("Offboard mode detected, starting traverse")
        else:
            rospy.loginfo_throttle(
                5.0, "Publishing setpoints, waiting for OFFBOARD mode switch..."
            )

    def _handle_traverse_state(self):
        """Execute direct door traversal."""
        if self.current_state.mode != "OFFBOARD":
            # Switch to OFFBOARD mode if not already in it
            self.execution_state = "WAIT_FOR_POSITION_HOLD"
            rospy.loginfo("Switching back to WAIT_FOR_POSITION_HOLD state")
            return

        if not self.door_center and not self.fixed_door_center:
            # Hold position while waiting for door detection
            self._publish_setpoint(self.current_pose.pose.position)
            rospy.loginfo_throttle(5.0, "Waiting for door detection")
            return

        # First time in traverse with valid door data - store fixed values and initial alignment
        if not self.fixed_door_center:
            # Store door data
            self.fixed_door_center = self.door_center
            self.fixed_door_normal = self.door_normal

            # Determine dominant axis
            current_pos = self.current_pose.pose.position
            dx = abs(self.door_center.x - current_pos.x)
            dy = abs(self.door_center.y - current_pos.y)
            self.dominant_axis = 'x' if dx > dy else 'y'

            # Record which side we're starting from
            if self.dominant_axis == 'x':
                self.initial_side = current_pos.x < self.door_center.x
            else:
                self.initial_side = current_pos.y < self.door_center.y

            rospy.loginfo(f"Door data fixed. Dominant axis: {self.dominant_axis}, Starting {'before' if self.initial_side else 'after'} door")

        # Update door data if detection is close to fixed position
        if (
            self.door_center
            and self._calculate_distance(self.fixed_door_center, self.door_center) < 0.1
        ):
            self.fixed_door_center = self.door_center
            self.fixed_door_normal = self.door_normal

            # Visualize door center and end position
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
            center_marker.color.a = 1.0
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
            end_marker.color.b = 1.0
            end_marker.color.a = 1.0
            markers.markers.append(end_marker)

            self.door_markers_pub.publish(markers)

        # Calculate end position beyond the door
        end_pos = self._calculate_door_position(self.DOOR_END_DISTANCE)
        traverse_height = self.fixed_door_center.z
        current_pos = self.current_pose.pose.position

        # Calculate distance to end point and normalize to get speed scale
        distance_to_door = self._calculate_distance(current_pos, self.fixed_door_center)

        # Adjust speed scale based on distance
        if distance_to_door >= 1.0:
            factor = 1.0
        elif distance_to_door <= 0.5:
            factor = 0.25
        else:
            # Linear interpolation between 1.0 and 0.25
            factor = 0.25 + 0.75 * (distance_to_door - 0.5) / 0.5

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

        self._publish_setpoint(target, orientation)

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
                    rospy.logwarn_throttle(5.0, "Door detection timeout")

                # State machine
                if self.execution_state == "WAIT_FOR_POSITION_HOLD":
                    self._handle_position_hold_state()
                elif self.execution_state == "TRAVERSE":
                    self._handle_traverse_state()

            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                # Hold position on error
                if self.current_state.mode == "OFFBOARD":
                    self._publish_setpoint(self.current_pose.pose.position)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = DoorTraverseNodeReal()
        node.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
import math
import time
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, Vector3, TransformStamped
from visualization_msgs.msg import MarkerArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong


class DoorTraverseNode:
    """ROS node for autonomous door traversal using visual detection and MAVROS control.

    State Machine Flow:
    INIT -> TAKEOFF -> POSITIONING -> HOVERING -> TRAVERSE

    1. INIT: Enable offboard mode and arm
    2. TAKEOFF: Rise to specified height
    3. POSITIONING: Move to hover position in front of door
    4. HOVERING: Stabilize and prepare traverse
    5. TRAVERSE: Execute pre-calculated door traversal
    """

    # Configuration constants
    CONTROL_RATE = 20  # Hz
    HOVER_DURATION = 5.0  # seconds
    TRAVERSE_SPEED = 2.0  # m/s
    POSITIONING_THRESHOLD = 0.1  # meters
    TAKEOFF_HEIGHT = 1.0  # meters
    DOOR_HOVER_DISTANCE = 1.0  # meters
    DOOR_END_DISTANCE = 0.2  # meters
    DOOR_DETECTION_TIMEOUT = 5.0  # seconds

    def __init__(self):
        """Initialize the door traverse node."""
        rospy.init_node("door_traverse_node")
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

        # Publisher
        self.local_pos_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10
        )

        # Service clients
        rospy.wait_for_service("mavros/cmd/arming")
        rospy.wait_for_service("mavros/set_mode")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.cmd_client = rospy.ServiceProxy("mavros/cmd/command", CommandLong)

    def _setup_tf(self):
        """Setup TF2 for frame transformations."""
        # TODO: check if transfrom further needed
        # ref: https://github.com/thien94/vision_to_mavros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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
                    marker.points[0], "camera_link"
                )
                point2_camera = self._create_point_stamped(
                    marker.points[1], "camera_link"
                )

                point1_map = self.tf_buffer.transform(
                    point1_camera, "map", rospy.Duration(1.0)
                )
                point2_map = self.tf_buffer.transform(
                    point2_camera, "map", rospy.Duration(1.0)
                )

                self.door_center = point1_map.point
                self.door_normal = self._calculate_normal_vector(
                    point1_map.point, point2_map.point
                )
                self.last_door_detection = rospy.Time.now()

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
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

    def _publish_setpoint(self, position, orientation=None):
        """Publish a setpoint position."""
        # TODO: use setpoint_raw instead of setpoint_position
        # This will allow for more control over velocity, acceleration and yaw
        # original thought was to use setpoint_position and setpoint_velocity
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = position
        pose.pose.orientation = (
            orientation if orientation else self.current_pose.pose.orientation
        )
        self.local_pos_pub.publish(pose)

    #
    # Navigation Helper Methods
    #

    def _create_point_stamped(self, point, frame_id):
        """Create a PointStamped message with header."""
        point_stamped = tf2_geometry_msgs.PointStamped()
        point_stamped.header.frame_id = frame_id
        point_stamped.header.stamp = rospy.Time.now()
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

    def _calculate_door_position(self, distance):
        """Calculate position relative to door center."""
        point = Point()
        point.x = self.door_center.x + self.door_normal.x * distance
        point.y = self.door_center.y + self.door_normal.y * distance
        point.z = self.door_center.z
        return point

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
            self.execution_state = "POSITIONING"
            rospy.loginfo("Takeoff complete, starting door approach")

        self._publish_setpoint(takeoff_pos)

    def _handle_positioning_state(self):
        """Move to hover position in front of door."""
        if not self.door_center or not self.door_normal:
            # Hold position while waiting for door detection
            self._publish_setpoint(self.current_pose.pose.position)
            rospy.loginfo_throttle(5.0, "Waiting for door detection")
            return

        # Calculate hover position
        hover_pos = self._calculate_door_position(-self.DOOR_HOVER_DISTANCE)

        # Check if we've reached hover position
        if (
            self._calculate_distance(self.current_pose.pose.position, hover_pos)
            < self.POSITIONING_THRESHOLD
        ):
            self.hover_start_time = time.time()
            self.execution_state = "HOVERING"
            rospy.loginfo("Reached hover position, stabilizing")

        self._publish_setpoint(hover_pos)

    def _handle_hovering_state(self):
        """Stabilize at hover position and prepare for traverse."""
        if not self.door_center or not self.door_normal:
            self.execution_state = "POSITIONING"
            rospy.logwarn("Lost door detection, returning to positioning")
            return

        # Continue hovering at current position
        hover_pos = self._calculate_door_position(-self.DOOR_HOVER_DISTANCE)
        self._publish_setpoint(hover_pos)

        # After hover duration, prepare and start traverse
        if time.time() - self.hover_start_time >= self.HOVER_DURATION:
            start_pos = self._calculate_door_position(-self.DOOR_HOVER_DISTANCE)
            end_pos = self._calculate_door_position(self.DOOR_END_DISTANCE)
            self.traverse_waypoints = (start_pos, end_pos, self.door_center.z)
            self.execution_state = "TRAVERSE"
            rospy.loginfo("Starting door traverse")

    def _set_speed(self, scale, speed_type=1):
        """Send MAV_CMD_DO_CHANGE_SPEED command.

        Args:
            scale (float): Speed scale factor between 0.0 and 1.0
            speed_type (int, optional): 0=airspeed, 1=groundspeed, 2=climb speed, 3=descent speed.
                                      Defaults to groundspeed.
        """
        # MAV_CMD_DO_CHANGE_SPEED parameters:
        # param1: speed type
        # param2: speed in m/s
        # param3: -1 (no change to throttle)
        # param4: absolute or relative [0=absolute, 1=relative]
        try:
            # Clamp scale between 0 and 1
            scale = max(0.0, min(1.0, scale))
            speed = self.TRAVERSE_SPEED * scale

            self.cmd_client(
                command=178,  # MAV_CMD_DO_CHANGE_SPEED
                param1=float(speed_type),
                param2=float(speed),
                param3=-1.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0,
            )
            rospy.loginfo(f"Speed scale set to {scale:.2f} ({speed:.2f} m/s)")
        except rospy.ServiceException as e:
            rospy.logerr(f"Speed change failed: {e}")

    def _handle_traverse_state(self):
        """Execute door traversal using pre-calculated waypoints."""
        if not self.traverse_waypoints:
            self.execution_state = "POSITIONING"
            rospy.logwarn("Traverse waypoints not found, restarting")
            return

        start_pos, end_pos, traverse_height = self.traverse_waypoints
        current_pos = self.current_pose.pose.position

        # Calculate distance to end point and normalize to get speed scale
        distance_to_end = self._calculate_distance(current_pos, end_pos)

        # Adjust speed scale based on distance to end point
        if distance_to_end > 2.0:
            # Full speed when far from door
            self._set_speed(1.0)
        elif distance_to_end > 1.0:
            # 50% speed when approaching door
            self._set_speed(0.5)
        else:
            # 25% speed for final approach
            self._set_speed(0.25)

        # Create next setpoint
        target = Point()
        direction = self._calculate_normal_vector(current_pos, end_pos)
        step_size = self.TRAVERSE_SPEED / self.CONTROL_RATE

        target.x = current_pos.x + direction.x * step_size
        target.y = current_pos.y + direction.y * step_size
        target.z = traverse_height

        # Check if we've reached the end
        if distance_to_end < step_size:
            target = end_pos
            rospy.loginfo("Traverse complete")
            # Could transition to a new state here

        self._publish_setpoint(target)

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
                elif self.execution_state == "POSITIONING":
                    self._handle_positioning_state()
                elif self.execution_state == "HOVERING":
                    self._handle_hovering_state()
                elif self.execution_state == "TRAVERSE":
                    self._handle_traverse_state()

            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                # Reset to safe state if not in initialization or takeoff
                if self.execution_state not in ["INIT", "TAKEOFF"]:
                    self.execution_state = "POSITIONING"

            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = DoorTraverseNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

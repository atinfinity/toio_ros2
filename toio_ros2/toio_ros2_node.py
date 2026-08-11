# Copyright (C) 2025 atinfinity
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import math
import threading
import time

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import ColorRGBA, UInt8
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from toio import (Battery, BLEScanner, Color, CubeLocation, IdInformation,
                  IndicatorParam, MidiNote, Motor, MotorResponseCode,
                  MovementType, Point, PositionId, ResponseMotorControlTarget,
                  RotationOption, SoundId, Speed, SpeedChangeType,
                  TargetPosition, ToioCoreCube)
from toio_msgs.msg import Led, LedPattern, Melody
from toio_msgs.msg import MidiNote as MidiNoteMsg

# Auto-connect mode reports up to this many nearby cubes as cube_id
# candidates. BLEScanner.scan(num) never stops early at num cubes (toio.py
# only truncates the result list after the full 5s scan timeout), so a
# larger value costs nothing while a small one hides cubes (issue #18).
AUTO_CONNECT_SCAN_NUM = 10


class ToioNode(Node):
    # Floor for the BLE send intervals. A single write plus its response takes
    # roughly 60ms on this link (issue #31 measured /toio/pose at 20-57Hz), so
    # asking for less than this only queues work the radio cannot deliver.
    MIN_SEND_INTERVAL = 0.02  # seconds

    # Every motor command carries this auto-stop duration, so a drive command
    # has to be resent before it expires or the cube stops mid-motion
    MOTOR_DURATION_MS = 500

    # Send-rate parameters and their bounds, as (minimum, maximum). Only the
    # motor resend has an upper bound, from MOTOR_DURATION_MS above.
    SEND_INTERVAL_BOUNDS = {
        'led_write_interval': (MIN_SEND_INTERVAL, None),
        'sound_min_interval': (MIN_SEND_INTERVAL, None),
        'motor_dedup_interval': (MIN_SEND_INTERVAL, MOTOR_DURATION_MS / 1000.0),
    }

    # How long past the cube's own dock_timeout a dock still waits for the
    # motor response before giving up. A notification lost to a flaky BLE
    # link must not leave the caller (Open-RMF) hanging forever.
    DOCK_RESPONSE_GRACE = 5.0  # seconds

    # How often the dock waits on its completion event, which is also how
    # quickly a cancel request and a BLE disconnection are noticed
    DOCK_POLL_INTERVAL = 0.1  # seconds

    def __init__(self, **kwargs) -> None:
        super().__init__('toio_ros2_node', **kwargs)
        # https://toio.github.io/toio-spec/en/docs/hardware_shape
        self.wheel_base = 0.0266  # meter
        self.wheel_radius = 0.00625  # meter
        self.cube_height = 0.0256  # meter

        # https://toio.github.io/toio-spec/en/docs/ble_motor
        self.max_rpm = 494.0
        self.max_input_speed = 115.0
        self.is_connected = False
        self.connect_timeout = 10.0  # seconds
        self.reconnect_interval = 3.0  # seconds
        # Serializes the connected -> disconnected transition between the
        # rclpy executor thread (1Hz monitor) and the asyncio loop thread
        # (motor send failure), so only one reconnection is ever scheduled
        self._reconnect_lock = threading.Lock()

        # Command deduplication with time-based resend
        self._last_motor_cmd: tuple = (0, 0)
        self._last_motor_cmd_time: float = 0.0

        # Latest-command-only pattern for cmd_vel: the subscriber only stores
        # the newest command and motor_command_loop() sends it, so BLE writes
        # never queue up behind a fast publisher
        self._latest_cmd_vel: tuple = None
        self._latest_cmd_vel_stamp: float = 0.0
        self.cmd_vel_timeout = 0.5  # seconds, matches motor duration_ms=500

        # Latest-command-only pattern for the indicator (issue #28), the same
        # as cmd_vel: a fast publisher must not queue up BLE writes behind the
        # 20Hz motor loop. The color is a state rather than an event, so the
        # newest one is coalesced instead of dropped -- dropping an 'off'
        # command would leave the cube lit for good.
        self._pending_led: tuple = None
        self._last_led_cmd: tuple = None
        self._last_led_pattern_time: float = 0.0

        # A sound is an event and cannot be coalesced, so commands arriving
        # faster than this are dropped instead
        self._last_sound_time: float = 0.0

        # Docking state (issue toio_fleet_adapter#3). Claimed in the action
        # server's goal callback and released in its execute callback, and
        # read by cmd_vel_callback / motor_command_loop so nothing writes to
        # the motor while the cube is running a target motion.
        self._dock_lock = threading.Lock()
        self._docking = False
        self._dock_done = threading.Event()
        self._dock_response = None
        self._dock_started_at: float = 0.0
        # Latest published position, used for the dock action feedback
        self._last_pose_xy: tuple = None

        # Default is a param for A4 mat https://toio.github.io/toio-spec/docs/hardware_position_id
        self.declare_parameter('field_min_x', 98.0)
        self.declare_parameter('field_max_x', 402.0)
        self.declare_parameter('field_min_y', 142.0)
        self.declare_parameter('field_max_y', 358.0)
        self.declare_parameter('field_width_meter', 0.297)
        self.declare_parameter('field_height_meter', 0.210)

        # Params for goal_pose motion, shared with the dock_to_pose action
        self.declare_parameter('goal_max_speed', 30)
        self.declare_parameter('goal_timeout', 60)
        # A dock covers a few centimetres at most, so it gets its own much
        # shorter timeout. With goal_timeout a cube that cannot reach the
        # target - because something is standing on it - keeps pushing for a
        # full minute before the motion is abandoned.
        self.declare_parameter('dock_timeout', 10)
        # Margin (in Position ID units) kept between a clamped goal and the
        # mat boundary so the cube's ID sensor stays in the readable area
        self.declare_parameter('goal_boundary_margin', 10)

        # Cube identification (issue #14). Both empty (default): connect to
        # the nearest cube found by the scan. cube_id is matched as a substring
        # of the BLE local name, which is 'toio Core Cube-XXX' or
        # 'toio-XXX (toio Core Cube)' depending on the cube, and is platform
        # independent; cube_address is a MAC address (Linux/Windows) or a
        # CoreBluetooth UUID (macOS). cube_id takes precedence when both are set.
        self.declare_parameter('cube_id', '')
        self.declare_parameter('cube_address', '')

        # TF frame prefix for multi-cube setups (issue #15): the published
        # transform becomes map -> <frame_prefix>center. Use the same value
        # as the frame_prefix of robot_state_publisher (e.g. 'toio1/').
        self.declare_parameter('frame_prefix', '')

        # The cube's built-in target motion (goal_pose topic) moves the cube
        # on a path that no external planner knows about. Disable it when an
        # external traffic authority (e.g. Open-RMF) owns the motion plan and
        # all movement must go through Nav2 cmd_vel instead.
        #
        # This does NOT disable the dock_to_pose action, which uses the same
        # built-in motion; see the comment where that server is created.
        self.declare_parameter('enable_goal_pose_motion', True)

        # Visual / audible feedback (issue #28). led_duration_ms 0 keeps the
        # indicator lit until the next command; 10-2550 lets the cube turn it
        # off on its own (a fraction below 10ms is truncated and anything above
        # 2550ms is clipped by toio.py). sound_volume is mute or full volume
        # only, as the cube takes 0 as mute and every other value as the
        # maximum volume.
        self.declare_parameter('led_duration_ms', 0)
        self.declare_parameter('sound_volume', 255)

        # BLE send rate limits. Both default to the values these were fixed at
        # while they were node constants, which one cube on an A4 mat had no
        # trouble with (issue #31). They are parameters so a deployment that
        # does run into trouble - more cubes sharing the radio, LED driven as
        # a status display, sounds played back to back - can trade update rate
        # against BLE bandwidth without editing the node.
        #
        # led_write_interval is how often led_command_loop() sends the latest
        # color; sound_min_interval throttles sound commands, and anything
        # arriving inside it is dropped rather than queued.
        # motor_dedup_interval is how often the same non-stop motor command is
        # resent. It must stay under MOTOR_DURATION_MS or the cube auto-stops
        # before the resend arrives and the robot stutters.
        self.declare_parameter('led_write_interval', 0.1)
        self.declare_parameter('sound_min_interval', 0.1)
        self.declare_parameter('motor_dedup_interval', 0.3)

        # Get params for field information
        self.field_min_x = self.get_parameter('field_min_x').get_parameter_value().double_value
        self.field_max_x = self.get_parameter('field_max_x').get_parameter_value().double_value
        self.field_min_y = self.get_parameter('field_min_y').get_parameter_value().double_value
        self.field_max_y = self.get_parameter('field_max_y').get_parameter_value().double_value
        self.field_width_meter = self.get_parameter(
            'field_width_meter').get_parameter_value().double_value
        self.field_height_meter = self.get_parameter(
            'field_height_meter').get_parameter_value().double_value

        # Get params for goal_pose motion
        self.goal_max_speed = self.get_parameter(
            'goal_max_speed').get_parameter_value().integer_value
        self.goal_timeout = self.get_parameter(
            'goal_timeout').get_parameter_value().integer_value
        self.dock_timeout = self.get_parameter(
            'dock_timeout').get_parameter_value().integer_value
        self.goal_boundary_margin = self.get_parameter(
            'goal_boundary_margin').get_parameter_value().integer_value
        self.cube_id = self.get_parameter('cube_id').get_parameter_value().string_value
        self.cube_address = self.get_parameter(
            'cube_address').get_parameter_value().string_value
        self.frame_prefix = self.get_parameter(
            'frame_prefix').get_parameter_value().string_value
        self.enable_goal_pose_motion = self.get_parameter(
            'enable_goal_pose_motion').get_parameter_value().bool_value
        self.led_duration_ms = self.get_parameter(
            'led_duration_ms').get_parameter_value().integer_value
        self.sound_volume = self.get_parameter(
            'sound_volume').get_parameter_value().integer_value
        self.led_write_interval = self.clamp_send_interval(
            'led_write_interval',
            self.get_parameter(
                'led_write_interval').get_parameter_value().double_value)
        self.sound_min_interval = self.clamp_send_interval(
            'sound_min_interval',
            self.get_parameter(
                'sound_min_interval').get_parameter_value().double_value)
        self.motor_dedup_interval = self.clamp_send_interval(
            'motor_dedup_interval',
            self.get_parameter(
                'motor_dedup_interval').get_parameter_value().double_value)
        # led_command_loop() and the sound throttle read these every time, so
        # setting them at runtime takes effect without a restart
        self.add_on_set_parameters_callback(self.on_set_parameters)

        # calculate scale
        self.scale_x = self.field_width_meter / (self.field_max_x - self.field_min_x)
        self.scale_y = self.field_height_meter / (self.field_max_y - self.field_min_y)
        self.get_logger().debug(f'scale_x = {self.scale_x}, scale_y = {self.scale_y}')

        # subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        if self.enable_goal_pose_motion:
            self.goal_pose_sub = self.create_subscription(
                PoseStamped,
                'goal_pose',
                self.goal_pose_callback,
                10)
        else:
            self.goal_pose_sub = None
            self.get_logger().info(
                'goal_pose motion is disabled (enable_goal_pose_motion=false)')
        self.led_sub = self.create_subscription(
            ColorRGBA,
            'toio/led',
            self.led_callback,
            10)
        self.sound_sub = self.create_subscription(
            UInt8,
            'toio/sound',
            self.sound_callback,
            10)
        # Separate topics rather than replacing the two above: toio_gazebo
        # subscribes to toio/sound, and a plain color is the common case that
        # should stay publishable with `ros2 topic pub` and std_msgs alone
        self.led_timed_sub = self.create_subscription(
            Led,
            'toio/led_timed',
            self.led_timed_callback,
            10)
        self.led_pattern_sub = self.create_subscription(
            LedPattern,
            'toio/led_pattern',
            self.led_pattern_callback,
            10)
        self.melody_sub = self.create_subscription(
            Melody,
            'toio/melody',
            self.melody_callback,
            10)

        # publisher
        self.toio_pose_pub = self.create_publisher(PoseStamped, 'toio/pose', qos_profile=10)
        self.toio_battery_state_pub = self.create_publisher(
            BatteryState, 'toio/battery_state', qos_profile=10)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # action server
        #
        # Precise final positioning requested by an external traffic
        # authority (Open-RMF docking, toio_fleet_adapter#3).
        #
        # Deliberately NOT gated on enable_goal_pose_motion. That parameter
        # closes the goal_pose topic, where anyone can make the cube drive a
        # built-in target motion along a path no planner knows about, at a
        # time no planner chose. This action inverts all three: the traffic
        # authority issues it itself, at a waypoint it has already reserved,
        # over a few centimetres, and it waits for the result before doing
        # anything else. Gating it here would leave the fleet adapter with no
        # way to improve on the Nav2 goal tolerance.
        #
        # ReentrantCallbackGroup, not the default mutually exclusive one: the
        # cancel request arrives on this same server while the execute
        # callback is blocked waiting for the cube, so an exclusive group
        # would queue the cancel behind the goal it is meant to cancel.
        self._dock_cb_group = ReentrantCallbackGroup()
        self.dock_action_server = ActionServer(
            self,
            NavigateToPose,
            'dock_to_pose',
            execute_callback=self.dock_execute_callback,
            goal_callback=self.dock_goal_callback,
            cancel_callback=self.dock_cancel_callback,
            callback_group=self._dock_cb_group)

        # timer
        self.monitor_connection_timer = self.create_timer(1.0, self.monitor_connection_callback)

        # create thread to call toio API
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(
            target=self.start_loop, daemon=True)
        self.thread.start()
        asyncio.run_coroutine_threadsafe(
            self.connect_toio(),
            self.loop)
        asyncio.run_coroutine_threadsafe(
            self.motor_command_loop(),
            self.loop)
        asyncio.run_coroutine_threadsafe(
            self.led_command_loop(),
            self.loop)

    def start_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def cmd_vel_callback(self, msg: Twist) -> None:
        if not self.is_connected:
            return

        if self._docking:
            # Dropped rather than stored: applying a pre-dock command once the
            # dock finishes would drive the cube straight off the pose it just
            # reached. Nothing publishes cmd_vel during a dock under Open-RMF
            # anyway, since the fleet adapter has no Nav2 goal in flight then.
            self.get_logger().warn(
                'cmd_vel ignored while docking', throttle_duration_sec=1.0)
            return

        v = msg.linear.x
        omega = msg.angular.z

        # m/s
        v_l = v - omega * self.wheel_base / 2.0
        v_r = v + omega * self.wheel_base / 2.0

        # m/s -> RPM
        rpm_l = self.linear_speed_to_rpm(v_l)
        rpm_r = self.linear_speed_to_rpm(v_r)

        # clip
        rpm_l = max(min(rpm_l,  self.max_rpm), -self.max_rpm)
        rpm_r = max(min(rpm_r,  self.max_rpm), -self.max_rpm)
        self.get_logger().debug(f'rpm_l = {rpm_l}, rpm_r = {rpm_r}')

        # RPM -> toio motor_speed
        left_motor_speed = int((rpm_l / self.max_rpm) * self.max_input_speed)
        right_motor_speed = int((rpm_r / self.max_rpm) * self.max_input_speed)

        # tuple assignment is atomic, read by motor_command_loop()
        self._latest_cmd_vel = (left_motor_speed, right_motor_speed)
        self._latest_cmd_vel_stamp = time.monotonic()

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        if not self.is_connected:
            return

        if self._docking:
            # The cube's motor response carries no usable request id, so a
            # target motion started here would be indistinguishable from the
            # dock's own and could complete it early with the wrong result
            self.get_logger().warn('goal_pose ignored while docking')
            return

        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w
        x, y, angle = self.convert_ros_to_toio_coord(pos_x, pos_y, q_x, q_y, q_z, q_w)
        self.get_logger().info(f'goal_pose_callback() received, x = {x}, y = {y}, angle = {angle}')
        asyncio.run_coroutine_threadsafe(
            self.motor_control_target(x, y, angle),
            self.loop)

    def dock_goal_callback(self, goal_request) -> GoalResponse:
        """Accept a dock goal only while the cube is connected and idle."""
        if not self.is_connected:
            self.get_logger().warn('dock rejected: toio is not connected')
            return GoalResponse.REJECT
        with self._dock_lock:
            if self._docking:
                # Only one target motion can be tracked at a time: the cube's
                # response carries no usable request id (toio.py hardcodes it
                # to 0), so a second dock could complete the first one
                self.get_logger().warn('dock rejected: already docking')
                return GoalResponse.REJECT
            # Claimed here rather than in execute() so a second goal cannot
            # be accepted in the window before the first one starts running
            self._docking = True
            self._dock_done.clear()
            self._dock_response = None
        return GoalResponse.ACCEPT

    def dock_cancel_callback(self, goal_handle) -> CancelResponse:
        """Allow a dock to be cancelled (Open-RMF stopping mid-dock)."""
        return CancelResponse.ACCEPT

    def dock_execute_callback(self, goal_handle):
        """Drive the cube to the goal pose with its built-in target motion."""
        try:
            return self._run_dock(goal_handle)
        finally:
            with self._dock_lock:
                self._docking = False
            # motor_control() skips a command identical to the last one sent
            # inside motor_dedup_interval. The cmd_vel that resumes after a
            # dock is very often that same tuple, so without this the first
            # write back would be dropped and the cube would sit still.
            self._last_motor_cmd_time = 0.0

    def _run_dock(self, goal_handle):
        pose = goal_handle.request.pose.pose
        x, y, angle = self.convert_ros_to_toio_coord(
            pose.position.x, pose.position.y,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        self.get_logger().info(
            f'dock_to_pose received, x = {x}, y = {y}, angle = {angle}')
        self._dock_started_at = time.monotonic()
        asyncio.run_coroutine_threadsafe(
            self.motor_control_target(x, y, angle, timeout=self.dock_timeout),
            self.loop)

        result = NavigateToPose.Result()
        deadline = self._dock_started_at + self.dock_timeout + \
            self.DOCK_RESPONSE_GRACE
        while not self._dock_done.wait(self.DOCK_POLL_INTERVAL):
            if goal_handle.is_cancel_requested:
                self._stop_motor()
                goal_handle.canceled()
                result.error_msg = 'canceled'
                return result
            if not self.is_connected:
                goal_handle.abort()
                result.error_code = \
                    MotorResponseCode.ERROR_INVALID_CUBE_STATE.value
                result.error_msg = 'BLE disconnected while docking'
                return result
            if time.monotonic() > deadline:
                self._stop_motor()
                goal_handle.abort()
                result.error_code = MotorResponseCode.ERROR_TIMEOUT.value
                result.error_msg = 'no motor response before the deadline'
                return result
            goal_handle.publish_feedback(self._make_dock_feedback(pose))

        code = self._dock_response
        result.error_msg = code.name
        if code in (MotorResponseCode.SUCCESS,
                    MotorResponseCode.SUCCESS_WITH_OVERWRITE):
            if code is MotorResponseCode.SUCCESS_WITH_OVERWRITE:
                # Another motor command replaced the target motion, so the
                # cube may have stopped short. Still reported as success: the
                # caller already reached this pose with Nav2 before asking for
                # the dock, so a preempted refinement is not a task failure.
                # It does mean something wrote to the motor during a dock,
                # which the guards in cmd_vel_callback / motor_command_loop
                # are supposed to prevent - worth investigating if it appears.
                self.get_logger().warn(
                    'dock finished with SUCCESS_WITH_OVERWRITE: the target '
                    'motion was preempted by another motor command')
            # error_code stays at NONE (0), which is also SUCCESS's value
            goal_handle.succeed()
        else:
            self.get_logger().warn(f'dock aborted: {code.name}')
            result.error_code = code.value
            goal_handle.abort()
        return result

    def _stop_motor(self) -> None:
        """Stop the motors from the executor thread (cancel / timeout)."""
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.cube.api.motor.motor_control(
                    0, 0, duration_ms=self.MOTOR_DURATION_MS),
                self.loop)
            future.result(timeout=1.0)
        except Exception as e:
            self.get_logger().warn(f'failed to stop the motor: {e}')

    def _make_dock_feedback(self, goal_pose):
        feedback = NavigateToPose.Feedback()
        feedback.navigation_time = Duration(
            seconds=time.monotonic() - self._dock_started_at).to_msg()
        pose_xy = self._last_pose_xy
        if pose_xy is not None:
            feedback.current_pose.header.frame_id = 'map'
            feedback.current_pose.pose.position.x = pose_xy[0]
            feedback.current_pose.pose.position.y = pose_xy[1]
            feedback.distance_remaining = float(math.hypot(
                goal_pose.position.x - pose_xy[0],
                goal_pose.position.y - pose_xy[1]))
        return feedback

    def led_callback(self, msg: ColorRGBA) -> None:
        if not self.is_connected:
            return

        # ColorRGBA is 0.0-1.0 while the cube takes 0-255; alpha is unused.
        # tuple assignment is atomic, read by led_command_loop()
        self._pending_led = (self.to_led_value(msg.r),
                             self.to_led_value(msg.g),
                             self.to_led_value(msg.b),
                             None)

    def led_timed_callback(self, msg) -> None:
        """Set the indicator with a duration carried by the message."""
        if not self.is_connected:
            return

        self._pending_led = (self.to_led_value(msg.color.r),
                             self.to_led_value(msg.color.g),
                             self.to_led_value(msg.color.b),
                             self.to_duration_ms(msg.duration_ms))

    def led_pattern_callback(self, msg) -> None:
        """Hand a blink sequence to the cube to play on its own."""
        if not self.is_connected:
            return

        steps = self.checked_sequence(
            msg.steps, LedPattern.STEPS_MAX, 'led pattern')
        if steps is None:
            return

        # A pattern is an event and cannot be coalesced the way a single color
        # can, so it is throttled like a sound instead of latched
        now = time.monotonic()
        if now - self._last_led_pattern_time < self.led_write_interval:
            self.get_logger().debug('led pattern command throttled')
            return
        self._last_led_pattern_time = now

        # The cube is now running a pattern, so the latched single color no
        # longer describes the indicator; drop it or the next flush would
        # overwrite the pattern with a stale color
        self._pending_led = None
        self._last_led_cmd = None

        asyncio.run_coroutine_threadsafe(
            self.play_led_pattern(
                msg.repeat,
                [(self.to_led_value(s.color.r),
                  self.to_led_value(s.color.g),
                  self.to_led_value(s.color.b),
                  self.to_duration_ms(s.duration_ms)) for s in steps]),
            self.loop)

    def melody_callback(self, msg) -> None:
        """Hand a melody to the cube to play on its own."""
        if not self.is_connected:
            return

        notes = self.checked_sequence(
            msg.notes, Melody.NOTES_MAX, 'melody')
        if notes is None:
            return
        for note in notes:
            if note.note > MidiNoteMsg.NOTE_MAX:
                self.get_logger().warn(
                    f'melody rejected: note {note.note} is above '
                    f'{MidiNoteMsg.NOTE_MAX}')
                return

        # Shares the cube's sound channel with the effect topic, so it shares
        # the throttle too
        now = time.monotonic()
        if now - self._last_sound_time < self.sound_min_interval:
            self.get_logger().debug('melody command throttled')
            return
        self._last_sound_time = now

        asyncio.run_coroutine_threadsafe(
            self.play_melody(
                msg.repeat,
                [(self.to_duration_ms(n.duration_ms), n.note, n.volume)
                 for n in notes]),
            self.loop)

    def checked_sequence(self, items, limit, what):
        """Return items if the cube can take them, otherwise None."""
        if len(items) == 0:
            self.get_logger().warn(f'{what} rejected: no entries')
            return None
        if len(items) > limit:
            # Truncating would play a pattern nobody asked for, which is
            # harder to notice than nothing happening
            self.get_logger().warn(
                f'{what} rejected: {len(items)} entries exceeds the '
                f'{limit} the cube accepts')
            return None
        return items

    @staticmethod
    def to_duration_ms(duration_ms: int) -> int:
        # The cube counts in 10ms units and stops at 2550ms; toio.py clips
        # silently, so clip here too and keep the two consistent
        return min(int(duration_ms), Led.DURATION_MAX_MS)

    def sound_callback(self, msg: UInt8) -> None:
        if not self.is_connected:
            return

        try:
            sound_id = SoundId(msg.data)
        except ValueError:
            self.get_logger().warn(
                f'unknown sound effect id {msg.data}, '
                f'expected 0-{int(max(SoundId))}')
            return

        now = time.monotonic()
        if now - self._last_sound_time < self.sound_min_interval:
            self.get_logger().debug('sound command throttled')
            return
        self._last_sound_time = now

        asyncio.run_coroutine_threadsafe(
            self.play_sound(sound_id),
            self.loop)

    def send_interval_error(self, name: str, value: float):
        """Return why `value` is not usable for `name`, or None if it is."""
        minimum, maximum = self.SEND_INTERVAL_BOUNDS[name]
        # NaN compares false against everything, so a plain range check would
        # let it through
        if not math.isfinite(value):
            return f'{name} must be a finite number'
        if value < minimum:
            return f'{name} must be at least {minimum}s'
        if maximum is not None and value >= maximum:
            return (f'{name} must be below {maximum}s, the motor command '
                    f'auto-stop')
        return None

    def clamp_send_interval(self, name: str, value: float) -> float:
        # Startup only: a bad value in a params file clamps with a warning
        # rather than refusing to start. Runtime changes are rejected instead
        # (see on_set_parameters), which keeps the reported value honest.
        error = self.send_interval_error(name, value)
        if error is None:
            return value
        minimum, maximum = self.SEND_INTERVAL_BOUNDS[name]
        # Half the auto-stop leaves room for one missed resend
        fallback = minimum if (
            not math.isfinite(value) or value < minimum) else maximum / 2.0
        self.get_logger().warn(f'{error} (got {value}); using {fallback}s')
        return fallback

    def on_set_parameters(self, params):
        # Rejected rather than clamped: a clamp would leave the parameter
        # reporting the value that was asked for while the node runs at a
        # different one, so `ros2 param get` would lie about the send rate
        for param in params:
            if param.name not in self.SEND_INTERVAL_BOUNDS:
                continue
            error = self.send_interval_error(param.name, param.value)
            if error is not None:
                return SetParametersResult(successful=False, reason=error)
        for param in params:
            if param.name in self.SEND_INTERVAL_BOUNDS:
                setattr(self, param.name, param.value)
        return SetParametersResult(successful=True)

    @staticmethod
    def to_led_value(value: float) -> int:
        # NaN / inf from a malformed message would raise on int() and kill the
        # subscriber callback, so they are treated as "off" instead
        if not math.isfinite(value):
            return 0
        return max(min(int(round(value * 255.0)), 255), 0)

    def linear_speed_to_rpm(self, v_lin: float) -> float:
        return (v_lin / (2.0 * math.pi * self.wheel_radius)) * 60.0

    def _on_id_notification(self, payload: bytearray) -> None:
        """Handle Position ID notification (called on the asyncio loop thread)."""
        info = IdInformation.is_my_data(payload)
        # PositionIdMissed (cube left the mat) and StandardId are not published
        if not isinstance(info, PositionId):
            return

        # convert ROS 2 coordinate
        pos_x, pos_y, q_x, q_y, q_z, q_w = self.convert_toio_to_ros_coord(
            info.center.point.x, info.center.point.y, info.center.angle)

        # kept for the dock action feedback, which runs on another thread
        self._last_pose_xy = (pos_x, pos_y)

        # publish PoseStamped
        toio_pose_stamped_msg = self.make_pose_stamped_msg(pos_x, pos_y, q_x, q_y, q_z, q_w)
        self.toio_pose_pub.publish(toio_pose_stamped_msg)

        # send the transformation
        toio_transform = self.make_toio_transform(pos_x, pos_y, q_x, q_y, q_z, q_w)
        self.tf_broadcaster.sendTransform(toio_transform)

    def convert_toio_to_ros_coord(self, x, y, angle):
        pos_x = float(x - self.field_min_x) * self.scale_x
        pos_y = -float(y - self.field_min_y) * self.scale_y
        yaw_deg = 360.0 - float(angle)  # deg
        yaw_rad = math.radians(yaw_deg)
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw_rad)
        return pos_x, pos_y, q_x, q_y, q_z, q_w

    def convert_ros_to_toio_coord(self, pos_x, pos_y, q_x, q_y, q_z, q_w):
        x = int((float(pos_x) / self.scale_x) + self.field_min_x)
        y = int(-(pos_y / self.scale_y) + self.field_min_y)
        # clamp to the Position ID range of the mat with a margin
        # (out-of-range values are not validated by toio.py and break BLE
        # packing; a goal at the exact boundary makes the cube lose the
        # Position ID at the edge and abort mid-drive, see issue #9)
        clamped_x = max(min(x, int(self.field_max_x) - self.goal_boundary_margin),
                        int(self.field_min_x) + self.goal_boundary_margin)
        clamped_y = max(min(y, int(self.field_max_y) - self.goal_boundary_margin),
                        int(self.field_min_y) + self.goal_boundary_margin)
        if (clamped_x, clamped_y) != (x, y):
            self.get_logger().warn(
                f'goal position ({x}, {y}) is outside the safe mat area, '
                f'clamped to ({clamped_x}, {clamped_y})')
        _, _, yaw_rad = euler_from_quaternion([q_x, q_y, q_z, q_w])
        yaw_deg = math.degrees(yaw_rad)
        angle = int(360.0 - yaw_deg) % 360
        return clamped_x, clamped_y, angle

    def make_pose_stamped_msg(self, x, y, q_x, q_y, q_z, q_w):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = 'map'
        pose_stamped_msg.pose.position.x = x
        pose_stamped_msg.pose.position.y = y
        # z is the geometric center of the cube body so the RViz Pose arrow
        # does not sink into the ground grid; the map->center TF stays at z=0
        # because the URDF 'center' frame is at the ground-contact plane
        pose_stamped_msg.pose.position.z = self.cube_height / 2.0
        pose_stamped_msg.pose.orientation.x = q_x
        pose_stamped_msg.pose.orientation.y = q_y
        pose_stamped_msg.pose.orientation.z = q_z
        pose_stamped_msg.pose.orientation.w = q_w
        return pose_stamped_msg

    def make_toio_transform(self, x, y, q_x, q_y, q_z, q_w):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = self.frame_prefix + 'center'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = q_x
        transform.transform.rotation.y = q_y
        transform.transform.rotation.z = q_z
        transform.transform.rotation.w = q_w
        return transform

    def _on_battery_notification(self, payload: bytearray) -> None:
        """Handle battery notification (called on the asyncio loop thread)."""
        info = Battery.is_my_data(payload)
        if info is None:
            return

        # battery_level is a percentage notified in 10% steps (0-100):
        # https://toio.github.io/toio-spec/docs/ble_battery
        battery_state_msg = BatteryState()
        battery_state_msg.header.stamp = self.get_clock().now().to_msg()
        battery_state_msg.percentage = float(info.battery_level) / 100.0
        battery_state_msg.present = True
        battery_state_msg.power_supply_status = \
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_state_msg.power_supply_technology = \
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        self.toio_battery_state_pub.publish(battery_state_msg)
        self.get_logger().debug(f'battery_level = {info.battery_level}')

    def _on_motor_notification(self, payload: bytearray) -> None:
        """Handle motor response notification (called on the asyncio loop thread)."""
        info = Motor.is_my_data(payload)
        # only motor_control_target results are reported (issue #9);
        # motor_control() used for cmd_vel does not send responses
        if not isinstance(info, ResponseMotorControlTarget):
            return

        # Matched by state rather than by request id: toio.py hardcodes the
        # request id of a target motion to 0, so every response carries the
        # same one. It is sound here because a dock is the only target motion
        # that can be in flight (goal_pose is refused while docking, a second
        # dock goal is rejected, and the cmd_vel path uses motor_control(),
        # which sends no response at all).
        if self._docking:
            self._dock_response = info.response_code
            self._dock_done.set()

        if info.response_code in (MotorResponseCode.SUCCESS,
                                  MotorResponseCode.SUCCESS_WITH_OVERWRITE):
            self.get_logger().info(f'goal result: {info.response_code.name}')
        else:
            self.get_logger().warn(f'goal aborted: {info.response_code.name}')

    def monitor_connection_callback(self) -> None:
        if not self.is_connected:
            return

        if not self.cube.is_connect():
            self._schedule_reconnect()

    def _schedule_reconnect(self) -> None:
        """Flip to disconnected and start reconnection exactly once."""
        with self._reconnect_lock:
            if not self.is_connected:
                return
            self.is_connected = False
        self.get_logger().error('toio is disconnected. reconnecting...')
        asyncio.run_coroutine_threadsafe(
            self.connect_toio(),
            self.loop)

    async def scan_toio(self):
        """Scan and select the cube to connect to (returns a CubeInfo or None)."""
        if self.cube_id:
            found = await BLEScanner.scan_with_id(cube_id={self.cube_id})
        elif self.cube_address:
            found = await BLEScanner.scan_with_address(address={self.cube_address})
        else:
            # Auto-connect mode: report the cubes nearby as cube_id
            # candidates (see AUTO_CONNECT_SCAN_NUM).
            found = await BLEScanner.scan(AUTO_CONNECT_SCAN_NUM)
            for cube_info in found:
                self.get_logger().info(
                    f'found cube: {cube_info.name} ({cube_info.device.address})')
            if len(found) > 1:
                self.get_logger().warn(
                    f'{len(found)} cubes found; connecting to the nearest one. '
                    'set the cube_id parameter to select a specific cube')
        if not found:
            return None
        return found[0]

    # async function
    async def connect_toio(self) -> None:
        while rclpy.ok():
            try:
                cube_info = await self.scan_toio()
                if cube_info is None:
                    raise RuntimeError('no toio cube found by BLE scan')
                self.cube = ToioCoreCube(interface=cube_info.interface, name=cube_info.name)
                # connect() may wait forever on a silent BLE failure
                await asyncio.wait_for(self.cube.connect(), timeout=self.connect_timeout)
                # cube.api is created inside connect(), so register handlers here;
                # reconnection is covered because a fresh cube is built each attempt
                await self.cube.api.id_information.register_notification_handler(
                    self._on_id_notification)
                await self.cube.api.battery.register_notification_handler(
                    self._on_battery_notification)
                await self.cube.api.motor.register_notification_handler(
                    self._on_motor_notification)
                # the indicator state after a reconnection is not guaranteed to
                # match the last requested color, so let led_command_loop()
                # write it again (an identical write is harmless)
                self._last_led_cmd = None
                self.is_connected = True
                self.get_logger().info(
                    f'toio is connected: {cube_info.name} ({cube_info.device.address})')
                return
            except Exception as e:
                self.get_logger().error(
                    f'toio connection failed: {e}. retrying in {self.reconnect_interval}s...')
                await asyncio.sleep(self.reconnect_interval)

    def pending_motor_cmd(self):
        """Return the command motor_command_loop() should send, or None."""
        if not self.is_connected or self._latest_cmd_vel is None:
            return None
        # A dock owns the motor until it finishes. Without this the stop
        # below would land half a second into every dock and overwrite the
        # target motion, leaving the cube short of the dock pose (the cube
        # would report SUCCESS_WITH_OVERWRITE).
        if self._docking:
            return None
        # Send stop when cmd_vel goes silent, keeping the auto-stop
        # semantics of duration_ms=500
        if time.monotonic() - self._latest_cmd_vel_stamp > self.cmd_vel_timeout:
            return (0, 0)
        return self._latest_cmd_vel

    async def motor_command_loop(self) -> None:
        # Resident sender: reads the latest cmd_vel at a fixed rate so BLE
        # writes are bounded to 20Hz no matter how fast cmd_vel is published
        while rclpy.ok():
            await asyncio.sleep(0.05)  # 20Hz
            cmd = self.pending_motor_cmd()
            if cmd is None:
                continue
            try:
                await self.motor_control(*cmd)
            except Exception as e:
                self.get_logger().debug(f'motor command failed: {e}')
                # React to disconnection as soon as a send fails instead of
                # waiting up to 1s for the monitor timer (see issue #10)
                if not self.cube.is_connect():
                    self._schedule_reconnect()

    async def motor_control(self, left_motor_speed, right_motor_speed) -> None:
        # Command deduplication with time-based resend:
        # Skip if same command AND sent less than dedup interval ago.
        # A stop command is never resent (the motor is already stopped and
        # duration_ms guarantees auto-stop), avoiding idle BLE writes.
        cmd = (left_motor_speed, right_motor_speed)
        now = time.monotonic()
        if cmd != self._last_motor_cmd or \
                (cmd != (0, 0) and (now - self._last_motor_cmd_time) >= self.motor_dedup_interval):
            await self.cube.api.motor.motor_control(
                left_motor_speed, right_motor_speed,
                duration_ms=self.MOTOR_DURATION_MS)
            self._last_motor_cmd = cmd
            self._last_motor_cmd_time = now

    async def motor_control_target(self, x, y, angle, timeout=None) -> None:
        self.get_logger().debug(f'motor_control_target(): x = {x}, y={y}, angle = {angle}')
        await self.cube.api.motor.motor_control_target(
            timeout=self.goal_timeout if timeout is None else timeout,
            movement_type=MovementType.Linear,
            speed=Speed(
                max=self.goal_max_speed,
                speed_change_type=SpeedChangeType.AccelerationAndDeceleration),
            target=TargetPosition(
                cube_location=CubeLocation(point=Point(x=x, y=y), angle=angle),
                rotation_option=RotationOption.AbsoluteOptimal,
            ),
        )

    async def led_command_loop(self) -> None:
        # Resident sender: writes the newest color at a bounded rate so BLE
        # writes never queue up behind a fast publisher
        while rclpy.ok():
            await asyncio.sleep(self.led_write_interval)
            await self.flush_led()

    async def flush_led(self) -> None:
        if not self.is_connected:
            return

        cmd = self._pending_led
        # the indicator holds its state, so an unchanged color needs no write
        if cmd is None or cmd == self._last_led_cmd:
            return

        try:
            await self.set_indicator(*cmd)
        except Exception as e:
            # keep _last_led_cmd untouched so the color is retried next tick
            self.get_logger().warn(f'led command failed: {e}')
            return
        self._last_led_cmd = cmd

    async def set_indicator(self, r, g, b, duration_ms=None) -> None:
        # duration_ms None means "use the node-wide default", which is what
        # the plain ColorRGBA topic has no way to express
        if duration_ms is None:
            duration_ms = self.led_duration_ms
        if (r, g, b) == (0, 0, 0):
            await self.cube.api.indicator.turn_off_all()
        else:
            await self.cube.api.indicator.turn_on(
                IndicatorParam(
                    duration_ms=duration_ms,
                    color=Color(r=r, g=g, b=b)))

    async def play_led_pattern(self, repeat, steps) -> None:
        try:
            await self.cube.api.indicator.repeated_turn_on(
                repeat,
                [IndicatorParam(duration_ms=d, color=Color(r=r, g=g, b=b))
                 for r, g, b, d in steps])
        except Exception as e:
            self.get_logger().warn(f'led pattern command failed: {e}')

    async def play_melody(self, repeat, notes) -> None:
        try:
            await self.cube.api.sound.play_midi(
                repeat,
                [MidiNote(duration_ms=d, note=n, volume=v)
                 for d, n, v in notes])
        except Exception as e:
            self.get_logger().warn(f'melody command failed: {e}')

    async def play_sound(self, sound_id: SoundId) -> None:
        # Nobody awaits the future returned by run_coroutine_threadsafe, so an
        # escaping exception would only show up as a stray 'Future exception
        # was never retrieved' message with no context
        try:
            await self.cube.api.sound.play_sound_effect(sound_id, self.sound_volume)
        except Exception as e:
            self.get_logger().warn(f'sound command failed: {e}')

    async def shutdown_toio(self) -> None:
        if self.is_connected:
            self.is_connected = False  # stop motor_command_loop / watchdog sends
            # Unregister before the cleanup writes below: a SIGINT shuts the
            # rclpy context down before destroy_node() runs (see main()), so a
            # notification arriving while those BLE round-trips are in flight
            # would publish on an invalid context and raise.
            # Passing None unregisters all handlers.
            await self.cube.api.id_information.unregister_notification_handler(None)
            await self.cube.api.battery.unregister_notification_handler(None)
            await self.cube.api.motor.unregister_notification_handler(None)
            await self.cube.api.motor.motor_control(0, 0)
            # do not leave the cube lit or buzzing after the node exits, but
            # keep disconnecting when the cube no longer answers
            try:
                await self.cube.api.indicator.turn_off_all()
                await self.cube.api.sound.stop()
            except Exception as e:
                self.get_logger().warn(f'failed to turn off led/sound: {e}')
            await self.cube.disconnect()

    async def cancel_pending_tasks(self) -> None:
        """Cancel every coroutine still running on the loop and await them."""
        # Resident loops only leave on their own when the rclpy context is
        # already down (a SIGINT does that before destroy_node() runs), so
        # stopping the loop without cancelling leaves them pending and asyncio
        # reports 'Task was destroyed but it is pending!' for each one.
        # all_tasks() also covers the reconnection and the one-shot goal_pose /
        # sound coroutines, which no bookkeeping list would keep up with.
        current = asyncio.current_task()
        pending = [task for task in asyncio.all_tasks() if task is not current]
        for task in pending:
            task.cancel()
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)

    def destroy_node(self):
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.shutdown_toio(),
                self.loop)
            future.result(timeout=5.0)
        except Exception as e:
            self.get_logger().warn(f'toio shutdown failed: {e}')
        # after shutdown_toio(), so that the motor stop / indicator off /
        # disconnect are not cancelled along with the resident loops
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.cancel_pending_tasks(),
                self.loop)
            future.result(timeout=2.0)
        except Exception as e:
            self.get_logger().warn(f'failed to cancel background tasks: {e}')
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ToioNode()

    # MultiThreadedExecutor because the dock action blocks its executor
    # thread for the whole target motion while the cube drives. A single
    # threaded executor would stop serving cmd_vel and the connection
    # monitor for that whole time, and could never deliver the cancel
    # request that ends the dock early. Only the dock server uses a
    # reentrant callback group; every other callback keeps the default
    # mutually exclusive one and so stays serialized as before.
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGINT from `ros2 launch` shuts down the context before this
        # `finally` runs; `try_shutdown()` is a no-op in that case.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

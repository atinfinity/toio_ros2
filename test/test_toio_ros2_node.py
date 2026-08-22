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
import struct
import threading
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import CancelResponse, GoalResponse
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from std_msgs.msg import Bool, ColorRGBA, UInt8
from tf_transformations import euler_from_quaternion
from toio import MotorResponseCode, SoundId
from toio.device_interface import CubeInfo

from toio_msgs.msg import Led, LedPattern, Melody, MotionDetection
from toio_msgs.msg import MidiNote as MidiNoteMsg

from toio_ros2.toio_ros2_node import AUTO_CONNECT_SCAN_NUM, ToioNode

# captured before the node fixture replaces it with a no-op
REAL_CONNECT_TOIO = ToioNode.connect_toio

SEND_INTERVALS = ['led_write_interval', 'sound_min_interval',
                  'motor_dedup_interval']


@pytest.fixture
def node(monkeypatch):
    async def _noop(self):
        return None

    # no real BLE scan / resident motor sender in unit tests
    monkeypatch.setattr(ToioNode, 'connect_toio', _noop)
    monkeypatch.setattr(ToioNode, 'motor_command_loop', _noop)
    rclpy.init()
    node = ToioNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def node_no_goal_pose(monkeypatch):
    """Build a node the way Open-RMF starts it, with goal_pose off (ADR-5)."""
    async def _noop(self):
        return None

    monkeypatch.setattr(ToioNode, 'connect_toio', _noop)
    monkeypatch.setattr(ToioNode, 'motor_command_loop', _noop)
    rclpy.init()
    node = ToioNode(parameter_overrides=[
        Parameter('enable_goal_pose_motion', Parameter.Type.BOOL, False)])
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def scheduled(monkeypatch):
    """Capture the coroutines a callback hands to the asyncio loop thread."""
    coros = []

    def fake_run_coroutine_threadsafe(coro, loop):
        coros.append(coro)
        return MagicMock()

    monkeypatch.setattr(
        'toio_ros2.toio_ros2_node.asyncio.run_coroutine_threadsafe',
        fake_run_coroutine_threadsafe)
    yield coros
    # the patch must be gone before the node fixture calls destroy_node(),
    # otherwise its shutdown coroutine is captured here and never awaited
    monkeypatch.undo()
    for coro in coros:
        coro.close()


def make_position_id_payload(x, y, angle):
    # https://toio.github.io/toio-spec/en/docs/ble_id#position-id
    return bytearray(struct.pack('<BHHHHHH', 0x01, x, y, angle, x, y, angle))


def make_position_id_missed_payload():
    # https://toio.github.io/toio-spec/en/docs/ble_id#position-id-missed
    return bytearray(struct.pack('<B', 0x03))


def make_motion_payload(horizontal=1, collision=0, double_tap=0, posture=1, shake=0):
    # https://toio.github.io/toio-spec/en/docs/ble_sensor#obtaining-motion-detection-information
    return bytearray(struct.pack(
        '<BBBBBB', 0x01, horizontal, collision, double_tap, posture, shake))


def make_connectable_cube():
    """Build a cube mock that connect_toio() can go through without BLE."""
    cube = MagicMock()
    cube.connect = AsyncMock()
    for api in (cube.api.id_information, cube.api.battery, cube.api.motor,
                cube.api.sensor, cube.api.button):
        api.register_notification_handler = AsyncMock()
    cube.api.button.read = AsyncMock(return_value=None)
    cube.api.configuration._write = AsyncMock()
    cube.api.configuration.set_horizontal_detection_threshold = AsyncMock()
    cube.api.configuration.set_motor_speed_information_acquisition = AsyncMock()
    cube.api.configuration.set_posture_angle_detection = AsyncMock()
    cube.api.sensor.read = AsyncMock(return_value=None)
    cube.api.sensor.request_motion_information = AsyncMock()
    return cube


def test_convert_toio_to_ros_coord(node):
    pos_x, pos_y, q_x, q_y, q_z, q_w = node.convert_toio_to_ros_coord(250, 250, 90)
    assert pos_x == pytest.approx((250 - node.field_min_x) * node.scale_x)
    assert pos_y == pytest.approx(-(250 - node.field_min_y) * node.scale_y)
    _, _, yaw = euler_from_quaternion([q_x, q_y, q_z, q_w])
    # toio angle 90 deg -> ROS yaw 360 - 90 = 270 deg, wrapped to -90 deg
    assert math.degrees(yaw) == pytest.approx(-90.0)


def test_convert_round_trip(node):
    pos_x, pos_y, q_x, q_y, q_z, q_w = node.convert_toio_to_ros_coord(250, 250, 90)
    x, y, angle = node.convert_ros_to_toio_coord(pos_x, pos_y, q_x, q_y, q_z, q_w)
    assert abs(x - 250) <= 1
    assert abs(y - 250) <= 1
    assert abs((angle - 90 + 180) % 360 - 180) <= 1


def test_goal_inside_mat_is_not_clamped(node):
    pos_x, pos_y, q_x, q_y, q_z, q_w = node.convert_toio_to_ros_coord(250, 250, 0)
    x, y, _ = node.convert_ros_to_toio_coord(pos_x, pos_y, q_x, q_y, q_z, q_w)
    assert abs(x - 250) <= 1
    assert abs(y - 250) <= 1


def test_goal_clamped_inside_boundary_margin(node):
    # beyond the (field_max_x, field_min_y) corner of the A4 mat
    x, y, _ = node.convert_ros_to_toio_coord(0.4, 0.05, 0.0, 0.0, 0.0, 1.0)
    assert x == int(node.field_max_x) - node.goal_boundary_margin
    assert y == int(node.field_min_y) + node.goal_boundary_margin


def test_goal_clamped_at_opposite_corner(node):
    # beyond the (field_min_x, field_max_y) corner of the A4 mat
    x, y, _ = node.convert_ros_to_toio_coord(-0.1, -0.3, 0.0, 0.0, 0.0, 1.0)
    assert x == int(node.field_min_x) + node.goal_boundary_margin
    assert y == int(node.field_max_y) - node.goal_boundary_margin


def test_linear_speed_to_rpm(node):
    # one wheel revolution per second
    v = 2.0 * math.pi * node.wheel_radius
    assert node.linear_speed_to_rpm(v) == pytest.approx(60.0)


def test_cmd_vel_ignored_when_disconnected(node):
    msg = Twist()
    msg.linear.x = 0.1
    node.cmd_vel_callback(msg)
    assert node._latest_cmd_vel is None


def test_cmd_vel_clipped_to_max_input_speed(node):
    node.is_connected = True
    msg = Twist()
    msg.linear.x = 10.0
    node.cmd_vel_callback(msg)
    assert node._latest_cmd_vel == (int(node.max_input_speed), int(node.max_input_speed))


def test_cmd_vel_rotation_is_symmetric(node):
    node.is_connected = True
    msg = Twist()
    msg.angular.z = 2.0
    node.cmd_vel_callback(msg)
    left, right = node._latest_cmd_vel
    assert left == -right
    assert right > 0


def test_goal_pose_ignored_when_disconnected(node, scheduled):
    node.goal_pose_callback(PoseStamped())
    assert not scheduled


def test_motor_control_dedup(node):
    node.cube = MagicMock()
    node.cube.api.motor.motor_control = AsyncMock()

    asyncio.run(node.motor_control(50, 50))
    asyncio.run(node.motor_control(50, 50))
    assert node.cube.api.motor.motor_control.await_count == 1

    # the same non-stop command is resent after the dedup interval
    node._last_motor_cmd_time -= node.motor_dedup_interval
    asyncio.run(node.motor_control(50, 50))
    assert node.cube.api.motor.motor_control.await_count == 2


def test_motor_control_stop_not_resent(node):
    node.cube = MagicMock()
    node.cube.api.motor.motor_control = AsyncMock()

    # already stopped at startup: no idle BLE write
    asyncio.run(node.motor_control(0, 0))
    assert node.cube.api.motor.motor_control.await_count == 0

    asyncio.run(node.motor_control(50, 50))
    asyncio.run(node.motor_control(0, 0))
    asyncio.run(node.motor_control(0, 0))
    # one drive command and one stop; the second stop is deduplicated
    assert node.cube.api.motor.motor_control.await_count == 2


def test_led_ignored_when_disconnected(node):
    node.led_callback(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
    assert node._pending_led is None


def test_sound_ignored_when_disconnected(node, scheduled):
    node.sound_callback(UInt8(data=int(SoundId.Get1)))
    assert not scheduled


def test_led_scales_color_to_cube_range(node):
    node.is_connected = True
    node.cube = MagicMock()
    node.cube.api.indicator.turn_on = AsyncMock()

    node.led_callback(ColorRGBA(r=1.0, g=0.0, b=0.5, a=1.0))
    asyncio.run(node.flush_led())

    param = node.cube.api.indicator.turn_on.call_args[0][0]
    assert param.color.flatten() == (255, 0, 128)
    assert param.duration_ms == node.led_duration_ms


def test_led_clips_out_of_range_color(node):
    node.is_connected = True

    # NaN would raise on int() inside the callback if it were not filtered
    node.led_callback(ColorRGBA(r=1.5, g=-0.2, b=float('nan'), a=1.0))

    # the fourth element is the per-command duration, unset on this topic
    assert node._pending_led == (255, 0, 0, None)


def test_led_all_zero_turns_the_indicator_off(node):
    node.is_connected = True
    node.cube = MagicMock()
    node.cube.api.indicator.turn_on = AsyncMock()
    node.cube.api.indicator.turn_off_all = AsyncMock()

    node.led_callback(ColorRGBA())
    asyncio.run(node.flush_led())

    node.cube.api.indicator.turn_off_all.assert_awaited_once()
    node.cube.api.indicator.turn_on.assert_not_awaited()


def test_led_keeps_only_the_newest_color(node):
    node.is_connected = True
    node.cube = MagicMock()
    node.cube.api.indicator.turn_on = AsyncMock()

    # a burst from a fast publisher must not queue up one BLE write each
    node.led_callback(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
    node.led_callback(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
    asyncio.run(node.flush_led())

    param = node.cube.api.indicator.turn_on.call_args[0][0]
    assert param.color.flatten() == (0, 255, 0)
    assert node.cube.api.indicator.turn_on.await_count == 1


def test_led_unchanged_color_is_not_rewritten(node):
    node.is_connected = True
    node.cube = MagicMock()
    node.cube.api.indicator.turn_on = AsyncMock()

    node.led_callback(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
    asyncio.run(node.flush_led())
    asyncio.run(node.flush_led())

    # the indicator holds its state, so an idle BLE write is pointless
    assert node.cube.api.indicator.turn_on.await_count == 1


def test_led_is_retried_after_a_failed_write(node):
    node.is_connected = True
    node.cube = MagicMock()
    node.cube.api.indicator.turn_on = AsyncMock(side_effect=RuntimeError('ble'))

    node.led_callback(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
    asyncio.run(node.flush_led())
    node.cube.api.indicator.turn_on.side_effect = None
    asyncio.run(node.flush_led())

    assert node.cube.api.indicator.turn_on.await_count == 2
    assert node._last_led_cmd == (255, 0, 0, None)


def test_sound_plays_the_requested_effect(node, scheduled):
    node.is_connected = True
    node.cube = MagicMock()
    node.cube.api.sound.play_sound_effect = AsyncMock()

    node.sound_callback(UInt8(data=int(SoundId.Get1)))
    asyncio.run(scheduled[0])

    node.cube.api.sound.play_sound_effect.assert_awaited_once_with(
        SoundId.Get1, node.sound_volume)


def test_sound_unknown_id_is_ignored(node, scheduled):
    node.is_connected = True

    # https://toio.github.io/toio-spec/docs/ble_sound#sound-effect-id
    node.sound_callback(UInt8(data=int(max(SoundId)) + 1))

    assert not scheduled


def test_sound_is_throttled(node, scheduled):
    node.is_connected = True

    # a sound is an event and cannot be coalesced, so the burst is dropped
    node.sound_callback(UInt8(data=int(SoundId.Get1)))
    node.sound_callback(UInt8(data=int(SoundId.Get2)))
    assert len(scheduled) == 1

    # the next command is accepted once the interval has passed
    node._last_sound_time -= node.sound_min_interval
    node.sound_callback(UInt8(data=int(SoundId.Get2)))
    assert len(scheduled) == 2


def test_sound_command_failure_does_not_raise(node):
    node.cube = MagicMock()
    node.cube.api.sound.play_sound_effect = AsyncMock(side_effect=RuntimeError('ble'))

    # nobody awaits this coroutine, so it must swallow and log instead
    asyncio.run(node.play_sound(SoundId.Get1))


def make_connected_cube_mock():
    cube = MagicMock()
    cube.api.motor.motor_control = AsyncMock()
    cube.api.indicator.turn_off_all = AsyncMock()
    cube.api.sound.stop = AsyncMock()
    cube.api.id_information.unregister_notification_handler = AsyncMock()
    cube.api.battery.unregister_notification_handler = AsyncMock()
    cube.api.motor.unregister_notification_handler = AsyncMock()
    cube.api.sensor.unregister_notification_handler = AsyncMock()
    cube.api.button.unregister_notification_handler = AsyncMock()
    cube.disconnect = AsyncMock()
    return cube


def test_shutdown_turns_off_led_and_sound(node):
    node.is_connected = True
    node.cube = make_connected_cube_mock()

    asyncio.run(node.shutdown_toio())

    node.cube.api.indicator.turn_off_all.assert_awaited_once()
    node.cube.api.sound.stop.assert_awaited_once()
    node.cube.disconnect.assert_awaited_once()


def test_shutdown_disconnects_even_if_turning_off_fails(node):
    node.is_connected = True
    node.cube = make_connected_cube_mock()
    node.cube.api.indicator.turn_off_all = AsyncMock(side_effect=RuntimeError('ble'))

    asyncio.run(node.shutdown_toio())

    node.cube.disconnect.assert_awaited_once()


def test_cancel_pending_tasks_cancels_the_resident_loops(node):
    async def resident():
        while True:
            await asyncio.sleep(3600)

    async def scenario():
        tasks = [asyncio.ensure_future(resident()) for _ in range(3)]
        await asyncio.sleep(0)  # let them reach the first await
        await node.cancel_pending_tasks()
        return tasks

    tasks = asyncio.run(scenario())

    assert all(task.cancelled() for task in tasks)


def test_cancel_pending_tasks_does_not_cancel_itself(node):
    async def scenario():
        await node.cancel_pending_tasks()
        # reached only when cancel_pending_tasks() left its own task alone
        return True

    assert asyncio.run(scenario()) is True


def test_destroy_node_cancels_pending_tasks_after_shutdown(node, monkeypatch):
    calls = []

    def fake_run_coroutine_threadsafe(coro, loop):
        calls.append(coro.__name__)
        coro.close()
        return MagicMock()

    monkeypatch.setattr(
        'toio_ros2.toio_ros2_node.asyncio.run_coroutine_threadsafe',
        fake_run_coroutine_threadsafe)
    # keep the real loop running: the node fixture destroys the node again on
    # teardown, and a stopped loop would make that call wait out its timeouts
    monkeypatch.setattr(node, 'loop', MagicMock())
    monkeypatch.setattr(node, 'thread', MagicMock())

    node.destroy_node()
    monkeypatch.undo()

    # the cube cleanup must not be cancelled along with the resident loops
    assert calls == ['shutdown_toio', 'cancel_pending_tasks']


def test_position_id_notification_publishes_pose(node):
    node.toio_pose_pub = MagicMock()
    node.tf_broadcaster = MagicMock()
    node.publish_odom = False  # legacy tree: map -> center straight from the Position ID

    node._on_id_notification(make_position_id_payload(250, 250, 0))

    node.toio_pose_pub.publish.assert_called_once()
    msg = node.toio_pose_pub.publish.call_args[0][0]
    assert msg.header.frame_id == 'map'
    assert msg.pose.position.x == pytest.approx((250 - node.field_min_x) * node.scale_x)
    assert msg.pose.position.y == pytest.approx(-(250 - node.field_min_y) * node.scale_y)
    assert msg.pose.position.z == pytest.approx(node.cube_height / 2.0)
    node.tf_broadcaster.sendTransform.assert_called_once()
    transform = node.tf_broadcaster.sendTransform.call_args[0][0]
    assert transform.header.frame_id == 'map'
    assert transform.child_frame_id == 'center'
    assert transform.transform.translation.z == 0.0


def test_position_id_missed_publishes_state_without_pose(node):
    node.toio_pose_pub = MagicMock()
    node.tf_broadcaster = MagicMock()
    node.position_id_missed_pub = MagicMock()

    node._on_id_notification(make_position_id_missed_payload())

    node.toio_pose_pub.publish.assert_not_called()
    node.tf_broadcaster.sendTransform.assert_not_called()
    node.position_id_missed_pub.publish.assert_called_once_with(Bool(data=True))
    assert node._position_id_missed


def test_position_id_missed_is_published_on_change_only(node):
    node.toio_pose_pub = MagicMock()
    node.tf_broadcaster = MagicMock()
    node.position_id_missed_pub = MagicMock()

    # on the mat at startup: a Position ID does not announce anything
    node._on_id_notification(make_position_id_payload(250, 250, 0))
    node.position_id_missed_pub.publish.assert_not_called()

    node._on_id_notification(make_position_id_missed_payload())
    node._on_id_notification(make_position_id_missed_payload())
    assert node.position_id_missed_pub.publish.call_count == 1

    # back on the mat: recovery is announced and the pose flows again
    node._on_id_notification(make_position_id_payload(250, 250, 0))
    assert node.position_id_missed_pub.publish.call_count == 2
    assert node.position_id_missed_pub.publish.call_args[0][0] == Bool(data=False)
    assert not node._position_id_missed
    assert node.toio_pose_pub.publish.call_count == 2


def test_position_id_missed_topic_is_latched(node):
    qos = node.position_id_missed_pub.qos_profile
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_position_id_missed_stops_the_motor(node):
    node.is_connected = True
    msg = Twist()
    msg.linear.x = 0.1
    node.cmd_vel_callback(msg)
    drive = node._latest_cmd_vel
    assert drive != (0, 0)

    node._on_id_notification(make_position_id_missed_payload())
    assert node.pending_motor_cmd() == (0, 0)
    # the newest cmd_vel is kept, not dropped, for when the mat is read again
    assert node._latest_cmd_vel == drive

    node._on_id_notification(make_position_id_payload(250, 250, 0))
    assert node.pending_motor_cmd() == drive


def test_position_id_missed_stop_can_be_disabled(node):
    node.is_connected = True
    node.set_parameters([Parameter(
        'stop_on_position_id_missed', Parameter.Type.BOOL, False)])
    msg = Twist()
    msg.linear.x = 0.1
    node.cmd_vel_callback(msg)

    node._on_id_notification(make_position_id_missed_payload())
    assert node.pending_motor_cmd() == node._latest_cmd_vel


@pytest.mark.parametrize('missed_before', [True, False])
def test_position_id_missed_is_reset_on_connect(node, monkeypatch, missed_before):
    node.position_id_missed_pub = MagicMock()
    if missed_before:
        node._on_id_notification(make_position_id_missed_payload())
    node.position_id_missed_pub.publish.reset_mock()

    cube = make_connectable_cube()
    monkeypatch.setattr('toio_ros2.toio_ros2_node.ToioCoreCube', lambda **kwargs: cube)
    monkeypatch.setattr(
        ToioNode, 'scan_toio',
        AsyncMock(return_value=make_cube_info('toio Core Cube-C7f', 'AA:BB')))

    # the node fixture stubs out connect_toio, so call the real one
    asyncio.run(REAL_CONNECT_TOIO(node))

    assert node.is_connected
    assert not node._position_id_missed
    # published even when nothing changed, so the latched topic has a value
    node.position_id_missed_pub.publish.assert_called_once_with(Bool(data=False))


def test_motion_notification_publishes_motion(node):
    node.motion_pub = MagicMock()

    node._on_sensor_notification(make_motion_payload(
        horizontal=0, collision=1, double_tap=0, posture=3, shake=4))

    node.motion_pub.publish.assert_called_once()
    msg = node.motion_pub.publish.call_args[0][0]
    assert msg.header.frame_id == 'center'
    assert msg.horizontal is False
    assert msg.collision is True
    assert msg.double_tap is False
    assert msg.posture == MotionDetection.POSTURE_REAR
    assert msg.shake == 4


def test_motion_frame_uses_frame_prefix(node):
    node.motion_pub = MagicMock()
    node.frame_prefix = 'toio1/'
    node._on_sensor_notification(make_motion_payload())
    assert node.motion_pub.publish.call_args[0][0].header.frame_id == 'toio1/center'


def test_other_sensor_notifications_are_not_published(node):
    node.motion_pub = MagicMock()
    node.imu_pub = MagicMock()
    # posture angle (Euler) shares the sensor characteristic
    # https://toio.github.io/toio-spec/en/docs/ble_high_precision_tilt_sensor
    node._on_sensor_notification(bytearray(struct.pack('<BBhhh', 0x03, 0x01, 0, 0, 0)))
    # magnetic sensor
    node._on_sensor_notification(bytearray(struct.pack('<BBBbbb', 0x02, 0, 0, 0, 0, 0)))
    node.motion_pub.publish.assert_not_called()
    node.imu_pub.publish.assert_not_called()


def make_posture_quaternion_payload(w, x, y, z):
    # https://toio.github.io/toio-spec/en/docs/ble_high_precision_tilt_sensor#obtaining-posture-angle-information-notifications-in-quaternions
    return bytearray(struct.pack('<BBffff', 0x03, 0x02, w, x, y, z))


def test_posture_quaternion_is_published_as_imu_in_rep103(node):
    node.imu_pub = MagicMock()
    node.frame_prefix = 'toio1/'
    # nose up by 40deg is +pitch for the cube (x forward, y right, z down)
    from tf_transformations import quaternion_from_euler
    qx, qy, qz, qw = quaternion_from_euler(0.0, math.radians(40.0), 0.0)

    node._on_sensor_notification(make_posture_quaternion_payload(qw, qx, qy, qz))

    node.imu_pub.publish.assert_called_once()
    msg = node.imu_pub.publish.call_args[0][0]
    assert msg.header.frame_id == 'toio1/center'
    o = msg.orientation
    roll, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
    # and -pitch in REP-103 (x forward, y left, z up)
    assert math.degrees(pitch) == pytest.approx(-40.0, abs=1e-3)
    assert math.degrees(roll) == pytest.approx(0.0, abs=1e-3)
    assert math.degrees(yaw) == pytest.approx(0.0, abs=1e-3)
    assert msg.angular_velocity_covariance[0] == -1.0
    assert msg.linear_acceleration_covariance[0] == -1.0
    assert msg.orientation_covariance[8] > msg.orientation_covariance[0]


def test_posture_quaternion_ignored_when_imu_disabled(node):
    node.imu_pub = MagicMock()
    node.imu_interval_ms = 0
    node._on_sensor_notification(make_posture_quaternion_payload(1.0, 0.0, 0.0, 0.0))
    node.imu_pub.publish.assert_not_called()


def test_connect_applies_thresholds_and_reads_motion(node, monkeypatch):
    node.motion_pub = MagicMock()
    node.collision_threshold = 3
    node.horizontal_threshold = 20
    cube = make_connectable_cube()
    from toio.cube.api.sensor import MotionDetectionData
    cube.api.sensor.read = AsyncMock(
        return_value=MotionDetectionData(make_motion_payload(posture=2)))
    monkeypatch.setattr('toio_ros2.toio_ros2_node.ToioCoreCube', lambda **kwargs: cube)
    monkeypatch.setattr(
        ToioNode, 'scan_toio',
        AsyncMock(return_value=make_cube_info('toio Core Cube-C7f', 'AA:BB')))

    asyncio.run(REAL_CONNECT_TOIO(node))

    # https://toio.github.io/toio-spec/en/docs/ble_configuration#collision-detection-threshold-settings
    cube.api.configuration._write.assert_awaited_once_with(bytes((0x06, 0x00, 3)))
    cube.api.configuration.set_horizontal_detection_threshold.assert_awaited_once_with(20)
    cube.api.sensor.register_notification_handler.assert_awaited_once_with(
        node._on_sensor_notification)
    node.motion_pub.publish.assert_called_once()
    assert node.motion_pub.publish.call_args[0][0].posture == MotionDetection.POSTURE_BOTTOM


def test_connect_survives_motion_setup_failure(node, monkeypatch):
    node.motion_pub = MagicMock()
    cube = make_connectable_cube()
    cube.api.configuration._write = AsyncMock(side_effect=RuntimeError('ble'))
    cube.api.configuration.set_horizontal_detection_threshold = AsyncMock(
        side_effect=RuntimeError('ble'))
    cube.api.sensor.read = AsyncMock(side_effect=RuntimeError('ble'))
    monkeypatch.setattr('toio_ros2.toio_ros2_node.ToioCoreCube', lambda **kwargs: cube)
    monkeypatch.setattr(
        ToioNode, 'scan_toio',
        AsyncMock(return_value=make_cube_info('toio Core Cube-C7f', 'AA:BB')))

    asyncio.run(REAL_CONNECT_TOIO(node))

    assert node.is_connected
    node.motion_pub.publish.assert_not_called()


def run_diagnostic(task):
    return task(DiagnosticStatusWrapper())


def values(stat):
    return {kv.key: kv.value for kv in stat.values}


def test_diagnostics_connection(node):
    stat = run_diagnostic(node.diagnose_connection)
    assert stat.level == DiagnosticStatus.ERROR
    assert values(stat)['connected'] == 'False'

    node.is_connected = True
    node._cube_name = 'toio-abc (AA:BB)'
    stat = run_diagnostic(node.diagnose_connection)
    assert stat.level == DiagnosticStatus.OK
    assert values(stat)['cube'] == 'toio-abc (AA:BB)'

    node._docking = True
    stat = run_diagnostic(node.diagnose_connection)
    assert stat.level == DiagnosticStatus.OK
    assert 'docking' in stat.message


def test_diagnostics_battery_levels(node):
    assert run_diagnostic(node.diagnose_battery).level == DiagnosticStatus.WARN  # unknown yet
    for level, expected in ((100, DiagnosticStatus.OK), (30, DiagnosticStatus.OK),
                            (20, DiagnosticStatus.WARN), (10, DiagnosticStatus.ERROR)):
        node._battery_level = level
        stat = run_diagnostic(node.diagnose_battery)
        assert stat.level == expected, level
        assert values(stat)['level_percent'] == str(level)


def test_battery_notification_feeds_the_diagnostics(node):
    node.toio_battery_state_pub = MagicMock()
    # https://toio.github.io/toio-spec/en/docs/ble_battery
    node._on_battery_notification(bytearray(struct.pack('<B', 20)))
    assert node._battery_level == 20


def test_diagnostics_position(node):
    assert run_diagnostic(node.diagnose_position).level == DiagnosticStatus.WARN  # no pose yet

    node._last_pose_xy = (0.1, 0.2)
    stat = run_diagnostic(node.diagnose_position)
    assert stat.level == DiagnosticStatus.OK
    assert values(stat)['x'] == '0.100'

    node.position_id_missed_pub = MagicMock()
    node._on_id_notification(make_position_id_missed_payload())
    stat = run_diagnostic(node.diagnose_position)
    assert stat.level == DiagnosticStatus.WARN
    assert 'missed' in stat.message

    node._on_id_notification(make_position_id_payload(250, 250, 0))
    node.motion_pub = MagicMock()
    node._on_sensor_notification(make_motion_payload(horizontal=0, posture=4))
    stat = run_diagnostic(node.diagnose_position)
    assert stat.level == DiagnosticStatus.WARN
    assert 'tilted' in stat.message
    assert values(stat)['posture'] == 'Front'

    node._on_sensor_notification(make_motion_payload(horizontal=1, posture=1))
    assert run_diagnostic(node.diagnose_position).level == DiagnosticStatus.OK


def test_diagnostics_tasks_are_registered(node):
    names = [task.name for task in node.diagnostic_updater.tasks]
    assert names == ['connection', 'battery', 'position']


def make_button_payload(pressed):
    # https://toio.github.io/toio-spec/en/docs/ble_button#read-operations
    return bytearray(struct.pack('<BB', 0x01, 0x80 if pressed else 0x00))


def test_button_notification_publishes_state(node):
    node.button_pub = MagicMock()
    node._on_button_notification(make_button_payload(True))
    node.button_pub.publish.assert_called_once_with(Bool(data=True))
    assert node._button_pressed
    node._on_button_notification(make_button_payload(False))
    assert node.button_pub.publish.call_args[0][0] == Bool(data=False)
    assert not node._button_pressed


def test_button_topic_is_latched(node):
    assert node.button_pub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_button_stops_the_motor_only_when_enabled(node):
    node.is_connected = True
    msg = Twist()
    msg.linear.x = 0.1
    node.cmd_vel_callback(msg)
    drive = node._latest_cmd_vel

    node._on_button_notification(make_button_payload(True))
    assert node.pending_motor_cmd() == drive  # default: the button is just reported

    node.set_parameters([Parameter('stop_on_button', Parameter.Type.BOOL, True)])
    assert node.pending_motor_cmd() == (0, 0)
    assert node._latest_cmd_vel == drive  # kept, so releasing resumes

    node._on_button_notification(make_button_payload(False))
    assert node.pending_motor_cmd() == drive


def test_connect_reads_the_button_state(node, monkeypatch):
    node.button_pub = MagicMock()
    cube = make_connectable_cube()
    from toio.cube.api.button import ButtonInformation
    cube.api.button.read = AsyncMock(return_value=ButtonInformation(make_button_payload(True)))
    monkeypatch.setattr('toio_ros2.toio_ros2_node.ToioCoreCube', lambda **kwargs: cube)
    monkeypatch.setattr(
        ToioNode, 'scan_toio',
        AsyncMock(return_value=make_cube_info('toio Core Cube-C7f', 'AA:BB')))

    asyncio.run(REAL_CONNECT_TOIO(node))

    cube.api.button.register_notification_handler.assert_awaited_once_with(
        node._on_button_notification)
    node.button_pub.publish.assert_called_once_with(Bool(data=True))
    assert node._button_pressed


def test_connect_assumes_released_if_the_button_read_fails(node, monkeypatch):
    node.button_pub = MagicMock()
    node._button_pressed = True
    cube = make_connectable_cube()
    cube.api.button.read = AsyncMock(side_effect=RuntimeError('ble'))
    monkeypatch.setattr('toio_ros2.toio_ros2_node.ToioCoreCube', lambda **kwargs: cube)
    monkeypatch.setattr(
        ToioNode, 'scan_toio',
        AsyncMock(return_value=make_cube_info('toio Core Cube-C7f', 'AA:BB')))

    asyncio.run(REAL_CONNECT_TOIO(node))

    assert node.is_connected
    assert not node._button_pressed
    node.button_pub.publish.assert_called_once_with(Bool(data=False))


def make_motor_speed_payload(left, right):
    # https://toio.github.io/toio-spec/en/docs/ble_motor#obtaining-motor-speed-information
    return bytearray(struct.pack('<BBB', 0xE0, left, right))


def test_motor_speed_notification_is_stored(node):
    node._on_motor_notification(make_motor_speed_payload(30, 40))
    assert node._wheel_speed == (30, 40)


def test_speed_to_mps_matches_the_cube_maximum(node):
    v = node.speed_to_mps(int(node.max_input_speed))
    assert v == pytest.approx(node.max_rpm / 60.0 * 2.0 * math.pi * node.wheel_radius)
    assert node.speed_to_mps(0) == 0.0


def test_wheel_velocities_take_the_sign_from_the_last_command(node):
    # the cube reports magnitudes only, so the command decides the direction
    node._wheel_speed = (30, 30)
    node._wheel_dir = (-1, 1)
    v_l, v_r = node.wheel_velocities()
    assert v_l == pytest.approx(-node.speed_to_mps(30))
    assert v_r == pytest.approx(node.speed_to_mps(30))


def test_motor_control_remembers_the_direction_across_a_stop(node):
    node.cube = MagicMock()
    node.cube.api.motor.motor_control = AsyncMock()
    asyncio.run(node.motor_control(-30, 30))
    assert node._wheel_dir == (-1, 1)
    # the wheels coast after a stop, so the reported speed keeps its sign
    asyncio.run(node.motor_control(0, 0))
    assert node._wheel_dir == (-1, 1)
    asyncio.run(node.motor_control(30, 0))
    assert node._wheel_dir == (1, 1)


def test_integrate_odom_straight_line(node):
    node._wheel_speed = (30, 30)
    v, omega = node.integrate_odom(1.0)
    assert v == pytest.approx(node.speed_to_mps(30))
    assert omega == 0.0
    x, y, yaw = node._odom_pose
    assert x == pytest.approx(v)
    assert y == 0.0
    assert yaw == 0.0


def test_integrate_odom_spin_in_place(node):
    node._wheel_speed = (30, 30)
    node._wheel_dir = (-1, 1)
    v, omega = node.integrate_odom(0.1)
    assert v == pytest.approx(0.0)
    assert omega == pytest.approx(2.0 * node.speed_to_mps(30) / node.wheel_base)
    x, y, yaw = node._odom_pose
    assert (x, y) == (0.0, 0.0)
    assert yaw == pytest.approx(omega * 0.1)


def test_compose_map_to_odom_puts_the_odom_pose_on_the_map_pose():
    # a point at (1, 0, 90deg) in odom and (0, 1, 180deg) in map: odom sits
    # rotated by 90deg and the point itself rotates onto (0, 1)
    x, y, yaw = ToioNode.compose_map_to_odom((0.0, 1.0, math.pi), (1.0, 0.0, math.pi / 2))
    assert yaw == pytest.approx(math.pi / 2)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
    # and the identity case
    assert ToioNode.compose_map_to_odom((0.3, 0.2, 0.1), (0.3, 0.2, 0.1)) == \
        pytest.approx((0.0, 0.0, 0.0))


def test_position_id_updates_map_to_odom_instead_of_the_transform(node):
    node.toio_pose_pub = MagicMock()
    node.tf_broadcaster = MagicMock()
    node._odom_pose = (0.1, 0.0, 0.0)

    node._on_id_notification(make_position_id_payload(250, 250, 0))

    node.toio_pose_pub.publish.assert_called_once()
    node.tf_broadcaster.sendTransform.assert_not_called()
    msg = node.toio_pose_pub.publish.call_args[0][0]
    x, y, yaw = node._map_to_odom
    assert x == pytest.approx(msg.pose.position.x - 0.1)
    assert y == pytest.approx(msg.pose.position.y)
    assert yaw == pytest.approx(0.0)


def test_odom_timer_publishes_odometry_and_both_transforms(node):
    node.odom_pub = MagicMock()
    node.tf_broadcaster = MagicMock()
    node.frame_prefix = 'toio1/'
    node.is_connected = True
    node._wheel_speed = (30, 30)
    node._map_to_odom = (0.2, 0.1, 0.0)

    node.odom_timer_callback()  # first tick only takes the time
    node.odom_pub.publish.assert_not_called()
    node._last_odom_time -= rclpy.duration.Duration(seconds=0.5)
    node.odom_timer_callback()

    odom = node.odom_pub.publish.call_args[0][0]
    assert odom.header.frame_id == 'toio1/odom'
    assert odom.child_frame_id == 'toio1/center'
    assert odom.twist.twist.linear.x == pytest.approx(node.speed_to_mps(30))
    assert odom.pose.pose.position.x == pytest.approx(node.speed_to_mps(30) * 0.5, rel=0.05)
    transforms = node.tf_broadcaster.sendTransform.call_args[0][0]
    assert [(t.header.frame_id, t.child_frame_id) for t in transforms] == [
        ('map', 'toio1/odom'), ('toio1/odom', 'toio1/center')]
    assert transforms[0].transform.translation.x == pytest.approx(0.2)
    assert transforms[0].header.stamp == transforms[1].header.stamp == odom.header.stamp


def test_odom_timer_does_nothing_while_disconnected(node):
    node.odom_pub = MagicMock()
    node.tf_broadcaster = MagicMock()
    node._last_odom_time = node.get_clock().now()
    node.odom_timer_callback()
    node.odom_pub.publish.assert_not_called()
    # the gap is not motion: the next connected tick restarts the clock
    assert node._last_odom_time is None


def test_connect_enables_motor_speed_notification(node, monkeypatch):
    cube = make_connectable_cube()
    monkeypatch.setattr('toio_ros2.toio_ros2_node.ToioCoreCube', lambda **kwargs: cube)
    monkeypatch.setattr(
        ToioNode, 'scan_toio',
        AsyncMock(return_value=make_cube_info('toio Core Cube-C7f', 'AA:BB')))

    asyncio.run(REAL_CONNECT_TOIO(node))

    from toio.cube.api.configuration import (MotorSpeedInformationAcquisitionState,
                                             PostureAngleDetectionCondition,
                                             PostureAngleDetectionType)
    cube.api.configuration.set_motor_speed_information_acquisition.assert_awaited_once_with(
        MotorSpeedInformationAcquisitionState.Enable)
    cube.api.configuration.set_posture_angle_detection.assert_awaited_once_with(
        PostureAngleDetectionType.Quaternions, node.imu_interval_ms,
        PostureAngleDetectionCondition.Always)


def test_reconnect_clears_the_wheel_speed(node, scheduled):
    node.is_connected = True
    node._wheel_speed = (30, 30)
    node._schedule_reconnect()
    assert node._wheel_speed == (0, 0)


def test_toio_transform_uses_frame_prefix(node):
    # default: no prefix
    transform = node.make_toio_transform(0.1, -0.1, 0.0, 0.0, 0.0, 1.0)
    assert transform.child_frame_id == 'center'

    # multi-cube setup: prefixed child frame, shared map frame
    node.frame_prefix = 'toio1/'
    transform = node.make_toio_transform(0.1, -0.1, 0.0, 0.0, 0.0, 1.0)
    assert transform.header.frame_id == 'map'
    assert transform.child_frame_id == 'toio1/center'


def test_battery_notification_publishes_state(node):
    node.toio_battery_state_pub = MagicMock()

    node._on_battery_notification(bytearray([80]))

    node.toio_battery_state_pub.publish.assert_called_once()
    msg = node.toio_battery_state_pub.publish.call_args[0][0]
    assert msg.percentage == pytest.approx(0.8)
    assert msg.present is True


def test_motor_notification_accepts_target_responses(node):
    # https://toio.github.io/toio-spec/en/docs/ble_motor#responses-to-motor-control-with-target-specified
    success = bytearray(struct.pack('<BBB', 0x83, 0, 0))
    id_missed = bytearray(struct.pack('<BBB', 0x83, 0, 2))
    # motor speed information (0xe0) is not a target response and is ignored
    motor_speed = bytearray(struct.pack('<BBB', 0xe0, 10, 10))

    node._on_motor_notification(success)
    node._on_motor_notification(id_missed)
    node._on_motor_notification(motor_speed)


# --------------------------------------------------------------------------
# dock_to_pose action (toio_fleet_adapter#3)
# --------------------------------------------------------------------------
def make_motor_target_payload(response_code):
    # https://toio.github.io/toio-spec/en/docs/ble_motor#responses-to-motor-control-with-target-specified
    return bytearray(struct.pack('<BBB', 0x83, 0, response_code))


def make_dock_goal_handle(x=0.05, y=-0.10, yaw=0.0):
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
    goal_handle = MagicMock()
    goal_handle.request = goal
    goal_handle.is_cancel_requested = False
    return goal_handle


def arm_dock(node):
    """Put the node in the state the action server's goal callback leaves."""
    node.is_connected = True
    node.cube = MagicMock()
    node.cube.api.motor.motor_control = AsyncMock()
    assert node.dock_goal_callback(NavigateToPose.Goal()) == GoalResponse.ACCEPT


def run_dock(node, goal_handle):
    """Run the blocking execute callback on its own thread."""
    outcome = {}

    def target():
        outcome['result'] = node.dock_execute_callback(goal_handle)

    thread = threading.Thread(target=target)
    thread.start()
    return thread, outcome


def wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def finish_dock(node, goal_handle, response_code, timeout=5.0):
    """Run a dock to completion, answering it with `response_code`."""
    thread, outcome = run_dock(node, goal_handle)
    assert wait_until(lambda: node._dock_started_at > 0.0)
    node._on_motor_notification(make_motor_target_payload(response_code))
    thread.join(timeout=timeout)
    assert not thread.is_alive()
    return outcome['result']


def test_dock_action_server_exists_without_goal_pose_motion(node_no_goal_pose):
    # the ADR-5 exception: the topic is closed, the dock action is not
    assert node_no_goal_pose.goal_pose_sub is None
    assert node_no_goal_pose.dock_action_server is not None


def test_dock_goal_rejected_when_disconnected(node):
    assert node.dock_goal_callback(NavigateToPose.Goal()) == GoalResponse.REJECT


def test_dock_goal_rejected_while_already_docking(node):
    arm_dock(node)
    # the cube's response carries no request id, so a second dock could
    # complete the first one
    assert node.dock_goal_callback(NavigateToPose.Goal()) == GoalResponse.REJECT


def test_dock_cancel_is_always_accepted(node):
    assert node.dock_cancel_callback(MagicMock()) == CancelResponse.ACCEPT


def test_dock_converts_the_goal_to_cube_coordinates(node, scheduled, monkeypatch):
    target = MagicMock()
    monkeypatch.setattr(node, 'motor_control_target', target)
    arm_dock(node)
    pos_x, pos_y, q_x, q_y, q_z, q_w = node.convert_toio_to_ros_coord(250, 250, 90)
    goal_handle = make_dock_goal_handle()
    goal_handle.request.pose.pose.position.x = pos_x
    goal_handle.request.pose.pose.position.y = pos_y
    goal_handle.request.pose.pose.orientation.x = q_x
    goal_handle.request.pose.pose.orientation.y = q_y
    goal_handle.request.pose.pose.orientation.z = q_z
    goal_handle.request.pose.pose.orientation.w = q_w

    finish_dock(node, goal_handle, MotorResponseCode.SUCCESS.value)

    x, y, angle = target.call_args[0]
    assert abs(x - 250) <= 1
    assert abs(y - 250) <= 1
    assert abs((angle - 90 + 180) % 360 - 180) <= 1


def test_dock_success_completes_the_goal(node, scheduled):
    arm_dock(node)
    goal_handle = make_dock_goal_handle()

    result = finish_dock(node, goal_handle, MotorResponseCode.SUCCESS.value)

    goal_handle.succeed.assert_called_once()
    goal_handle.abort.assert_not_called()
    assert result.error_code == NavigateToPose.Result.NONE
    assert result.error_msg == 'SUCCESS'
    assert node._docking is False


def test_dock_overwrite_is_reported_as_success(node, scheduled):
    # the cube may have stopped short, but Nav2 already put it on the
    # waypoint, so a preempted refinement is not a task failure
    arm_dock(node)
    goal_handle = make_dock_goal_handle()

    result = finish_dock(
        node, goal_handle, MotorResponseCode.SUCCESS_WITH_OVERWRITE.value)

    goal_handle.succeed.assert_called_once()
    assert result.error_code == NavigateToPose.Result.NONE
    assert result.error_msg == 'SUCCESS_WITH_OVERWRITE'


def test_dock_aborts_when_the_cube_loses_the_position_id(node, scheduled):
    arm_dock(node)
    goal_handle = make_dock_goal_handle()

    result = finish_dock(
        node, goal_handle, MotorResponseCode.ERROR_ID_MISSED.value)

    goal_handle.abort.assert_called_once()
    goal_handle.succeed.assert_not_called()
    assert result.error_code == MotorResponseCode.ERROR_ID_MISSED.value
    assert result.error_msg == 'ERROR_ID_MISSED'


def test_dock_cancel_stops_the_motor(node, scheduled):
    arm_dock(node)
    goal_handle = make_dock_goal_handle()
    goal_handle.is_cancel_requested = True

    thread, outcome = run_dock(node, goal_handle)
    thread.join(timeout=5.0)
    assert not thread.is_alive()

    goal_handle.canceled.assert_called_once()
    assert outcome['result'].error_msg == 'canceled'
    # cancelling the action is the only thing that stops a target motion:
    # it runs inside the cube, not on cmd_vel
    node.cube.api.motor.motor_control.assert_called_once()
    assert node._docking is False


def test_dock_uses_its_own_short_timeout(node, scheduled, monkeypatch):
    target = MagicMock()
    monkeypatch.setattr(node, 'motor_control_target', target)
    arm_dock(node)
    node.dock_timeout = 7

    finish_dock(node, make_dock_goal_handle(), MotorResponseCode.SUCCESS.value)

    # not goal_timeout (60s): a cube that cannot reach the target because
    # something is standing on it would keep pushing for a full minute
    assert target.call_args.kwargs['timeout'] == 7
    assert node.goal_timeout != node.dock_timeout


def test_dock_aborts_when_the_cube_never_answers(node, scheduled):
    arm_dock(node)
    node.dock_timeout = 0
    node.DOCK_RESPONSE_GRACE = 0.2
    node.DOCK_POLL_INTERVAL = 0.02
    goal_handle = make_dock_goal_handle()

    thread, outcome = run_dock(node, goal_handle)
    thread.join(timeout=5.0)
    assert not thread.is_alive()

    goal_handle.abort.assert_called_once()
    assert outcome['result'].error_code == MotorResponseCode.ERROR_TIMEOUT.value


def test_dock_aborts_when_ble_drops(node, scheduled):
    arm_dock(node)
    node.DOCK_POLL_INTERVAL = 0.02
    goal_handle = make_dock_goal_handle()

    thread, outcome = run_dock(node, goal_handle)
    assert wait_until(lambda: node._dock_started_at > 0.0)
    node.is_connected = False
    thread.join(timeout=5.0)
    assert not thread.is_alive()

    goal_handle.abort.assert_called_once()
    assert outcome['result'].error_code == \
        MotorResponseCode.ERROR_INVALID_CUBE_STATE.value


def test_dock_clears_its_state_even_when_it_fails(node, scheduled):
    arm_dock(node)
    node._last_motor_cmd_time = time.monotonic()

    finish_dock(node, make_dock_goal_handle(),
                MotorResponseCode.ERROR_ID_MISSED.value)

    assert node._docking is False
    # the resumed cmd_vel is often the same command as before the dock, and
    # motor_control() would otherwise dedup the first write back away
    assert node._last_motor_cmd_time == 0.0


def test_cmd_vel_is_dropped_while_docking(node):
    node.is_connected = True
    node._docking = True
    msg = Twist()
    msg.linear.x = 0.1

    node.cmd_vel_callback(msg)

    # dropped, not stored: applying it after the dock would drive the cube
    # straight off the pose it just reached
    assert node._latest_cmd_vel is None


def test_goal_pose_is_ignored_while_docking(node, scheduled):
    node.is_connected = True
    node._docking = True
    node.goal_pose_callback(PoseStamped())
    assert not scheduled


def test_motor_command_loop_is_quiesced_while_docking(node):
    node.is_connected = True
    node._latest_cmd_vel = (50, 50)
    node._latest_cmd_vel_stamp = time.monotonic()
    assert node.pending_motor_cmd() == (50, 50)

    node._docking = True
    # otherwise the stop below would overwrite the target motion
    assert node.pending_motor_cmd() is None


def test_motor_command_loop_stops_when_cmd_vel_goes_silent(node):
    node.is_connected = True
    node._latest_cmd_vel = (50, 50)
    node._latest_cmd_vel_stamp = time.monotonic() - node.cmd_vel_timeout - 0.1
    assert node.pending_motor_cmd() == (0, 0)


def make_cube_info(name, address):
    return CubeInfo(
        name=name,
        device=SimpleNamespace(address=address),
        interface=MagicMock(),
        advertisement=None)


def test_scan_toio_with_cube_id(node, monkeypatch):
    info = make_cube_info('toio Core Cube-C7f', 'AA:BB')
    scanner = MagicMock()
    scanner.scan_with_id = AsyncMock(return_value=[info])
    monkeypatch.setattr('toio_ros2.toio_ros2_node.BLEScanner', scanner)

    node.cube_id = 'C7f'
    assert asyncio.run(node.scan_toio()) is info
    scanner.scan_with_id.assert_awaited_once_with(cube_id={'C7f'})


def test_scan_toio_with_cube_address(node, monkeypatch):
    info = make_cube_info('toio Core Cube-C7f', 'AA:BB')
    scanner = MagicMock()
    scanner.scan_with_address = AsyncMock(return_value=[info])
    monkeypatch.setattr('toio_ros2.toio_ros2_node.BLEScanner', scanner)

    node.cube_address = 'AA:BB'
    assert asyncio.run(node.scan_toio()) is info
    scanner.scan_with_address.assert_awaited_once_with(address={'AA:BB'})


def test_scan_toio_cube_id_takes_precedence(node, monkeypatch):
    info = make_cube_info('toio Core Cube-C7f', 'AA:BB')
    scanner = MagicMock()
    scanner.scan_with_id = AsyncMock(return_value=[info])
    scanner.scan_with_address = AsyncMock(return_value=[])
    monkeypatch.setattr('toio_ros2.toio_ros2_node.BLEScanner', scanner)

    node.cube_id = 'C7f'
    node.cube_address = 'AA:BB'
    assert asyncio.run(node.scan_toio()) is info
    scanner.scan_with_address.assert_not_awaited()


def test_scan_toio_auto_mode_picks_nearest(node, monkeypatch):
    nearest = make_cube_info('toio Core Cube-C7f', 'AA:BB')
    other = make_cube_info('toio Core Cube-p9G', 'CC:DD')
    scanner = MagicMock()
    scanner.scan = AsyncMock(return_value=[nearest, other])
    monkeypatch.setattr('toio_ros2.toio_ros2_node.BLEScanner', scanner)

    # default: cube_id and cube_address are both empty
    assert asyncio.run(node.scan_toio()) is nearest
    scanner.scan.assert_awaited_once_with(AUTO_CONNECT_SCAN_NUM)


def test_scan_toio_returns_none_when_not_found(node, monkeypatch):
    scanner = MagicMock()
    scanner.scan = AsyncMock(return_value=[])
    scanner.scan_with_id = AsyncMock(return_value=[])
    monkeypatch.setattr('toio_ros2.toio_ros2_node.BLEScanner', scanner)

    assert asyncio.run(node.scan_toio()) is None
    node.cube_id = 'C7f'
    assert asyncio.run(node.scan_toio()) is None


def test_schedule_reconnect_runs_only_once(node, scheduled):
    node.is_connected = True
    node._schedule_reconnect()
    node._schedule_reconnect()

    assert node.is_connected is False
    assert len(scheduled) == 1


def test_send_intervals_default_to_the_previous_constants(node):
    # These were node constants before issue #32; parameterizing them must not
    # change what a deployment that sets nothing gets
    assert node.led_write_interval == pytest.approx(0.1)
    assert node.sound_min_interval == pytest.approx(0.1)
    assert node.motor_dedup_interval == pytest.approx(0.3)


@pytest.mark.parametrize('name', SEND_INTERVALS)
def test_send_interval_can_be_set_at_runtime(node, name):
    node.set_parameters([rclpy.parameter.Parameter(
        name, rclpy.Parameter.Type.DOUBLE, 0.25)])
    assert getattr(node, name) == pytest.approx(0.25)


@pytest.mark.parametrize('name', SEND_INTERVALS)
@pytest.mark.parametrize('bad', [0.0, -1.0, 0.001, float('nan')])
def test_send_interval_below_the_floor_is_rejected(node, name, bad):
    # led_command_loop() sleeps for this: zero or negative would spin the
    # asyncio loop, and NaN compares false against a plain lower bound.
    # Rejecting rather than clamping keeps `ros2 param get` honest - clamping
    # left the parameter reporting 0.0 while the node ran at 0.02.
    before = getattr(node, name)
    result = node.set_parameters([rclpy.parameter.Parameter(
        name, rclpy.Parameter.Type.DOUBLE, bad)])

    assert result[0].successful is False
    assert name in result[0].reason
    assert getattr(node, name) == pytest.approx(before)
    assert node.get_parameter(name).value == pytest.approx(before)


@pytest.mark.parametrize('name', SEND_INTERVALS)
@pytest.mark.parametrize('bad', [0.0, -1.0, float('nan')])
def test_send_interval_from_a_params_file_is_clamped(monkeypatch, name, bad):
    # A bad value at startup must not stop the node from coming up
    async def _noop(self):
        return None

    monkeypatch.setattr(ToioNode, 'connect_toio', _noop)
    monkeypatch.setattr(ToioNode, 'motor_command_loop', _noop)
    rclpy.init()
    try:
        node = ToioNode()
        node.set_parameters_atomically([rclpy.parameter.Parameter(
            name, rclpy.Parameter.Type.DOUBLE, ToioNode.MIN_SEND_INTERVAL)])
        assert node.clamp_send_interval(name, bad) == pytest.approx(
            ToioNode.MIN_SEND_INTERVAL)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_sound_throttle_follows_the_parameter(node, scheduled):
    node.is_connected = True
    node.set_parameters([rclpy.parameter.Parameter(
        'sound_min_interval', rclpy.Parameter.Type.DOUBLE, 0.5)])

    node.sound_callback(UInt8(data=0))
    assert len(scheduled) == 1

    # Inside the new interval: dropped, where the old 0.1s would have passed
    node._last_sound_time -= 0.2
    node.sound_callback(UInt8(data=0))
    assert len(scheduled) == 1

    node._last_sound_time -= 0.5
    node.sound_callback(UInt8(data=0))
    assert len(scheduled) == 2


@pytest.mark.parametrize('bad', [0.5, 0.75])
def test_motor_dedup_above_the_auto_stop_is_rejected(node, bad):
    # Every motor command carries a 500ms auto-stop, so resending at or after
    # that leaves the cube stopped between commands and the robot stutters
    before = node.motor_dedup_interval
    result = node.set_parameters([rclpy.parameter.Parameter(
        'motor_dedup_interval', rclpy.Parameter.Type.DOUBLE, bad)])

    assert result[0].successful is False
    assert 'auto-stop' in result[0].reason
    assert node.motor_dedup_interval == pytest.approx(before)


def test_motor_dedup_above_the_auto_stop_from_a_params_file_is_clamped(node):
    # Startup must not refuse to come up; half the auto-stop leaves room for
    # one missed resend
    assert node.clamp_send_interval('motor_dedup_interval', 5.0) == \
        pytest.approx(ToioNode.MOTOR_DURATION_MS / 2000.0)


def test_motor_resend_follows_the_parameter(node):
    node.cube = MagicMock()
    node.cube.api.motor.motor_control = AsyncMock()
    node.set_parameters([rclpy.parameter.Parameter(
        'motor_dedup_interval', rclpy.Parameter.Type.DOUBLE, 0.1)])

    asyncio.run(node.motor_control(50, 50))
    assert node.cube.api.motor.motor_control.await_count == 1

    # The old 0.3s constant would still be deduplicating here
    node._last_motor_cmd_time -= 0.15
    asyncio.run(node.motor_control(50, 50))
    assert node.cube.api.motor.motor_control.await_count == 2


def test_motor_command_carries_the_auto_stop_duration(node):
    node.cube = MagicMock()
    node.cube.api.motor.motor_control = AsyncMock()

    asyncio.run(node.motor_control(50, 50))

    kwargs = node.cube.api.motor.motor_control.call_args[1]
    assert kwargs['duration_ms'] == ToioNode.MOTOR_DURATION_MS


def led_msg(r, g, b, duration_ms):
    msg = Led()
    msg.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
    msg.duration_ms = duration_ms
    return msg


def test_led_timed_carries_its_own_duration(node):
    node.is_connected = True

    node.led_timed_callback(led_msg(1.0, 0.0, 0.0, 300))

    assert node._pending_led == (255, 0, 0, 300)


def test_led_timed_duration_is_clipped_to_the_cube_limit(node):
    node.is_connected = True

    node.led_timed_callback(led_msg(1.0, 0.0, 0.0, 60000))

    assert node._pending_led[3] == Led.DURATION_MAX_MS


def test_led_timed_duration_reaches_the_cube(node):
    node.is_connected = True
    node.cube = MagicMock()
    node.cube.api.indicator.turn_on = AsyncMock()

    node.led_timed_callback(led_msg(1.0, 0.0, 0.0, 300))
    asyncio.run(node.flush_led())

    param = node.cube.api.indicator.turn_on.call_args[0][0]
    assert param.duration_ms == 300
    # the node-wide default must not win over the message
    assert param.duration_ms != node.led_duration_ms


def test_led_pattern_is_sent_to_the_cube(node, scheduled):
    node.is_connected = True
    msg = LedPattern()
    msg.steps = [led_msg(1.0, 0.0, 0.0, 500), led_msg(0.0, 0.0, 0.0, 500)]
    msg.repeat = 3

    node.led_pattern_callback(msg)

    assert len(scheduled) == 1


def test_led_pattern_drops_the_latched_single_color(node, scheduled):
    node.is_connected = True
    node.led_callback(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
    assert node._pending_led is not None

    msg = LedPattern()
    msg.steps = [led_msg(0.0, 0.0, 1.0, 500)]
    node.led_pattern_callback(msg)

    # otherwise the next flush would overwrite the running pattern
    assert node._pending_led is None
    assert node._last_led_cmd is None


def test_led_pattern_rejects_an_oversized_sequence(node, scheduled):
    node.is_connected = True
    msg = LedPattern()
    msg.steps = [led_msg(1.0, 0.0, 0.0, 100)] * (LedPattern.STEPS_MAX + 1)

    node.led_pattern_callback(msg)

    # truncating would play a pattern nobody asked for
    assert not scheduled


def test_led_pattern_rejects_an_empty_sequence(node, scheduled):
    node.is_connected = True

    node.led_pattern_callback(LedPattern())

    assert not scheduled


def test_melody_is_sent_to_the_cube(node, scheduled):
    node.is_connected = True
    msg = Melody()
    msg.notes = [MidiNoteMsg(duration_ms=200, note=60, volume=255)]

    node.melody_callback(msg)

    assert len(scheduled) == 1


def test_melody_rejects_a_note_above_the_cube_range(node, scheduled):
    node.is_connected = True
    msg = Melody()
    msg.notes = [MidiNoteMsg(duration_ms=200, note=200, volume=255)]

    node.melody_callback(msg)

    assert not scheduled


def test_melody_rejects_an_oversized_sequence(node, scheduled):
    node.is_connected = True
    msg = Melody()
    msg.notes = [MidiNoteMsg(duration_ms=100, note=60, volume=255)] * (
        Melody.NOTES_MAX + 1)

    node.melody_callback(msg)

    assert not scheduled


def test_melody_shares_the_sound_throttle(node, scheduled):
    node.is_connected = True
    msg = Melody()
    msg.notes = [MidiNoteMsg(duration_ms=100, note=60, volume=255)]

    node.melody_callback(msg)
    assert len(scheduled) == 1

    # a sound effect right after must be dropped: both use the cube's one
    # sound channel
    node.sound_callback(UInt8(data=int(SoundId.Get1)))
    assert len(scheduled) == 1

    node._last_sound_time -= node.sound_min_interval
    node.sound_callback(UInt8(data=int(SoundId.Get1)))
    assert len(scheduled) == 2


def test_led_pattern_and_melody_ignored_when_disconnected(node, scheduled):
    pattern = LedPattern()
    pattern.steps = [led_msg(1.0, 0.0, 0.0, 100)]
    melody = Melody()
    melody.notes = [MidiNoteMsg(duration_ms=100, note=60, volume=255)]

    node.led_pattern_callback(pattern)
    node.melody_callback(melody)

    assert not scheduled


def test_connect_requests_motion_when_the_read_returns_something_else(node, monkeypatch):
    node.motion_pub = MagicMock()
    cube = make_connectable_cube()
    # the characteristic is shared with the posture angle
    from toio.cube.api.sensor import PostureAngleQuaternionsData
    cube.api.sensor.read = AsyncMock(return_value=PostureAngleQuaternionsData(
        make_posture_quaternion_payload(1.0, 0.0, 0.0, 0.0)))
    monkeypatch.setattr('toio_ros2.toio_ros2_node.ToioCoreCube', lambda **kwargs: cube)
    monkeypatch.setattr(
        ToioNode, 'scan_toio',
        AsyncMock(return_value=make_cube_info('toio Core Cube-C7f', 'AA:BB')))

    asyncio.run(REAL_CONNECT_TOIO(node))

    cube.api.sensor.request_motion_information.assert_awaited_once()
    node.motion_pub.publish.assert_not_called()

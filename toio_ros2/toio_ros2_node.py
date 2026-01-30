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
import threading
import math
import time

import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped

from toio import *


class ToioNode(Node):
    def __init__(self) -> None:
        super().__init__('toio_ros2_node')
        # https://toio.github.io/toio-spec/en/docs/hardware_shape
        self.wheel_base = 0.0266 # meter
        self.wheel_radius = 0.00625 # meter
        self.cube_height = 0.0256 # meter

        # https://toio.github.io/toio-spec/en/docs/ble_motor
        self.max_rpm = 494.0
        self.max_input_speed = 115.0
        self.is_connected = False

        # Command deduplication with time-based resend
        self._last_motor_cmd: tuple = (0, 0)
        self._last_motor_cmd_time: float = 0.0
        self.motor_dedup_interval = 0.3  # seconds

        # Default is a param for A4 mat https://toio.github.io/toio-spec/docs/hardware_position_id
        self.declare_parameter('field_min_x', 98.0)
        self.declare_parameter('field_max_x', 402.0)
        self.declare_parameter('field_min_y', 142.0)
        self.declare_parameter('field_max_y', 358.0)
        self.declare_parameter('field_width_meter', 0.297)
        self.declare_parameter('field_height_meter', 0.210)

        # Get params for field information
        self.field_min_x = self.get_parameter('field_min_x').get_parameter_value().double_value
        self.field_max_x = self.get_parameter('field_max_x').get_parameter_value().double_value
        self.field_min_y = self.get_parameter('field_min_y').get_parameter_value().double_value
        self.field_max_y = self.get_parameter('field_max_y').get_parameter_value().double_value
        self.field_width_meter = self.get_parameter('field_width_meter').get_parameter_value().double_value
        self.field_height_meter = self.get_parameter('field_height_meter').get_parameter_value().double_value

        # calculate scale
        self.scale_x = self.field_width_meter / (self.field_max_x - self.field_min_x)
        self.scale_y = self.field_height_meter / (self.field_max_y - self.field_min_y)
        self.get_logger().debug(f"scale_x = {self.scale_x}, scale_y = {self.scale_y}")

        # subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.cmd_vel_sub
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10)
        self.goal_pose_sub

        # publisher
        self.toio_pose_pub = self.create_publisher(PoseStamped, 'toio/pose', qos_profile=10)
        self.toio_battery_level_pub = self.create_publisher(Float32, 'toio/battery_level', qos_profile=10)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # timer (50Hz for position to match control rate, 1Hz for battery)
        self.monitor_id_information_timer = self.create_timer(0.02, self.monitor_id_information_callback)
        self.monitor_battery_information_timer = self.create_timer(1.0, self.monitor_battery_information_callback)

        # Track pending async operations to avoid blocking
        self._position_read_pending = False
        self._battery_read_pending = False

        # create thread to call toio API
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(
            target=self.start_loop, daemon=True)
        self.thread.start()
        asyncio.run_coroutine_threadsafe(
            self.connect_toio(),
            self.loop)

    def start_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def cmd_vel_callback(self, msg: Twist) -> None:
        if not self.is_connected:
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
        self.get_logger().debug(f"rpm_l = {rpm_l}, rpm_r = {rpm_r}")

        # RPM -> toio motor_speed
        left_motor_speed = int((rpm_l / self.max_rpm) * self.max_input_speed)
        right_motor_speed = int((rpm_r / self.max_rpm) * self.max_input_speed)

        asyncio.run_coroutine_threadsafe(
            self.motor_control(left_motor_speed, right_motor_speed),
            self.loop)

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        if not self.is_connected:
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

    def linear_speed_to_rpm(self, v_lin: float) -> float:
        return (v_lin / (2.0 * math.pi * self.wheel_radius)) * 60.0

    def monitor_id_information_callback(self) -> None:
        if not self.is_connected:
            return

        # Skip if a read is already pending (non-blocking pattern)
        if self._position_read_pending:
            return

        self._position_read_pending = True
        future = asyncio.run_coroutine_threadsafe(
            self.get_cube_location(),
            self.loop)
        future.add_done_callback(self._handle_position_result)

    def _handle_position_result(self, future) -> None:
        """Handle position read result (non-blocking callback)."""
        self._position_read_pending = False
        try:
            result = future.result(timeout=0)
            self.get_logger().debug(f'get_cube_location(), result = {result}')

            if result[0] and result[1]:
                # convert ROS 2 coordinate
                pos_x, pos_y, q_x, q_y, q_z, q_w = self.convert_toio_to_ros_coord(result[0][0], result[0][1], result[1])

                # publish PoseStamped
                toio_pose_stamped_msg = self.make_pose_stamped_msg(pos_x, pos_y, q_x, q_y, q_z, q_w)
                self.toio_pose_pub.publish(toio_pose_stamped_msg)

                # send the transformation
                toio_transform = self.make_toio_transform(pos_x, pos_y, q_x, q_y, q_z, q_w)
                self.tf_broadcaster.sendTransform(toio_transform)
        except Exception as e:
            self.get_logger().warn(f'Position read failed: {e}')

    def convert_toio_to_ros_coord(self, x, y, angle):
        pos_x = float(x - self.field_min_x) * self.scale_x
        pos_y = -float(y - self.field_min_y) * self.scale_y
        yaw_deg = 360.0 - float(angle) # deg
        yaw_rad = math.radians(yaw_deg)
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, yaw_rad)
        return pos_x, pos_y, q_x, q_y, q_z, q_w

    def convert_ros_to_toio_coord(self, pos_x, pos_y, q_x, q_y, q_z, q_w):
        x = int((float(pos_x) / self.scale_x) + self.field_min_x)
        y = int(-(pos_y / self.scale_y) + self.field_min_y)
        _, _, yaw_rad = euler_from_quaternion([q_x, q_y, q_z, q_w])
        yaw_deg = math.degrees(yaw_rad)
        angle = int(360.0 - yaw_deg)
        if angle > 360:
            angle = angle - 360
        return x, y, angle

    def make_pose_stamped_msg(self, x, y, q_x, q_y, q_z, q_w):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = 'map'
        pose_stamped_msg.pose.position.x = x
        pose_stamped_msg.pose.position.y = y
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
        transform.child_frame_id = 'center'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = q_x
        transform.transform.rotation.y = q_y
        transform.transform.rotation.z = q_z
        transform.transform.rotation.w = q_w
        return transform

    def monitor_battery_information_callback(self) -> None:
        if not self.is_connected:
            return

        # Skip if a read is already pending (non-blocking pattern)
        if self._battery_read_pending:
            return

        self._battery_read_pending = True
        future = asyncio.run_coroutine_threadsafe(
            self.get_battery_information(),
            self.loop)
        future.add_done_callback(self._handle_battery_result)

    def _handle_battery_result(self, future) -> None:
        """Handle battery read result (non-blocking callback)."""
        self._battery_read_pending = False
        try:
            result = future.result(timeout=0)
            if result:
                battery_level_msg = Float32()
                battery_level_msg.data = float(result)
                self.toio_battery_level_pub.publish(battery_level_msg)
                self.get_logger().debug(f'get_battery_information(), result = {result}')
        except Exception as e:
            self.get_logger().warn(f'Battery read failed: {e}')

    # async function
    async def connect_toio(self) -> None:
        self.cube = ToioCoreCube()
        await self.cube.scan()
        await self.cube.connect()
        self.is_connected = True
        self.get_logger().info('toio is connected.')

    async def motor_control(self, left_motor_speed, right_motor_speed) -> None:
        # Command deduplication with time-based resend:
        # Skip only if same command AND sent less than dedup interval ago
        cmd = (left_motor_speed, right_motor_speed)
        now = time.monotonic()
        if cmd != self._last_motor_cmd or (now - self._last_motor_cmd_time) >= self.motor_dedup_interval:
            await self.cube.api.motor.motor_control(left_motor_speed, right_motor_speed, duration_ms=500)
            self._last_motor_cmd = cmd
            self._last_motor_cmd_time = now

    async def motor_control_target(self, x, y, angle) -> None:
        self.get_logger().debug(f'motor_control_target(): x = {x}, y={y}, angle = {angle}')
        await self.cube.api.motor.motor_control_target(
            timeout=60,
            movement_type=MovementType.Linear,
            speed=Speed(
                max=30, speed_change_type=SpeedChangeType.AccelerationAndDeceleration),
            target=TargetPosition(
                cube_location=CubeLocation(point=Point(x=x, y=y), angle=angle),
                rotation_option=RotationOption.AbsoluteOptimal,
            ),
        )


    async def get_cube_location(self):
        data = await self.cube.api.id_information.read()
        if hasattr(data, 'center') and hasattr(data.center, 'point') and hasattr(data.center, 'angle'):
            pos = (data.center.point.x, data.center.point.y)
            angle = data.center.angle
            self.get_logger().debug(f'pos = {pos}, angle = {angle}')
            return pos, angle
        return None, None

    async def get_battery_information(self):
        data = await self.cube.api.battery.read()
        if hasattr(data, 'battery_level'):
            battery_level = data.battery_level
            self.get_logger().debug(f'battery_level = {battery_level}')
            return battery_level
        return None

def main(args=None):
    rclpy.init(args=args)
    node = ToioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

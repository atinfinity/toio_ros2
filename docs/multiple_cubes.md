# Using multiple cubes

Each cube is handled by its own node instance separated by a ROS namespace.
`toio_multi_bringup.launch.py` brings up one cube per namespace listed in
the `robots` argument (default `toio1,toio2`). For three or more cubes
pass `robots:=toio1,toio2,toio3 cube_ids:=a7D,A8e,B9f`. The example below
brings up two cubes in the namespaces `toio1`
and `toio2`. Specifying `cube_id` of every cube is mandatory here — without
it the two nodes would race for the same cube:

```bash
ros2 launch toio_ros2 toio_multi_bringup.launch.py cube1_id:=a7D cube2_id:=A8e
```

Topics are namespaced (`/toio1/cmd_vel`, `/toio1/toio/pose`, ...) and TF uses
one tree with the shared `map` frame and per-cube prefixed frames
(`toio1/odom`, `toio1/center`, `toio2/odom`, `toio2/center`, ...). RViz2 starts with
[rviz/toio_multi.rviz](../rviz/toio_multi.rviz), which shows the pose and robot
model of both cubes (the "2D Goal Pose" tool sends to `/toio1/goal_pose`,
which moves the cube only with `enable_goal_pose_motion:=true`;
change the topic in the tool properties to command the other cube).

The four node-parameter arguments of `toio_ros2_bringup.launch.py`
(`enable_goal_pose_motion`, `publish_odom`, `stop_on_position_id_missed`,
`stop_on_button`, see [interfaces.md](interfaces.md#launch-arguments)) are
accepted here too and apply to every cube.

For three or more cubes, include `toio_ros2_bringup.launch.py` once per cube
with a unique `namespace` / `cube_id` / `frame_prefix`, following
[launch/toio_multi_bringup.launch.py](../launch/toio_multi_bringup.launch.py).

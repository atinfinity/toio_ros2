# Connecting to a specific cube

By default (`cube_id` and `cube_address` are empty), the node connects to the
nearest cube found by the BLE scan. This is convenient when you have a single
cube, but with other cubes around you may connect to somebody else's cube.

To connect only to your own cube, set the `cube_id` parameter to the identifier
contained in the cube's BLE local name. The name format depends on the cube,
so take the `<cube_id>` part of whichever form your cube advertises:

- `toio Core Cube-<cube_id>` (e.g. `toio Core Cube-C7f` -> `C7f`)
- `toio-<cube_id> (toio Core Cube)` (e.g. `toio-a7D (toio Core Cube)` -> `a7D`)

The name of every cube found by the scan is printed in the node log at startup,
so you can find your `cube_id` there:

```
[INFO] [toio_ros2_node]: found cube: toio-a7D (toio Core Cube) (XXXXXXXX-...)
```

```bash
ros2 run toio_ros2 toio_ros2_node --ros-args -p cube_id:=a7D
```

`cube_id` is matched as a substring of the BLE local name, so use enough
characters to identify one cube: a short `cube_id` may match several cubes,
and the nearest match is used.

`cube_address` can be used instead to specify the BLE address directly, but
note that it is platform dependent: a MAC address on Linux/Windows and a
CoreBluetooth UUID on macOS. `cube_id` takes precedence when both are set.

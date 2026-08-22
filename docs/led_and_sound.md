# LED and sound

The indicator LED and the speaker are driven by topics. See [ROS 2 interfaces](interfaces.md) for the message definitions.

```bash
# light the indicator in red
ros2 topic pub --once /toio/led std_msgs/msg/ColorRGBA "{r: 1.0, g: 0.0, b: 0.0, a: 1.0}"

# play the 'Get1' sound effect
ros2 topic pub --once /toio/sound std_msgs/msg/UInt8 "{data: 6}"
```

Blink red for one second and off for one second, five times. The cube runs the
sequence itself, so it keeps its timing and survives a BLE dropout:

```bash
ros2 topic pub --once /toio/led_pattern toio_msgs/msg/LedPattern "{steps: [{color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}, duration_ms: 1000}, {color: {r: 0.0, g: 0.0, b: 0.0, a: 1.0}, duration_ms: 1000}], repeat: 5}"
```

Play three notes:

```bash
ros2 topic pub --once /toio/melody toio_msgs/msg/Melody "{notes: [{duration_ms: 400, note: 60, volume: 255}, {duration_ms: 400, note: 62, volume: 255}, {duration_ms: 400, note: 64, volume: 255}], repeat: 1}"
```

Light blue for three seconds, without sending an off command afterwards:

```bash
ros2 topic pub --once /toio/led_timed toio_msgs/msg/Led "{color: {r: 0.0, g: 0.0, b: 1.0, a: 1.0}, duration_ms: 3000}"

# turn the indicator off
ros2 topic pub --once /toio/led std_msgs/msg/ColorRGBA "{r: 0.0, g: 0.0, b: 0.0, a: 0.0}"
```

Both topics are relative names, so with `toio_multi_bringup.launch.py` they are
namespaced per cube (`/toio1/toio/led`, `/toio2/toio/led`, ...).
The sound effect IDs are the ones of the
[toio spec](https://toio.github.io/toio-spec/docs/ble_sound) (0:Enter, 1:Selected,
2:Cancel, 3:Cursor, 4:MatIn, 5:MatOut, 6:Get1, 7:Get2, 8:Get3, 9:Effect1, 10:Effect2).

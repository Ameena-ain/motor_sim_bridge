# motor_sim_bridge

A ROS 2 (Humble) Python package that simulates basic motor control firmware.
It subscribes to velocity commands (`cmd_vel`), converts them to left and right
motor RPM, simulates how a real motor responds, and publishes the RPM telemetry.

## Topics

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribes | `cmd_vel` | `geometry_msgs/Twist` | Uses `linear.x` (forward speed, m/s) and `angular.z` (turn rate, rad/s) |
| Publishes | `motor/rpm_telemetry` | `std_msgs/Float32MultiArray` | `data = [left_rpm, right_rpm]`, published at 50 Hz |

## How it works

1. **Velocity to wheel speed (differential drive).** A positive `angular.z`
   is a left turn, so the right wheel runs faster:
   - `v_left  = v - w * L / 2`
   - `v_right = v + w * L / 2`
2. **Wheel speed to RPM.**
   - Angular speed: `omega = v_wheel / r` (rad/s)
   - `RPM = omega * 60 / (2 * pi)`
3. **Motor response (first-order lag).** A real motor cannot change speed
   instantly. A timer runs every `dt` seconds, and each step the motor closes a
   fraction of the gap to its target:
   - `new_rpm = current_rpm + (target_rpm - current_rpm) * (dt / tau)`
   - After one time constant (`tau`) the motor reaches about 63% of the target.
4. **Watchdog.** If no `cmd_vel` message arrives for 0.5 s, both targets are
   set to 0 RPM. The motor then ramps down smoothly instead of running forever
   if the commanding computer stops.

The maths is in `motor_model.py` (plain Python, no ROS). The ROS node is in
`motor_sim_node.py`.

## Build and run

```bash
cd ~/ros2_ws
colcon build --packages-select motor_sim_bridge
source install/setup.bash
ros2 run motor_sim_bridge motor_sim_node
```

Test it in two other terminals (source `/opt/ros/humble/setup.bash` and
`~/ros2_ws/install/setup.bash` in each):

```bash
# watch the telemetry
ros2 topic echo /motor/rpm_telemetry

# drive forward: both motors settle near 95.49 RPM
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# turn in place: left settles near -28.65, right near +28.65
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
```

Stop the publisher with `Ctrl+C` to see the watchdog: after about 0.5 s the
RPM decays smoothly toward 0.

## Parameters

These are set in `motor_sim_node.py`.

| Name | Value | Meaning |
|---|---|---|
| `r` | 0.05 m | Wheel (or propeller) radius |
| `L` | 0.30 m | Distance between the left and right motors |
| `tau` | 0.3 s | Motor time constant: larger means a slower, heavier motor |
| `dt` | 0.02 s | Simulation and publish period (50 Hz) |
| `timeout` | 0.5 s | Watchdog: stop the motors if no command arrives for this long |

## Expected results

| Command | Left RPM | Right RPM |
|---|---|---|
| `v = 0.5`, `w = 0` | 95.49 | 95.49 |
| `v = 0`, `w = 1.0` | -28.65 | +28.65 |

# Go2 integration inventory

Updated October 4, 2026. This records information supplied by the team and interface clues from a script used in another project. The script is pre-existing work and is not part of this repository. Its defaults are leads to verify on the current robot, not confirmed Kinetra interfaces.

## Confirmed by the team

- The Go2 can run with a Jetson Orin. The team can SSH to the Jetson and send control commands from it.
- The robot environment uses ROS 2.
- A front camera is available.

## Suggested by the example script; verify on the current setup

| Interface | Script default or behavior | Live check needed |
| --- | --- | --- |
| Color | `/camera/D455_2/color/image_raw/compressed` or `/camera/D455_2/color/image_raw` | Topic, message type, frame rate, optical frame, image quality at robot height |
| Aligned depth | `/camera/D455_2/aligned_depth_to_color/image_raw` | Topic, encoding, alignment, useful range, invalid pixels |
| Camera calibration | `/camera/D455_2/color/camera_info` | Topic and matching color stream |
| Odometry | `/utlidar/robot_odom`, `odom` to `base_link` | Topic, frames, freshness, drift over a short move |
| Camera extrinsics | TF from camera optical frame to `base_link` | Availability and direction of transform |
| Command receiver | TCP port 6000 on the Jetson, three little-endian float32 body-frame values `(forward_mps, left_mps, yaw_rad_s)` | Receiver implementation, ownership, watchdog, disconnect behavior, arbitration with other controllers |

The example sends velocity commands at 10 Hz and caps them locally. Its defaults include 0.4 m/s maximum forward speed, 0.1 m/s reverse, 0.2 m/s lateral, and 0.8 rad/s yaw. **These are example settings, not approved Kinetra motion limits.** The script's stale-frame and stale-prediction timeout is 0.5 seconds. A sender-side zero command does not establish that the robot-side receiver stops on lost connectivity or that an operator emergency stop works.

## Next live probe

Run these read-only checks on the Jetson in the robot's sourced ROS 2 environment and record the output without credentials or private images:

```bash
printenv ROS_DISTRO
ros2 topic list -t
ros2 node list
ros2 topic info /camera/D455_2/color/image_raw/compressed
ros2 topic info /camera/D455_2/aligned_depth_to_color/image_raw
ros2 topic info /utlidar/robot_odom
```

Then confirm the actual topic names, camera/depth pairing and calibration, odometry frames, and whether the command receiver has its own watchdog and obstacle stop. Identify the operator stop and controller ownership before any motion test. In a cleared bounded area with a spotter, capture robot-height views of a standing dark-clothed distractor and a lying target at several distances, and verify one short commanded move followed by a measured stop. Keep identifiable images outside the public repository unless release is approved.

## Still unknown

- Exact Go2 model and ROS 2 distribution; whether the Jetson is onboard or externally mounted.
- Whether the D455_2 streams in the example are available on this robot, and whether front RGB and depth are aligned.
- Local obstacle avoidance and its authority over velocity commands.
- Receiver watchdog, emergency-stop mechanism, safe perimeter, approved speed and move length.
- What portions of the existing control stack may be published or documented.

The Kinetra adapter should consume stopped RGB-D snapshots and request bounded moves through a separate robot-side controller. The planner must never stream raw velocity commands or override the local stop path.

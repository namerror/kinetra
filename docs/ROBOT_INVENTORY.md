# Go2 integration inventory

**Live survey: October 7, 2026, 17:20–17:30 EDT**, run on the Jetson while it was connected to the Go2.
Everything in "Verified live" below was read from the running system. The survey was strictly read-only:
no velocity command, no `/api/*` request, and no SDK call was issued, so every statement about
*command* behavior and *safety* behavior remains unverified.

Replaces the October 4 version, which recorded second-hand interface clues from a script in another
project. Where that script's defaults differ from its current source, the current source is noted below.

---

## 1. Compute platform (verified live)

| Item | Value |
| --- | --- |
| Board | NVIDIA Jetson AGX Orin Developer Kit |
| L4T / JetPack | R36 rev 4.4 (`/etc/nv_tegra_release`), kernel `5.15.148-tegra`, aarch64 |
| OS | Ubuntu 22.04.5 LTS (jammy) |
| CUDA | 12.6 (`nvcc` 12.6.85), `/usr/local/cuda-12.6` |
| Memory | 61 GiB RAM, 30 GiB swap, 2.0 GiB in use at survey time |
| Storage | `/` 57 GB (71 % used, 16 GB free) · `/mnt/data` NVMe 1.8 TB (14 % used) |
| Power mode | `MODE_30W` (nvpmodel 2) — **not** max performance |
| Python | 3.10.12 |

Installed Python packages relevant to us: `torch` 2.8.0, `cv2` 4.11.0, `numpy` 1.26.4,
`pyrealsense2`, `unitree_sdk2py` 1.0.1 (installed from `/mnt/data/MMC/control/unitree_sdk2_python`).
**`openai` is not installed** — the Nebius client dependency still has to be added.

`/` has only 16 GB free. Keep logs, recordings, and model weights on `/mnt/data`.

## 2. Networking (verified live)

| Interface | Address | Role |
| --- | --- | --- |
| `eno1` | 192.168.123.222/24 | Go2 internal network. The Go2 answers at **192.168.123.161** (0.69 ms RTT). `192.168.123.18` does not exist. |
| `wlP1p1s0` | 192.168.0.76/24 | LAN / internet, default route via 192.168.0.1 |
| `can0`, `can1`, `usb0`, `usb1`, `docker0`, `l4tbr0` | — | all DOWN |

- Outbound internet works: `https://api.studio.nebius.com/v1/models` returns **401** (reachable, unauthenticated).
- Listening: SSH on 22, xrdp on 3389/3350, rpcbind on 111, the ROS 2 daemon on 127.0.0.1:11511, DDS on 7400/7401.
- **Nothing is listening on TCP 6000**, so the pre-existing command bridge is not running.

## 3. ROS 2 environment (verified live)

- **ROS 2 Humble**, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_LOCALHOST_ONLY=0`, domain id 0.
- Sourced overlays: `drivers/unitree_ros2/cyclonedds_ws` (`unitree_go`, `unitree_api`, `unitree_hg`,
  `rmw_cyclonedds_cpp`, CycloneDDS), `drivers/realsense`, `drivers/multi_realsense`, `drivers/vectornav`.
- `CYCLONEDDS_URI` is set but its `<Interfaces>` block is **empty** — it pins no interface. Something to
  understand before adding our own participants, since Go2 traffic arrives on `eno1` while the LAN is on wireless.
- `~/.bashrc` does **not** source ROS (line 125 is commented out); there is an alias `sr`. The working
  environment came from an interactive sourcing step, so a fresh shell or a systemd unit will need it explicitly.
- `ros2 node list` shows only `/camera/D455_1` and `/camera/D455_2`. **Every Go2 topic is published by
  `_CREATED_BY_BARE_DDS_APP_`** — the robot's own native DDS stack, visible to us over `eno1`. There are no
  Unitree ROS nodes to inspect, configure, or restart.

## 4. Cameras (verified live)

**Exactly one RealSense D455 is physically attached**: serial `213622300726`, firmware 5.17.0.10, on USB 3
(`lsusb` bus 002 device 003, `8086:0b5c`).

`ros2 launch multi_setup multi_sensors.launch.py` (running, PID 3790) starts two nodes:

| Node | Serial | `align_depth` | Status |
| --- | --- | --- | --- |
| `/camera/D455_1` | 213622301134 | false | **Camera not present.** Node is alive but publishes no image topics. |
| `/camera/D455_2` | 213622300726 | **true** | Working. The only usable camera. |

Both are configured `640,480,30` for depth and color, `pointcloud.enable=false`.

### Confirmed D455_2 topics and measured rates

| Topic | Type | Measured | Details |
| --- | --- | --- | --- |
| `/camera/D455_2/color/image_raw` | `sensor_msgs/Image` | **30.1 Hz** | `bgr8`, 640×480, frame `D455_2_color_optical_frame` |
| `/camera/D455_2/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | **30.1 Hz** | |
| `/camera/D455_2/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | **30.0 Hz** | `16UC1` (millimetres), 640×480, frame `D455_2_color_optical_frame` |
| `/camera/D455_2/color/camera_info` | `sensor_msgs/CameraInfo` | — | see intrinsics below |
| `/camera/D455_2/depth/image_rect_raw` | `sensor_msgs/Image` | — | unaligned depth |
| `/camera/D455_2/extrinsics/depth_to_color` | `realsense2_camera_msgs/Extrinsics` | — | |

Aligned depth carries the **same `frame_id` as color**, so a stopped RGB-D snapshot needs no reprojection.

**Color intrinsics** (640×480, `plumb_bob`): `fx` 387.552, `fy` 387.160, `cx` 327.805, `cy` 240.058;
`d` = [-0.05529, 0.06240, -0.001064, 0.001175, -0.01999].

**QoS on all image topics: RELIABLE, KEEP_LAST depth 1, VOLATILE** (`camera_info` uses depth 10). A subscriber
must match RELIABLE or it will get nothing.

**Depth is live and usable.** One sampled aligned frame: 84.3 % non-zero pixels, min 0.64 m, median 4.04 m,
max 62.1 m. Values far past the D455's useful range appear, so clamp to a sane maximum before reasoning on depth.

### Camera-to-body extrinsics are missing

`/tf_static` contains **only intra-camera transforms** (`D455_2_link` → depth/color frames; the color frame sits
at y = −0.0589 m from `D455_2_link`). There is **no transform from any camera frame to `base_link`**, and no
robot URDF or state publisher is running. The camera's mounting pose and height on the Go2 must be measured by
hand and published by us before any depth observation can be placed in the body or odometry frame.

### Alternative view

`/frontvideostream` (`unitree_go/msg/Go2FrontVideoData`) exposes the Go2's own built-in front camera. Not yet
sampled; a fallback or secondary view.

## 5. Robot state (verified live, passive)

| Topic | Type | Measured rate |
| --- | --- | --- |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | **500 Hz** |
| `/lowstate` | `unitree_go/msg/LowState` | **500 Hz** |
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | **245 Hz** |
| `/utlidar/robot_pose` | `geometry_msgs/msg/PoseStamped` | not sampled |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | not sampled — lets us observe the remote |

- **Odometry**: `/utlidar/robot_odom` has `frame_id: odom`, `child_frame_id: base_link`, and a
  **covariance that is all zeros** (no uncertainty is reported). Drift over a short move has not been measured.
- `/sportmodestate` at survey time: `error_code 0`, `mode 7`, `gait_type 1`, `body_height 0.32`,
  `foot_raise_height 0.09`, IMU temperature 79 °C, rpy pitch −0.097 rad.
  `foot_force [9, 16, 17, 7]` — the legs were essentially unloaded, so the robot was not standing on its feet.
  Also carries `position`, `velocity`, `yaw_speed`, `foot_position_body`, `foot_speed_body`.
- `range_obstacle` in `/sportmodestate` read **[2.0, 2.0, 2.0, 2.0]** — saturated or placeholder. Do not treat it
  as a working proximity sensor until it has been seen to change with a real obstacle.
- **Battery** (`/lowstate` → `bms_state`): **soc 46 %**, current −656 mA (discharging), cycle count 5,
  `power_v` 28.38 V, `power_a` 0.103 A, pack NTCs 36 °C / 32 °C. All 12 motors report 31–33 °C.

## 6. The L1 lidar is not producing data — blocking finding

`/utlidar/lidar_state` reports:

```
error_state: 6          cloud_frequency: 0.0    cloud_size: 0
cloud_scan_num: 0       imu_frequency: 0.0      software_version: 1.0.0.39
sys_rotation_speed: 0.0219
```

`ros2 topic hz` received **zero messages** on `/utlidar/cloud_deskewed`, `/utlidar/height_map_array`, and
`/utlidar/imu`. The topics are advertised but silent.

Consequences, which change the plan:

- No lidar point cloud, no height map, no voxel map. The Go2's lidar-based terrain and obstacle sensing is
  unavailable as things stand.
- `/uslam/*` (odom, cloud_map, localization, global_path) and `/lio_sam_ros2/mapping/odometry` are advertised
  but must be assumed non-functional until proven otherwise — they consume the lidar.
- The Unitree topological-graph interface (`/qt_add_node`, `/qt_add_edge`, `/qt_command`,
  `/query_result_node`, `/query_result_edge`, `unitree_interfaces/msg/QtNode|QtEdge`) is interesting for place
  memory, but it probably sits on top of `uslam` and therefore on the lidar. Do not plan around it yet.
- Leg odometry on `/utlidar/robot_odom` **is** still publishing at 245 Hz, so short-range odometry survives.

Next step: find out whether the lidar is switched off (`/utlidar/switch` exists, one publisher and one
subscriber), unpowered, miscabled, or faulty — and decode `error_state: 6`. A D455-only obstacle path is the
fallback, and it has a narrower field of view than the plan assumed.

## 7. Clock skew between the robot and the Jetson — blocking finding

The Jetson is NTP-synchronised (survey time 2026-10-07 21:22 UTC). Message stamps are not on a common timebase:

| Source | Example `header.stamp` | Relative to Jetson wall clock |
| --- | --- | --- |
| Jetson wall clock | 1791408132 | — |
| `/camera/D455_2/color/image_raw` | 1791408137 | matches |
| `/utlidar/robot_odom` | 1791405506 | **≈ 2626 s (44 min) behind** |
| `/sportmodestate` | 1791405487 | **≈ 2626 s behind** |

Camera stamps come from the Jetson; Go2-origin stamps come from the robot's own clock. **Do not pair a snapshot
with an odometry pose by header stamp.**

The offset is also not constant: measured in 30 s buckets it moved **+27 ms over 5 minutes, about 90 ppm or
+0.32 s per hour**, so it must be re-measured continuously and never cached. This is implemented and explained
in [SNAPSHOTS.md](SNAPSHOTS.md); the measured numbers and residual risks are in
[OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) §3.

## 8. Command path — interface known, behavior unverified

There is **no `/cmd_vel`**. Motion reaches the Go2 one of two ways:

1. `unitree_api/msg/Request` on `/api/sport/request` — the robot's native API. It already has **12 publishers**,
   all its own internal services, so we would be one voice among many with no visible arbitration.
2. `unitree_sdk2py` `SportClient` over DDS on `eno1` — what the pre-existing bridge uses.

A pre-existing lab bridge lives at `/home/jetson/repos/robot_control` on this Jetson. **It is outside this
repository and must not be copied into it** (see the public-release boundary in `HACKATHON.md`); only its
interface is recorded here. Its documented shape:

```
planner ──TCP 127.0.0.1:6000, struct "<3f" (vx, vy, wz)──► bridge ──unitree_sdk2py DDS──► Go2 sport mode
```

The bridge clamps, acceleration-limits, rejects non-finite values, and calls `StopMove()` when a command goes
stale, the client disconnects, or it exits. Current source defaults:

| Parameter | Default |
| --- | --- |
| rate | 20 Hz |
| stale timeout | 0.5 s |
| max forward | 0.4 m/s |
| max backward | 0.1 m/s |
| max lateral | **0.05 m/s** |
| max yaw | **0.6 rad/s** |
| acceleration | 0.8 / 0.5 / 1.5 per second (vx / vy / wz) |
| deadband | 0.02 |

Two corrections to the October 4 record: lateral is 0.05 m/s (not 0.2) and yaw is 0.6 rad/s (not 0.8).
**These are still that project's settings, not approved Kinetra motion limits.**

Critically, its own README states that the real `unitree_sdk2py` calls and the RealSense capture path were
**never tested on hardware** — the only testing was in `--dry-run` on a laptop. So the SDK motion path on this
robot is untested by anyone, and the first move will be its first real test.

Also unverified, and all of it gating any motion test:

- Whether `SportClient.Move` is subject to the Go2's own obstacle avoidance. `/api/obstacles_avoid/{request,response}`
  exists, but its enable state cannot be read passively, and we did not publish a request.
- Current motion mode. `/api/motion_switcher/request` exists; querying it means publishing to the robot, which
  this survey deliberately did not do. `/sportmodestate` reported `mode 7`, meaning not yet decoded.
- Whether the robot-side receiver has its own watchdog independent of the bridge's sender-side stop.
- The operator emergency stop. The bridge docs say the Unitree handheld remote overrides the SDK; this has not
  been demonstrated on this robot. `/wirelesscontroller` lets us at least observe remote input.
- Arbitration between us and the 12 existing `/api/sport/request` publishers.

## 9. Robot configuration read passively from `/multiplestate`

`/multiplestate` (`std_msgs/String`, 1 Hz) carries a JSON configuration dump, readable without publishing
anything to the robot:

```json
{"bodyHeight": 0.32, "footRaiseHeight": 0.09, "speedLevel": 0,
 "obstaclesAvoidSwitch": true, "uwbSwitch": true, "brightness": 0, "volume": 0}
```

**`obstaclesAvoidSwitch` is `true` while the lidar that feeds it is silent (§6).** Treat this as a hazard
rather than a reassurance: the flag suggests something is watching for obstacles when its primary sensor
produces nothing. Whether the feature gates `SportClient.Move` at all is still unverified (§8).

### Camera configuration, read from the node's parameters

`emitter_enabled 1`, auto-exposure on for both streams (`rgb_camera.exposure 166`,
`depth_module.exposure 8500`, `depth_module.gain 16`), both profiles `640,480,30`, `align_depth.enable true`.
`depth_module.depth_units` is **unset**, so librealsense's 1 mm default applies.

### Robot identity is not self-reported

`/lowstate` gives `sn: [0, 0]` and `version: [0, 0]`. The only version strings available anywhere were the
lidar's (`1.0.0.39`) and the BMS's (1.16), so the exact Go2 variant and firmware are not established from the
robot itself.

## 10. Other hardware and topics seen

- **VectorNav IMU**: the `vectornav` workspace is built and an FT232 USB-serial adapter is present
  (`lsusb` bus 001 device 004), but no vectornav node is running and no `/vectornav` topic exists.
- Go2 extras, unused so far: `/audiohub`, `/gas_sensor`, `/uwbstate`, `/gpt`, `/videohub`, `/arm_Command`,
  `/pctoimage_local`, `/webrtcreq`.
- `/dev/video0` … `/dev/video5` exist (the single D455's V4L2 nodes), so a camera can also be opened without ROS.

## 11. Other repositories present on this Jetson

`/home/jetson/repos` holds `drivers/`, `robot_control/`, `FollowAndAvoid/`, `MMC/`, and `kinetra/`. All but
`drivers/` and `kinetra/` are **pre-existing lab work outside the public-release boundary**: do not copy their
code, datasets, checkpoints, or recorded scenes into this repository. Document interfaces only, as above.

## 12. Open items

The full register of limitations and unknowns now lives in
[OPEN_QUESTIONS.md](OPEN_QUESTIONS.md), so there is one authoritative list rather than several drifting ones.
The four that block the most other work:

1. **Lidar** — revive it or commit to D455-only, time-boxed (§6 here, §2.1 there).
2. **Camera-to-body transform** — nothing relates the camera to `base_link`, which blocks place memory (§4).
3. **Safety before any motion** — operator stop, remote override, `mode 7`, and Kinetra's own limits (§8).
4. **Nebius plus target visibility** — the project's premise, and entirely unverified
   ([OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) §5).

The Kinetra adapter should still consume stopped RGB-D snapshots and request bounded moves through a separate
robot-side controller. The planner must never stream raw velocity commands or override the local stop path.

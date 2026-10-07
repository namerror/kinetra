#!/usr/bin/env bash
# Read-only survey of the Jetson + Go2 setup. Sends no command to the robot:
# it only lists, echoes, and times topics, and reads local system state.
#
#   source /opt/ros/humble/setup.bash   # plus the driver overlays
#   tools/live_survey.sh | tee /mnt/data/kinetra_survey_$(date +%F_%H%M).txt
#
# Results of the October 7, 2026 run are summarised in docs/ROBOT_INVENTORY.md.

set -u
CAM=${CAM:-D455_2}          # the camera namespace that is actually streaming
HZ_WINDOW=${HZ_WINDOW:-12}  # seconds per rate measurement

hdr() { printf '\n========== %s ==========\n' "$1"; }

hdr "when"
date -Is; echo "epoch: $(date +%s)"
timedatectl 2>/dev/null | sed -n '1,8p'

hdr "platform"
cat /proc/device-tree/model 2>/dev/null; echo
cat /etc/nv_tegra_release 2>/dev/null
uname -a
nvpmodel -q 2>/dev/null | head -5
free -h
df -h / /mnt/data 2>/dev/null
nvcc --version 2>/dev/null | tail -2

hdr "python deps"
python3 --version
for p in unitree_sdk2py pyrealsense2 torch cv2 numpy openai; do
  python3 -c "import $p; print('$p', getattr($p,'__version__','present'))" 2>/dev/null || echo "$p MISSING"
done

hdr "network"
ip -brief addr
ip route
ip neigh show dev eno1 2>/dev/null
for h in 192.168.123.161 192.168.123.18; do
  ping -c1 -W1 "$h" >/dev/null 2>&1 && echo "$h UP" || echo "$h unreachable"
done
echo -n "nebius endpoint HTTP status: "
curl -s -o /dev/null -w '%{http_code}\n' --max-time 8 https://api.studio.nebius.com/v1/models
echo "listeners:"; ss -tlnp 2>/dev/null | grep -E ':(6000|11511|22) ' || true

hdr "ros environment"
printenv | grep -E '^(ROS_|RMW_|CYCLONEDDS_URI)' | sort

hdr "cameras"
rs-enumerate-devices -s 2>/dev/null || echo "rs-enumerate-devices unavailable"
lsusb | grep -i 8086 || echo "no Intel USB device"
ls /dev/video* 2>/dev/null

hdr "ros nodes"
timeout 30 ros2 node list 2>&1

hdr "ros topics"
timeout 30 ros2 topic list -t 2>&1

hdr "camera stream details"
for t in "/camera/$CAM/color/image_raw" "/camera/$CAM/aligned_depth_to_color/image_raw"; do
  echo "--- $t"
  timeout 20 ros2 topic info -v "$t" 2>&1 | sed -n '1,20p'
  for f in header encoding width height; do
    echo -n "  $f: "; timeout 15 ros2 topic echo --once --field "$f" "$t" 2>/dev/null | tr '\n' ' '; echo
  done
done
echo "--- camera_info"
timeout 15 ros2 topic echo --once "/camera/$CAM/color/camera_info" 2>&1 | sed -n '1,30p'

hdr "tf_static (expect camera-internal frames only until we publish a body transform)"
timeout 15 ros2 topic echo --once /tf_static 2>&1 | grep -E 'frame_id|child_frame_id'

hdr "robot state"
timeout 15 ros2 topic echo --once /sportmodestate 2>&1 | sed -n '1,50p'
echo "--- odometry"
timeout 15 ros2 topic echo --once /utlidar/robot_odom 2>&1 | sed -n '1,20p'
echo "--- battery and power"
timeout 15 ros2 topic echo --once /lowstate 2>&1 | grep -A12 'bms_state' | head -16
timeout 15 ros2 topic echo --once /lowstate 2>&1 | grep -E 'power_v|power_a|temperature_ntc|^tick'

hdr "lidar health (error_state != 0 or cloud_frequency 0.0 means no point clouds)"
timeout 15 ros2 topic echo --once /utlidar/lidar_state 2>&1 | sed -n '1,20p'

hdr "measured rates (${HZ_WINDOW}s each; 'Terminated' with no numbers means the topic is silent)"
for t in "/camera/$CAM/color/image_raw/compressed" "/camera/$CAM/aligned_depth_to_color/image_raw" \
         /utlidar/robot_odom /sportmodestate /lowstate \
         /utlidar/cloud_deskewed /utlidar/height_map_array /utlidar/imu; do
  echo "--- $t"
  timeout "$HZ_WINDOW" ros2 topic hz "$t" 2>&1 | head -4
done

hdr "clock skew check"
echo "jetson epoch: $(date +%s)"
# Image and Odometry carry header.stamp; SportModeState carries a bare float `stamp`.
for t in "/camera/$CAM/color/image_raw" /utlidar/robot_odom; do
  echo -n "$t header.stamp.sec: "
  timeout 15 ros2 topic echo --once --field header.stamp.sec "$t" 2>/dev/null | head -1
done
echo -n "/sportmodestate stamp: "
timeout 15 ros2 topic echo --once --field stamp /sportmodestate 2>/dev/null | head -1
echo "Camera stamps come from the Jetson; Go2 stamps come from the robot's own clock."
echo "A large gap means snapshots and poses must NOT be paired by header stamp."

hdr "done"

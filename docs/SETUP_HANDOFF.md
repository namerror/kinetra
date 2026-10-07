# Next-agent setup handoff

## Status

**A live Jetson/Go2 survey was completed on October 7, 2026.** Step 1 below is largely done and its results
are in [ROBOT_INVENTORY.md](ROBOT_INVENTORY.md) — read that before anything else. Two findings change the
plan and are not yet resolved:

- **The L1 lidar produces no data** (`error_state: 6`, zero point clouds). Height map, voxel map, `uslam`, and
  the Unitree topological-graph interface should all be assumed unavailable. Leg odometry still works.
- **Go2 message stamps run ~44 min behind the Jetson clock.** Snapshots and poses cannot be paired by header
  stamp as written.

Every known limitation and unknown is now catalogued in
[OPEN_QUESTIONS.md](OPEN_QUESTIONS.md) — read that before relying on any part of the system. Also outstanding
from the survey: only one of the two D455 cameras is physically attached, no camera-to-body
transform is published anywhere, nothing about the motion or safety path has been verified on hardware (the
survey sent no commands, and the pre-existing bridge's own README says its SDK path was never hardware-tested),
and Nebius has no credentials configured and no client installed.

## Start here

This repository is still a planning scaffold: `src/` contains only `.gitkeep` files, and there is no working
package, environment specification, robot adapter, API client, or test suite yet. Read `PROJECT_SPEC.md`,
`ARCHITECTURE.md`, `EVALUATION.md`, and `HACKATHON.md` first.

The platform is now characterised: Jetson AGX Orin (JetPack R36.4.4, CUDA 12.6, ROS 2 Humble, CycloneDDS) on
the Go2's 192.168.123.x network, with one working D455 giving 30 Hz aligned RGB-D at 640x480 and leg odometry
at 245 Hz. Topic names, types, QoS, rates, and intrinsics are recorded in
[the robot inventory](ROBOT_INVENTORY.md). Re-run `tools/live_survey.sh` to refresh that picture; it only reads.

## Ordered first work

1. **Close out the inventory.** Sensing and state are verified. Still open: revive the lidar or commit to a
   D455-only obstacle path, resolve the clock skew, measure and publish the camera-to-`base_link` transform,
   and establish the command receiver's watchdog, obstacle stopping, and operator emergency stop. Record only
   releasable interface details in [ROBOT_INVENTORY.md](ROBOT_INVENTORY.md).
2. **Test physical feasibility in a bounded area:** collect robot-height snapshots of a standing dark-clothed distractor and a lying target at multiple distances; measure whether the robot can execute and stop one safe short move. Do not assume a person detector trained on standing pedestrians recognizes a lying person.
3. **Verify Nebius access:** list models for the team's account, make an actual Nemotron text inference call, test a candidate vision model with one real snapshot, and record latency, output quality, response format, and model identity. Avoid committing credentials or identifiable private images.
4. **Choose the first technical environment** based on the available Go2 stack. Establish a public mock robot and recorded-scene path before coupling model decisions to hardware.
5. **Build the smallest physical loop:** stop at a viewpoint, capture an observation, ask the NVIDIA planner for one bounded action, validate it, execute one safe move, and log the result.
6. **Add place memory and verification**, then the fixed-order baseline and matched physical trials. Keep demo preparation ahead of the October 30 deadline.

## Open decisions to document after the inventory

- Exact camera and depth configuration and safe viewing distance for a lying person.
- Whether a local navigation controller already provides bounded moves and obstacle stopping without SLAM.
- How visual place recognition will detect revisits despite odometry drift.
- Which hosted vision model actually supports the team's image requests and distinguishes target attributes.
- Whether Cosmos-Reason2 on Nebius AI Cloud improves the demo enough to justify deployment.
- Exact safety perimeter, maximum speed/move length, and operator-stop mechanism for physical trials.

## Public-release boundary

The public-release boundary is recorded in `HACKATHON.md`: keep unpublished data-generation code, datasets, private checkpoints, lab-owned code, and publication-sensitive experiments outside this repository. Connect any private follow/navigation policy through a documented adapter only if release is approved. This project should stand on its own through mock or public components for reviewers.

## Submission preparation

Keep the README runnable as implementation arrives, clearly identify which work was added during the hackathon, record Nebius/NVIDIA usage and feedback, check third-party licenses, and prepare the public repository and YouTube demo. Recheck the [official overview](https://nebiusglobalaihackathon.devpost.com/) and [rules](https://nebiusglobalaihackathon.devpost.com/rules) before submission.

# Next-agent setup handoff

## Start here

This repository is a planning scaffold. There is no working package, environment specification, robot adapter, API client, or test suite yet. Read `PROJECT_SPEC.md`, `ARCHITECTURE.md`, `EVALUATION.md`, and `HACKATHON.md` first.

The team has confirmed ROS 2, Jetson Orin access by SSH, and a front camera. An example from another project suggests RGB-D, odometry, and a TCP velocity receiver, but its topic names and safety behavior still need live verification. See [the robot inventory](ROBOT_INVENTORY.md) before implementing the robot adapter.

## Ordered first work

1. **Finish the live Go2 inventory:** verify the camera/depth/odometry topics and frames, command receiver, obstacle stopping, emergency stop, and what code may be published. Record only releasable interface details in [ROBOT_INVENTORY.md](ROBOT_INVENTORY.md).
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

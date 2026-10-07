# Kinetra Search

Text-guided, mapless person search on a Unitree Go2 for the [Nebius × NVIDIA Global AI Hackathon](https://nebiusglobalaihackathon.devpost.com/) Physical AI track.

> **Project status (October 2026): planning scaffold.** The search system, robot integration, model calls, and evaluation are not implemented. This repository currently documents the agreed project idea and the work needed to validate it.

## The mission

An operator describes a person, for example: **“Find the person wearing black who is lying on the floor.”** The Go2 searches an unfamiliar but bounded indoor area, checks plausible candidates from useful viewpoints, and reports either a verified match or **“not found within the searched area.”** A standing person in black is a deliberate distractor in the headline demo.

The Go2 will not rely on a prebuilt map or geometric SLAM. It will use short-range odometry, local obstacle sensing, and an explicit memory of visited viewpoints, connections, observations, and rejected candidates. A Nebius-hosted NVIDIA model will make consequential search decisions from the text goal and that memory. Vision reasoning is called when the robot stops to inspect a scene or verify a candidate; local control handles continuous motion.

## Planned system

```text
text goal + RGB-D snapshots + short-range odometry
                      |
                      v
       scene observations and place memory
                      |
                      v
       NVIDIA search planner via Nebius
                      |
           bounded next-view action
                      |
                      v
       local obstacle avoidance + Go2
                      |
                      +----> new observation / verification / result
```

The initial model candidate is **NVIDIA Nemotron 3.5 Lightning through Nebius Token Factory** for goal interpretation, place-memory reasoning, and next-view decisions. The snapshot vision model is undecided. Nebius's current public workbench guide lists MiniMax M3 as a vision default; an NVIDIA visual model such as Cosmos-Reason2-2B on Nebius AI Cloud is an option if access, deployment, and camera-image tests justify it. These are **candidates, not verified working integrations**. See [architecture and model decisions](docs/ARCHITECTURE.md).

## Demonstration and evidence

The first physical mission uses three to five viewing locations in a bounded, rearrangeable indoor area. The robot should reject a standing distractor, inspect an initially hidden region, and verify the lying target. A separate absent-target mission should end with a bounded, accurate report. The comparison baseline uses the same vision and movement components but visits viewpoints in a fixed order. See the [project specification](docs/PROJECT_SPEC.md) and [evaluation plan](docs/EVALUATION.md).

## Repository guide

| Path | Purpose |
| --- | --- |
| [docs/PROJECT_SPEC.md](docs/PROJECT_SPEC.md) | Mission, search rules, safety limits, and acceptance criteria |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Planned components, information flow, and model choices |
| [docs/EVALUATION.md](docs/EVALUATION.md) | Baselines, trial design, and metrics |
| [docs/SETUP_HANDOFF.md](docs/SETUP_HANDOFF.md) | Ordered starting tasks and unresolved environment questions |
| [docs/ROBOT_INVENTORY.md](docs/ROBOT_INVENTORY.md) | **Verified** Jetson/Go2 platform survey: topics, types, QoS, rates, intrinsics, and what is still broken or unverified |
| [docs/OPEN_QUESTIONS.md](docs/OPEN_QUESTIONS.md) | **Authoritative register of limitations, unknowns, and risks** — read before relying on anything |
| [docs/SNAPSHOTS.md](docs/SNAPSHOTS.md) | The RGB-D snapshot format, how to capture one, and why frames and poses are paired the way they are |
| [docs/HACKATHON.md](docs/HACKATHON.md) | Current hackathon requirements, judging, submission checklist, and public-release boundaries |
| `src/kinetra/perception/` | Snapshot capture: `capture` (ROS 2 node, subscribe-only), `snapshot` and `clock` (no ROS dependency) |
| `src/kinetra/` | Remaining packages reserved for public, hackathon-specific implementation |
| `tools/live_survey.sh` | Read-only survey of the Jetson and Go2; sends no command to the robot |
| `tests/`, `evaluation/`, `demo/` | Reserved for tests, trial artifacts, and demo material |

The first implemented piece is RGB-D snapshot capture (`docs/SNAPSHOTS.md`). It runs against the live camera
and its off-robot half has unit tests that need neither ROS nor hardware:

```bash
PYTHONPATH=src python3 -m pytest tests/ -q                                   # no robot needed
PYTHONPATH=src:$PYTHONPATH python3 -m kinetra.perception.capture --dry-run   # on the Jetson
```

There is no end-to-end application yet. The next agent should begin with
[SETUP_HANDOFF.md](docs/SETUP_HANDOFF.md) and [ROBOT_INVENTORY.md](docs/ROBOT_INVENTORY.md) — the platform has
now been surveyed live, but the lidar is dead, robot and Jetson clocks disagree by ~44 minutes, no
camera-to-body transform exists, the motion and safety path is untested on hardware, and Nebius access is
unconfigured. Do not present the planned interfaces or model names as implemented.

## Hackathon constraints

- Submission deadline: **October 30, 2026, 10:00 a.m. Pacific**. Reserve time before that for repeated robot trials, the public repository, and the video.
- The project must make a real runtime call to Nebius Token Factory or run on Nebius AI Cloud, and use an NVIDIA open-source model for a genuine function.
- The public YouTube demo must be no longer than three minutes and include at least one minute of actual physical hardware operation.
- Pre-existing lab policies and any unpublished data-generation code, datasets, checkpoints, or lab-owned stack must stay outside this public repository. Connect approved external components through documented adapters.

The [current official rules](https://nebiusglobalaihackathon.devpost.com/rules) take precedence over this README and [the hackathon reference](docs/HACKATHON.md) if they change.

## License

The public scaffold is licensed under [MIT](LICENSE). Third-party models, policies, robot SDKs, datasets, and lab components retain their own licenses and release requirements.

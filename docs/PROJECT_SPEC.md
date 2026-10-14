# Project specification — text-guided person search

## Status and objective

This is the agreed design direction, not a claim that the system works. The objective is to find a person matching a compositional text description in an unfamiliar, bounded indoor area using a real Unitree Go2. The first goal is: **“Find the person wearing black who is lying on the floor.”**

The robot must reject a standing person in black, gather another view when an attribute is unclear, and distinguish **verified found** from **not found within the searched area**. It should show the evidence behind its report.

## MVP environment and mission

- One level, no stairs or closed doors, with a clear operator-controlled safety perimeter.
- Three to five distinct reachable viewpoints, such as a central space and partitioned alcoves. Layout and target placement can change between trials; no prebuilt map is supplied.
- A person lies safely in a location not visible from the start. A standing person dressed similarly is a distractor. An absent-target variant is required.
- The robot may make short, bounded moves between views. It does not need room-scale metric mapping, return-to-home navigation, or continuous pursuit.
- An operator remains able to stop the robot immediately. Physical trials require a spotter and cleared movement area.

## Search protocol

1. Parse the goal into observable attributes. Keep the original text and an explicit verification checklist.
2. At a safe stop, capture one or more useful RGB-D views. Record visible openings, obstacles, people, hidden areas, and uncertainty.
3. Update a structured memory of places, their visual landmarks, connections, visit counts, inspected directions, and candidate outcomes. Never replace an observation with an unsupported model assertion.
4. Rank reachable uninspected viewpoints for the goal. Choose one bounded local movement or request a better view of the current scene.
5. If a plausible candidate appears, stop. Verify **person**, **dark clothing**, and **lying posture** separately. If evidence is insufficient, move to a safe second viewpoint or mark the candidate uncertain.
6. Once all reachable viewpoints have been inspected, revisit only viewpoints with obscured or uncertain regions.
7. End with a verified match, an explicit safety/operator stop, or “not found within the searched area” when the search budget is exhausted.

Initial design limits are **two inspection passes, no more than six viewpoint visits, and no more than three minutes per mission**. One evaluation trial is a complete mission with a specified target placement, distractor condition, and outcome. These limits are provisional until measured on the Go2.

## Local safety contract

The model can request only a bounded action such as inspect a reachable direction, take a closer safe view, verify a candidate, wait, or finish. A deterministic layer rejects actions that are invalid, stale, outside the perimeter, blocked, or inconsistent with safety state. Local sensing and control handle obstacles. Invalid output, lost connectivity, expired action, or uncertain clearance defaults to stop. No natural-language output goes directly to motor control.

### Approved motion limits

These are **normative**. `src/kinetra/robot/limits.py` is their executable copy, and
`tests/test_limits.py` fails if the two diverge. They are bring-up limits, chosen to sit *at* the first test
values so that no stage can be made more aggressive without editing both this table and that file. They are
not the pre-existing lab bridge's defaults, which are another project's settings
(`ROBOT_INVENTORY.md` section 8).

| Quantity | Limit |
| --- | --- |
| Forward speed | 0.15 m/s |
| Backward speed | **refused** |
| Lateral speed | **refused** |
| Yaw rate | 0.30 rad/s |
| Commanded window per action | 2.0 s, both acceleration ramps included |
| Commanded travel per action | 0.35 m |
| Commanded rotation per action | 0.70 rad |
| Acceleration | 0.3 m/s², 0.6 rad/s² |
| Command rate | 20 Hz, re-issued every tick |
| State staleness | 0.30 s in motion, 0.10 s before arming |
| Terminal state on abort | `StopMove()`; the robot is left standing |

Reverse and lateral are refused structurally — the ceiling is zero *and* no command-line flag exists to raise
it. The robot has no rearward sensing, lateral is the least-exercised axis in Unitree's own stack, and
turn-then-walk composes to every pose either would reach, so there is no safety case for carrying the risk.
Either gets its own bring-up stage, with its own spotter, if it is ever actually needed.

A requested value above a ceiling is **refused, not clamped**: clamping would let an operator ask for
0.4 m/s, receive 0.15, and read a success message.

### Operator contract for a physical trial

- One stage per invocation, chosen explicitly. There is no stage sequencer and no default stage.
- The script prints the stage's purpose and its exact tick-by-tick command schedule before anything is sent.
- A live run requires the operator to type the stage name verbatim. There is no bypass flag, and a non-TTY
  stdin is refused.
- A spotter holds the handheld remote throughout, with a thumb on the stop.
- Clearance is measured and confirmed, not assumed: 1.5 m radius for an in-place rotation, since rotation
  drifts, and for forward motion a distance computed from the stop distance the yaw stages measured.
- The operator stands the robot up and sits it down with the remote. The script never commands a posture
  change; its entire command surface is `Move`, `StopMove`, `CheckMode` and `GetServerApiVersion`.
- **While `OPEN_QUESTIONS.md` 1.3 is open, nothing stops this robot but the operator.** Obstacle avoidance
  reports enabled while the lidar feeding it is dead.

Procedure, stage rationale and how to read a run back are in [MOTION.md](MOTION.md).

## What makes the model contribution substantive

The model should choose an informative next view from the goal, scene evidence, prior failures, and place memory; request a better angle for a specific missing attribute; and explain why it accepted or rejected a candidate using visible evidence. Merely converting “no person visible” to “turn right” is insufficient. The fixed-order baseline in `EVALUATION.md` tests whether model-guided choices improve the mission.

## MVP acceptance criteria

- A real Go2 safely visits distinct viewpoints in a rearranged bounded area without a prebuilt map.
- An NVIDIA model makes runtime search decisions through Nebius and those decisions change physical behavior.
- The robot rejects a plausible standing distractor and verifies a matching lying person from visual evidence.
- An absent-target trial ends with an accurate bounded report.
- Every trial records goal, observations, place memory, model decision, executed action, and outcome.
- The public repository contains only releasable hackathon work and documents any separately installed policy or SDK.

## Deferred scope

Arbitrary buildings, stairs, closed-door manipulation, reliable emergency response, persistent maps across missions, training a new foundation model, and replacement of low-level locomotion are outside the one-month MVP.

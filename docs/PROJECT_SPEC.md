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

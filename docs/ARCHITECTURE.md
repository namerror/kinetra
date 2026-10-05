# Architecture and model decisions

## Boundaries

```text
operator goal
    |
    v
goal attributes + mission budget
    |
    v
RGB-D snapshots -> visual observations -> explicit place/candidate memory
                                          |
                                          v
                              NVIDIA planner on Nebius
                                          |
                              validated bounded action
                                          |
                                          v
                  local odometry + obstacle sensing + Go2
                                          |
                                result / next snapshot
```

The planner reasons at **events**: a new viewpoint, blocked action, candidate sighting, verification result, or exhausted budget. It is not in the continuous motor-control loop. A robot-side supervisor enforces limits and manages timeouts.

## Explicit memory

Store a small graph of viewpoints, not a long unstructured conversation. Each place should have a stable ID, representative images or landmark descriptions, known connections, inspected directions, visit count, observations, and unresolved regions. Candidate records should include which target attributes are supported, contradicted, or unseen and the images supporting each judgment. Odometry can estimate relative motion between nearby viewpoints; visual place recognition is needed to notice revisits. This is a topological memory, not geometric SLAM.

The model may propose updates to the memory, but robot events and observation provenance must be retained. An unobserved passage, candidate identity, or target location remains a hypothesis.

## Proposed components and directory ownership

| Directory | Planned responsibility |
| --- | --- |
| `src/kinetra/robot/` | Public Go2 adapter, bounded motion interface, stop/status; no lab implementation |
| `src/kinetra/perception/` | Snapshot capture and candidate/scene observation adapters |
| `src/kinetra/memory/` | Place graph, visit history, candidate evidence, revisit detection |
| `src/kinetra/agent/` | Goal parsing, Nebius client, planner, decision validation, mission supervisor |
| `src/kinetra/navigation/` | Short-range action execution and local obstacle checks; no assumed global SLAM |
| `evaluation/` | Trial definitions, recorded runs, metrics, and baseline comparison |
| `demo/` | Public demo plan and non-sensitive media notes |

Interfaces and schemas should be finalized only after inspecting the available Go2 stack. Keep vendor and private-policy adapters separate from the public core.

## Model shortlist as of October 2026

| Role | Candidate | Decision status |
| --- | --- | --- |
| Goal interpretation, place-memory reasoning, next-view selection | `nvidia/Nemotron-3_5-Lightning` via Nebius Token Factory | Preferred first integration. Confirm account access, structured-output behavior, latency, and decision quality with real state examples. |
| Snapshot interpretation and target-attribute verification | Available hosted VLM; Nebius public workbench currently defaults to `MiniMaxAI/MiniMax-M3` | Test with actual Go2 camera frames, including standing/lying distractors and uncertain views. Its use does not replace the required NVIDIA planner. |
| NVIDIA visual alternative | `nvidia/Cosmos-Reason2-2B` deployed on Nebius AI Cloud | Evaluate only if access, serving cost, image performance, and integration time are acceptable. NVIDIA's reference deployment states a 24 GB minimum GPU memory requirement. |

Nebius's [August 2026 Serverless retirement notice](https://docs.tokenfactory.nebius.com/august-2026-deprecation-notice) lists `nvidia/Cosmos3-Super-Reasoner` as retired. Do not use an old hosted-model announcement as evidence of current availability. Nebius's [workbench guide](https://github.com/nebius/nebius-physical-ai/blob/main/docs/workbench/token-factory.md) and [migration verification](https://github.com/nebius/nebius-physical-ai/blob/main/docs/workbench/token-factory-deprecation-verification.md) describe current public defaults but explicitly warn that account access and image support require live verification. The [Cosmos-Reason2 model card](https://huggingface.co/nvidia/Cosmos-Reason2-2B/blob/main/README.md) and [reference deployment](https://github.com/nvidia-cosmos/cosmos-reason2) document the self-hosted alternative.

## Key technical risks

1. **Place confusion and odometry drift:** a visit can be counted twice or a useful region missed. Keep missions short, use visual landmarks and explicit uncertainty, and measure revisit errors.
2. **Target visibility:** a lying person may be partly hidden or seen from a poor camera angle. Test real robot-height snapshots early and allow a second safe view.
3. **Model latency or failure:** stop while awaiting a high-level decision; bound each request and retain a safe fallback.
4. **False confirmation:** require evidence for every requested attribute and include absent-target trials.
5. **Integration time:** the smallest physical loop—observe, decide, move once, stop—must work before adding richer memory or extra models.

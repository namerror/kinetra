# Nebius × NVIDIA Global AI Hackathon reference

Last checked against the official Devpost pages: **October 4, 2026**. This file records requirements and project-specific release boundaries. The [official rules](https://nebiusglobalaihackathon.devpost.com/rules) take precedence if anything here changes or conflicts with them.

## Event and track

- [Hackathon overview](https://nebiusglobalaihackathon.devpost.com/)
- **Submission deadline:** October 30, 2026, at **10:00 a.m. Pacific Daylight Time**. Finish engineering earlier to leave time for physical trials, a public repository, video editing, and the Devpost writeup.
- **Selected track:** Physical AI. The track covers embodied or edge agents that sense and act in the real world. This project's physical hardware is a Unitree Go2.
- **Required technology:** the working project must use at least one NVIDIA open-source model and run on Nebius Token Factory or Nebius AI Cloud. The rules define running on Nebius as a runtime Token Factory inference API call **or** deployment/execution on Nebius AI Cloud compute such as Serverless Jobs, Serverless Endpoints, or DevPods. The entire robot stack need not run in the cloud. [Official rules](https://nebiusglobalaihackathon.devpost.com/rules)

For Kinetra Search, the intended qualifying path is a real Nemotron runtime call through Token Factory that chooses a high-level search action from the goal and observed state. The selected model, request, response, and physical effect must be traceable in the demo and logs. Model availability and behavior still require testing with the team's account; see [architecture](ARCHITECTURE.md).

## Judging implications

Stage one is a pass/fail check for a viable, genuine fit to the track and reasonable use of the required technology. Stage two scores four criteria equally: **Technological Implementation**, **Design**, **Potential Impact**, and **Quality of the Idea**. [Official rules](https://nebiusglobalaihackathon.devpost.com/rules)

The organizers say that a scripted Physical AI demonstration is a baseline; hardware reacting live to physical input stands out. The search demo should therefore show the Go2 responding to a live target placement or distractor, with visible evidence of why it changed its search plan. A meaningful NVIDIA/Nebius decision is more valuable than a qualifying API call with no effect. [Judging guidance](https://nebiusglobalaihackathon.devpost.com/updates/46204-here-s-how-judging-works)

The [project specification](PROJECT_SPEC.md) and [evaluation plan](EVALUATION.md) define the actual mission and baseline. Keep this file focused on hackathon obligations rather than duplicating the product design.

## Submission checklist

- [ ] Working Physical AI project that behaves as described.
- [ ] Select the Physical AI track and provide a project title and clear description.
- [ ] Public GitHub, GitLab, or Bitbucket repository with necessary releasable source, assets, setup/running instructions, and a visible open-source license.
- [ ] Explain exactly which NVIDIA model is used, what it does, and where Nebius Token Factory or AI Cloud is used.
- [ ] Public YouTube video **under three minutes** showing the project working, including **at least one minute of actual Go2 operation**. Call out the Nebius and NVIDIA contribution in the video.
- [ ] Provide specific feedback on the Nebius services and NVIDIA models or tools used.
- [ ] Explain significant work added during the submission period if the project or any of its components existed earlier.
- [ ] Verify that API keys, private data, and unapproved third-party or lab material are absent from the public repository and video.
- [ ] Recheck the official rules and submission form immediately before submitting.

A separate hosted demo URL is not required for a Physical AI submission. Judges may judge from the written submission and video, so both should make the physical behavior and model contribution understandable without a live robot demonstration. [Official rules](https://nebiusglobalaihackathon.devpost.com/rules) · [Organizer submission guidance](https://nebiusglobalaihackathon.devpost.com/updates/46205-how-to-build-a-winning-project)

## Pre-existing work and public-release boundary

The Go2 and any existing lab follow/navigation policy are pre-existing infrastructure. The hackathon contribution is the new public text-guided search layer: goal interpretation, place and candidate memory, bounded action selection, Nebius/NVIDIA integration, verification, baseline comparison, and reproducible evaluation. Document what was actually built during the submission period; do not claim the underlying robot or policy as new work.

Do **not** add unpublished data-generation code, datasets, model-training infrastructure, private checkpoints, lab-owned source, or publication-sensitive experiments to this repository. Any private/lab implementation belongs outside the public tree and may be connected only through a minimal documented interface when its use and release are approved. Follow the license and terms for each external policy, model, SDK, API, dataset, and library. The official rules require authorization for third-party integrations and a submission that does not infringe others' rights. [Official rules](https://nebiusglobalaihackathon.devpost.com/rules)

## Demo and documentation priorities

1. Get a repeatable physical search trial working and record real behavior.
2. Show how a live observation changes the next bounded action and how a plausible wrong candidate is rejected.
3. Report matched baseline results, including failures and absent-target cases.
4. Make the public repository runnable without private lab code, or clearly document any hardware-specific dependency and the mock/recorded-scene path.
5. Reserve the last days for a concise video, README, license visibility, secret review, model/service feedback, and submission.

The organizers recommend presenting the video as a pitch: identify the problem and audience, show the working behavior, and explain the Nebius/NVIDIA role. [Organizer submission guidance](https://nebiusglobalaihackathon.devpost.com/updates/46205-how-to-build-a-winning-project)

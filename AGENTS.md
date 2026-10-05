# Instructions for agents working on Kinetra Search

Read `README.md`, `docs/PROJECT_SPEC.md`, `docs/ARCHITECTURE.md`, `docs/HACKATHON.md`, and `docs/SETUP_HANDOFF.md` before implementing anything. `docs/HACKATHON.md` is the local reference for rules, submission obligations, and public-release boundaries; the current official Devpost rules take precedence.

## Project boundaries

- Keep this repository public and hackathon-specific. Do not copy unpublished lab code, datasets, checkpoints, data-generation infrastructure, or publication-sensitive experiments into it.
- Keep model decisions separate from local obstacle avoidance, locomotion, and emergency stop. Models choose only validated, bounded actions.
- The MVP is a bounded indoor search using an explicit topological/viewpoint memory, short-range odometry, and local sensing. Do not silently add geometric SLAM or claim arbitrary-building autonomy.
- Use the same perception and robot skills for the learned/planned system and its fixed-order baseline so evaluation isolates the supervisor's contribution.
- Verify current model availability and image-input support with the actual Nebius account. A catalog entry, old announcement, or model card alone is not proof of a working integration.
- Keep hardware interfaces swappable and provide a development path that does not require the Go2 for every iteration.
- Record observations, decisions, action outcomes, timeouts, and failure reasons. Never label an unverified candidate as found.
- Update setup instructions, source attribution, and the pre-existing-versus-hackathon-work distinction as implementation proceeds.

## Near-term priority

Follow `docs/SETUP_HANDOFF.md`: establish the Go2's available sensors and safe bounded motion, confirm Nebius/NVIDIA access, and test target visibility before expanding the architecture. Do not assume the repository has a prior implementation.

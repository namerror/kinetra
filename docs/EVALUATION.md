# Evaluation plan

## Question

Does goal-conditioned planning over explicit visual memory help the Go2 find the **correct** described person faster or more reliably than a fixed search order, using the same perception and movement components?

## Compared systems

1. **Fixed-order baseline:** visit reachable viewpoints in a predetermined order, verify each plausible candidate, then revisit uncertain viewpoints once.
2. **Kinetra Search:** use the same visual observations, candidate verification, movement limits, and mission budget; Nemotron chooses the next view and whether another angle is needed.

If time permits, add a simple deterministic heuristic that prioritizes unvisited viewpoints and regions hidden by occlusion. Do not compare against a deliberately weak baseline or give the model extra sensor information unavailable to the baseline.

## Trial conditions

Rearrange partitions or target placement across complete missions. Include a clear target, a partly hidden target, a standing dark-clothed distractor, and an absent-target condition. Record the target's ground-truth location and attributes separately from robot observations. Run matched conditions for both supervisors; randomize execution order where practical. Report the number of trials and all failures, including stops and timeouts.

## Metrics

- **Correct find rate:** verified reports of the described person within the mission limit.
- **False find rate:** wrong-person or unsupported found reports; this is the highest-priority error.
- **Absent-target accuracy:** correct bounded “not found” reports when no target exists.
- **Time and distance to correct find**, reported only with success rate beside them.
- **Redundant viewpoint visits**, blocked actions, model timeouts, and safety stops.
- **Evidence quality:** whether the final report cites actual views supporting each requested attribute.

Do not treat “saw a person” or a model's self-reported confidence as proof of success. A search system that is faster but increases false finds is not an improvement.

## Stage gates

1. Real Go2 snapshots reliably show the relevant person and posture at feasible viewing distances.
2. Local bounded motion and obstacle stopping work in the trial area.
3. A real Nebius/NVIDIA call produces a validated decision that changes a Go2 action.
4. The full mission completes target-present and target-absent trials.
5. Matched baseline trials show whether the planner adds measurable value; publish the result honestly even if the baseline wins.

The demo video should show live target placement, robot search, candidate rejection, final evidence, and a short side-by-side or metric summary. Official Physical AI submissions require at least one minute of operating hardware in a video no longer than three minutes. See the [hackathon rules](https://nebiusglobalaihackathon.devpost.com/rules).

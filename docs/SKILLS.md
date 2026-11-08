# Reusable skill boundaries

This is a deterministic baseline, not an RL policy library yet.

| Skill | Inputs | Outputs / completion |
| --- | --- | --- |
| Approach panel | Map goal, Nav2 status | Arrived or navigation failure/timeout |
| Settle base | Timestamped odometry | Low linear/angular speed |
| Align view | Fresh target box, command limits | Small centered error |
| Hold and capture | Fresh images, sharpness, observed image motion | Exact frame saved after stable dwell |
| Reacquire | Last command, elapsed loss, camera observations | Target visible again or target timeout |
| Report mission | Required IDs, accepted evidence, events | Complete coverage or explicit partial failure |

Replace search/alignment with residual RL only after a benchmark shows a concrete
weakness. Keep the deterministic freshness, stopping and evidence checks around the
learned component. Share interfaces, datasets and evaluation recipes across devices;
retrain motor policies for each embodiment.

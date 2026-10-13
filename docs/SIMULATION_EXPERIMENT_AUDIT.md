# COCO Simulation Experiment Audit

This document records the evidence boundary for the paper experiments. The
author baseline is `origin/feature/simulation` at commit `0e56c47`. The audit
work is on branch `audit/paper-faithful-20260902`.

## Research Questions

The experiments must answer three separate questions:

1. Does a per-leg step-scale change reach the simulated plant?
2. Does RSA identify a useful local relation between leg scale and lateral
   drift, then retain a beneficial scale?
3. Does changing the reaction/update horizon change the intended controller
   behaviour under otherwise identical conditions?

These questions must not be collapsed into one plot.

## Control Invariants

The intended movement chain is:

```text
cmd_vel -> FTG nominal foot step -> RSA step scaling -> KT/IK
         -> JointTrajectoryController -> GazeboSim ros2_control -> plant
```

For forward gait, RSA's controlled scalar is signed lateral displacement in the
gait frame. Yaw is a diagnostic quantity, not the primary RSA objective.

`reaction_horizon_steps` controls when the long-term goal is advanced. It is
not the duration of a learning probe. A probe is held for
`learning_observation_steps` half-steps. The local Jacobian is estimated from
cycle-matched drift in metres per gait step.

## Changes By Category

### Simulation-only startup fixtures

- start Gazebo paused;
- spawn above the ground, unfold, reset pose, then restore gravity;
- simplify collision proxies and disable self collision;
- use a stronger GazeboSim position gain;
- keep the startup grace period outside the RSA disturbance detector.

These changes are not algorithmic evidence. They must be reported as
simulation initialization details.

### RSA algorithm and observability

- per-leg stance XY scaling is applied before KT;
- the scale change is propagated through the remainder of the foot trajectory;
- normal gait drift is estimated separately for the two alternating phases;
- a full-cycle drift is used for learning;
- experiments are rolled back on invalid or catastrophic observations;
- persistent learning state is isolated per trial;
- signed tracking error can trigger a probe, while candidate prediction uses
  only the actual observation window, never the reaction horizon.

### Experiment tooling

- every trial records a private learning YAML, parameters, rosbag, odometry,
  IMU and RSA state CSV;
- fixed-distance evaluation interpolates at the target distance;
- the trial continues beyond the target so a phase-matched cycle sample exists;
- Gazebo receives an explicit `simulation_seed` and the seed is recorded;
- only `valid_for_fixed_distance=true`, non-fallen trials are eligible for
  aggregate statistics.

## Invalid Previous Evidence

The earlier H plots must not be used as paper evidence because they mixed fixed
time, fixed RSA-step and fixed-distance stopping. Some records were labelled
as 1 m although the robot had travelled much less. Earlier endpoint metrics
also sampled the last point after crossing the target, not the target itself.
In addition, H was temporarily used as a prediction horizon while probes were
held for only two half-steps. Those results are diagnostic history only.

## Current Evidence

The following evidence is valid as engineering evidence, not yet as a final
paper claim:

- RSA step scaling unit tests pass (`7/7` after removing dead horizon code).
- A frozen scale is visible in `/rsa/state` and changes the foot trajectory
  before KT.
- With a simulated leg propulsion gain of `0.90`, three nominal trials had
  mean phase-matched target cross-track `24.15 +/- 5.27 mm` and mean cycle
  drift RMS `8.63 +/- 1.80 mm/cycle`.
- Three frozen-scale trials (scale clamped by the configured maximum to 1.10)
  had mean phase-matched target cross-track `22.44 +/- 9.68 mm` and mean cycle
  drift RMS `7.16 +/- 0.61 mm/cycle`.

The cycle-drift reduction is suggestive of a real plant effect, but the sample
size is still too small for a paper conclusion. Endpoint cross-track variance
is large because a quadruped's body oscillates within a gait cycle.

Online RSA now starts and evaluates probes. In one 0.8 m run it accepted two
leg corrections and rolled back others, but it did not reliably identify the
artificially perturbed leg. Online convergence is therefore not established.

### Fresh reproducibility check (2026-09-02)

Three matched trials were repeated with `simulation_seed=17`, a 0.5 m target,
and 0.12 m post-target continuation. The artificial perturbation was a
single-leg stance-step scale of `0.90` on leg 1.

| condition | target cycle cross-track | lateral drift RMS |
|---|---:|---:|
| RSA off, repeat A | 18.5 mm | 6.43 mm/cycle |
| RSA off, repeat B | 24.4 mm | 8.79 mm/cycle |
| RSA frozen | 21.5 mm | 7.43 mm/cycle |
| RSA online | 32.5 mm | 8.91 mm/cycle |

The same Gazebo seed did not reproduce identical trajectories. The nominal
unperturbed run measured 8.08 mm/cycle, close to the perturbed runs. This means
the current fixture does not create a disturbance that is clearly larger than
contact and scheduling noise. Frozen correction is therefore only suggestive,
and the online result is not evidence of convergence. These runs are a
diagnostic reproducibility check, not final paper statistics.

The online run performed five probes and ended with scales
`[1.01, 1.01, 1.01, 1.00]`; it accepted probes on legs 0, 1 and 2, rolled back
leg 3, and left a later leg-0 probe neutral. This is real exploration, but it
does not identify the deliberately perturbed leg reliably.

### Objective-consistency correction (2026-09-02)

The candidate-selection path previously ranked a probe using an H-dependent
tracking-error prediction, while the acceptance path judged the same probe by
the measured gait-frame drift magnitude. Those are different objectives. The
selection path now uses the same signed gait-frame drift cost as acceptance;
`reaction_horizon_steps` is no longer involved in RSA candidate ranking. The
change is committed as `b4e75ab` and the seven direct RSA unit tests pass.

After this correction, matched 0.5 m trials with a 5-degree single-leg
joint-command bias still showed substantial variation: RSA-off drift RMS was
`4.63 mm/cycle` and RSA-online was `11.30 mm/cycle` for one seed. The online
run explored two candidates and restored all scales. This is evidence that the
objective mismatch was real, but also that the present Gazebo contact model and
the 1% probe size do not provide enough signal-to-noise for a reliable online
learning claim.

The correction keeps two distinct roles for the signals: accumulated
gait-frame tracking error or the latest cycle drift can trigger an experiment,
while candidate ranking and acceptance both use the same cycle-drift cost.
This preserves the ability to react to slow accumulated bias without allowing
the H-dependent goal bookkeeping to choose a leg.

## Required Experiment Protocol

For every condition:

- fixed target distance: 1.0 m;
- continue at least one complete gait cycle beyond the target;
- fixed command, startup parameters, collision model and controller gains;
- use a documented list of Gazebo seeds;
- start from a fresh learning file for independent trials;
- report target cycle cross-track, cycle drift RMS, signed cycle drift, yaw,
  roll/pitch peaks, forward distance and fall rate;
- report the actual start/end scale, confidence and experiment count.

The RSA comparison should be three separate conditions under the same plant
perturbation:

```text
RSA off       : adaptation=false, online_learning=false
RSA frozen    : adaptation=true,  online_learning=false, preidentified scale
RSA online    : adaptation=true,  online_learning=true, fresh learning file
```

At least five independent seeds are needed for a preliminary result; ten is
preferred. The frozen condition must first show a repeatable reduction in the
controlled drift. If frozen works but online does not, the defensible claim is
that the execution path and local effect are demonstrated, while online
identification remains limited by simulated contact noise.

## Horizon Ablation Boundary

Do not assume that H=1,5,10 must produce a monotonic curve. H can only support
that claim if the author confirms that it is intended to alter the RSA decision
interval, rather than only the bookkeeping cadence of the long-term goal. With
the current implementation, H is deliberately isolated from probe
identification. A flat H result is therefore valid and may be the correct
finding.

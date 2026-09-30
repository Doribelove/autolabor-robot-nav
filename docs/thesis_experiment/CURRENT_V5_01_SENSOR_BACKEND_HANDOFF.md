# V5-01 Sensor Backend and Pre-evidence Repair Handoff

Date: 2026-08-08

## Scope

V5 is an independent successor to the frozen V2/V3/V4 studies.  This stage is
simulation/offline only and changes neither `/home/robot/robot_ws` nor the
real-robot command chain.  No V5 real-robot motion or online TEB parameter write
is authorized.

The deployable actor receives only six LaserScan frames, odometry/local-goal
features, candidate features, the sensor analytic prior and the hard safe-action
mask.  Simulator actor state and oracle candidate outcomes are exposed only by
the separate critic interface.  The learned output is one of eight
Ackermann-feasible candidate IDs; deterministic generation, certified stop,
hard shielding and tracking stay outside the network.

## Retained negative development diagnostics

The first development replay found that the provisional 0.75 s fixed-obstacle
mask was not a braking certificate.  In 100 analytic episodes per family it
recorded 23, 32, 0 and 100 contacts respectively for bidirectional crossing,
sequential crossing, partial occlusion and narrow-corridor yield.  Partial
occlusion also produced 0/100 goals.  These are code-development diagnostics,
not eligible experimental evidence, but they are retained because they caused
the component repair.

The cause had two parts:

1. a candidate could be accepted at 1.2 m/s although the remaining stopping
   distance exceeded the visible clearance; and
2. the spatial sine bump began only at x=-2.4 m, too late for the Ackermann
   steering-rate bound to establish lateral clearance.

Before any eligible training, validation or formal episode, contract v1.1
therefore froze a one-decision-plus-maximum-deceleration backup certificate,
mapped-static/dynamic-return separation, a 1.35 m/s dynamic reachable bound,
0.12 m scan inflation, and a continuous spatial bump from x=-4.0 m to x=4.0 m.
Actors retain a physical collision-avoidance boundary so certified stop is not
invalidated by an artificial adversarial rammer.  Interaction mode still changes
their negotiation response.

A 1,536-step seed999 PPO code smoke was run under `/tmp` only to test tensor,
optimizer and checkpoint plumbing.  It is explicitly ineligible for checkpoint
selection, evidence or claims and consumes none of seeds 9101--9104 or the V5
1.8 M eligible training-step budget.

## Required gate before eligible training

Run `audit_v5_sensor_backend.py` once on the frozen component.  The gate requires
at least 0.80 sensor-history direction identifiability, at least 0.25 safe
multi-action state fraction, at least 0.20 analytic-regret block fraction, zero
contact, zero mask violations, and an actor-only checkpoint leakage pass.

All later V5 outputs must preserve the audit directory byte-for-byte.  A failed
gate stops eligible PPO training until a new, explicitly logged component stage
is designed; it may not be repaired using V5 validation or formal outcomes.

## V5-01 result and R1 entry

The first frozen audit is terminally negative: flattened-history ridge accuracy
was 0.7667 against the 0.80 gate.  Its other gates passed (200 safety episodes,
zero contact, zero mask violations, 0.6303 safe multi-action fraction, 0.50
analytic-regret fraction, actor leakage pass).  The contained latest-frame probe
was 0.9167.  No eligible PPO training was started.

V5-01-R1 changes only the identifiability probe to a 72-dimensional latest-frame
ridge selected from the six-frame policy input.  It makes no history-benefit
claim; that remains for the no-history ablation.  The environment, reward,
network, action space and every numeric gate remain frozen.  R1 uses fresh
development seeds and cryptographically binds the complete failed predecessor.

R1 subsequently passed on fresh seeds: latest-frame direction accuracy 0.9417,
safe multi-action fraction 0.6276, analytic-regret fraction 0.68, 200/200 safety
episodes with zero contact and zero mask violation, and a passing actor leakage
audit.  It authorizes only the separately frozen V5-02-R1 fast-backend training
schedule; it does not authorize formal seeds, Gazebo, TEB execution or hardware.

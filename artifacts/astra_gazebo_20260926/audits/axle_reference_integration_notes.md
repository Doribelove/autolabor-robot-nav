# Base/rear reference conversion components

`axle_reference_adapter.py` supplies pure pose, body-twist and footprint-point shifts with finite-input checks;30 tests pass. No runtime, planner, ROS node, physical plugin or frozen input is modified.

For wheelbase L=.65, rear_pose=base_pose−R(yaw)(.325,0); rear_twist=(base_vx, base_vy−.325*w, w). The inverse restores exactly the measured state, including lateral slip. An ideal rear bicycle has rear_vy=0 and therefore base_vy=.325*w. Replacing the actual base vy by zero is wrong. Longitudinal vx and yaw rate are unchanged; Euclidean speed magnitude is a different quantity. A footprint point expressed at base becomes (x+.325,y) at rear, preserving its world position.

Current executable chain uses AstraSyncEpisode→frozen R2→BenchmarkRuntimeAdapter. The inherited runtime_adapter.py:88 _kinematic_step integrates x+=v*cos(yaw)*dt/y+=v*sin(yaw)*dt, which is a rear-axle bicycle equation; _clearance at149 interprets the preview point as the centered robot. Global coordinate translation alone cannot repair this during a turn. The base and rear paths differ by a yaw-dependent vector.

For a future versioned adapter, keep the public runtime pose at base_link. Evaluate route/lateral targets at this base pose, convert to rear before bicycle translation and back after advancing yaw. If an unchanged old step produced raw_next from base_start with rear-bicycle translation, the equivalent corrected base point is raw_next.xy + .325*(heading_vector(next_yaw)−heading_vector(start_yaw)). This algebra preserves the old target lookup point while correcting the integrated reference; it does not qualify actuator tracking or measured slip. Convert every intermediate and backup preview sample before using footprint clearance. Leave scan sensor pose/TF at the real sensor origin; do not shift both scan and robot by accident.

External goals and gate positions remain defined for the actual base. At yaw0 the diagnostic base start−4 maps to rear−4.325, and base goal5 maps to rear4.675. For other goal headings use the full pose conversion, not a fixed world-X subtraction. Ackermann commands vx/w remain the same body command. Tests prove SE2 commutation, exact forward/inverse transformations, analytic circle derivatives at base, actual slip retention and invariant footprint world geometry.

Any live adoption needs a new candidate/runtime identity and fresh physical tracking/backup assessment. Pure geometry support is not evidence that current frozen R2 has been adapted or that its historical safety envelopes cover M12 dynamics.

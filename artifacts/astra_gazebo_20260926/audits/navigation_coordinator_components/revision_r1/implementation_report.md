# Shared navigation coordinator and adapters

Offline component checks passed: 19 focused tests and four actual `roslaunch --dump-params` expansions. No ROS master, Gazebo run, build, seed allocation or release was created. Source roster and exact interface are in `implementation_report.json`.

Coordinator owns the sole `/cmd_vel_navigation` publication. All methods publish private `/astra/method_cmd`; its headerless receipt time is never described as a generation/source stamp. Collector alone spawns, unpauses and pauses. A common 5 s activation begins .8 m/s warmup; the first actual odometry crossing of x=-4 m fixes the common 22 s formal deadline. Gate acknowledgement clears all earlier method commands. TEB/DWA get their map goal only after gate; the fixed global planner retrieves the shared path. Actual base pose controls goal/timeout.

The route adapter fixes method-local remapped publisher checking and takes the publisher clock inside its lock, with a reproduced racing start callback test. Frozen standalone bytes are retained. R2 overrides only readiness and warmup, leaving inherited algorithm, weights, candidates and verifier untouched. The same blocked raster enters R2 as exact rectangles, but it does not consume the fixed route and retains its implicit +/-20 m y walls and outside-map semantics. Its old report/clock is diagnostic; the inherited final zero tail may wait after collector pause.

The costmap uses the explicit raw16-plane footprint plus Euclidean .33 m clearance, enclosed by64 support planes with at most .00039798 m extra radial arc conservatism and `footprint_padding=0`. TEB receives the raw footprint with `min_obstacle_dist=.33`. This is a geometry/configuration statement, not proof of equal raster algorithm behavior or physical safety.

New analytic route is runnable for N01; the new PCT identity requires a separately trained new-schema checkpoint. The visible-point isotropic predictor remains an explicit bootstrap fallback. Live four-method input consumption, physical navigation qualification and independent review remain separate prerequisites.

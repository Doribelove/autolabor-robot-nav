N02 sensor lifecycle integration (offline)
=========================================

Three new source files preserve N01. The core changes only the R2 method identity; the coordinator ROS node changes that identity and core import. The launch connects reviewed sensor lifecycle r1 with the actual required bundle/checkpoint parameters and five evidence topics. Physical URDF and all non-R2 planner/provider parameters remain exact.

13 tests pass. Four actual roslaunch parameter expansions pass without starting ROS; the real R2 constructor consumes the expanded parameters and original checkpoint/runtime. Common gate/command/goal/timeout values match N01 after identity normalization. Original N01 and reviewed lifecycle source hashes remain unchanged.

Root must supply the new collector_script explicitly and bind its new N02 contract/runner. The method ROS name remains /astra_frozen_r2_method; public command authority stays /astra_navigation_coordinator. Bag contract must include the five lifecycle topics; the launch appends them, so root may exclude them from its bag_topics argument to avoid duplicate words.

No seed allocation, execution, training, build or release. R2 shared path usage remains audit_only, not full-path consumption; this is not primary-matrix or live physical qualification.

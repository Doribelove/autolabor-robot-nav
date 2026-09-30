# M08 independent component review

The six frozen new sources pass this focused offline component review. Ten independent small fixtures passed: causal acceptance/expiry, reset fault, exact odom/joint join, six real history rows, byte-equal tracker acceptance and inherited rollouts, KD connected components against all-pairs transitive closure, strict circle dominance against GEOS with the full robot hull, deadline zeroing, expired worker isolation, and namespace/terminal separation.

**Existing runtime environment is blocked.** After sourcing `astra_env.sh`, the new node’s `env python3` resolves to `.venv`, where SciPy1.3.3 requires the removed NumPy1.24 `typeDict`. The actual import failure is saved. Author36tests and timing used unsourced `/usr/bin/python3` with user-site NumPy1.21.0; sourced explicit system Python instead uses NumPy1.17.4. These are distinct environments.

A separately bound absolute `.venv_train_cuda/bin/python` is an available candidate: sourced-environment imports of rospy, rosbag, cv_bridge, M08 core and worker pass with NumPy1.24.4/SciPy1.10.1 and user-site disabled. Torch was not imported. This is import-only evidence, not cross-environment geometry, timing or runtime qualification.

The full-core author negative remains69.239986ms versus50ms, even before ROS scheduling or learned inference. No benchmark was repeated during N04. The code zeros an expired transaction at its eventual prepublish boundary; it cannot replace the last command retroactively while synchronous work blocks. A future research diagnostic must measure actual ingress and drop frequency. No worst-case real-time or physical safety claim follows.

The pure input buffer commits its raw frame before tracker acceptance; a failed tracker commit is detected as a freshness mismatch and the actual node faults/zeros. Do not reuse that helper as an atomic rollback transaction. Shared task outcomes remain separate from preserved local input faults. The policy does not subscribe to Gazebo truth.

This supports preparing a new bounded **analytic M08 diagnostic** after compatible environment closure and new method/launch/collector release integration. Runtime readiness remains false. The PCT entry additionally needs a newly trained matching namespace checkpoint; no such artifact exists. No source, old environment, N04 binding, seed registry, ROS process or GPU state was changed.

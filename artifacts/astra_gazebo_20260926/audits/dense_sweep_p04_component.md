P04 converted-model sweep component
=================================

Final source SHA: 3bebade4f43d53f0a7cc0be1fd067d42bdb9d9c6251d4d2c728f59993c6f0736

35 focused offline tests pass. Actual Q04 metadata matches all 7 surviving bodies and 9 collisions against the saved converter output at the unchanged 1e-7 comparison threshold. Base COM is 0.272673 m; its conservative common-center union radius is 0.6495887595854349 m. The ideal URDF radius/COM mismatch remains recorded in Q04; its historical result is not changed.

`create_conversion(urdf_path, output_dir)` creates two independently captured offline gz sdf conversions, exact-output reproduction, actual process maps and ELF bindings, then a manifest. The new evaluator requires the converted SDF and manifest plus the same sdformat SHA in the actual runtime maps. It permits byte-identical copied SDF/manifest paths. Required additional closure roles: converted_robot_sdf, urdf_sdf_conversion_manifest, conversion_tool, conversion_libraries. SCOPE/PROFILE/read_p03_bag remain exported; all numeric profile values are unchanged.

Runtime mass, COM, inertia pose, tensor, native ODE tensor and complete collision identities/poses/dimensions are checked. URDF fixed members/frames and moving joint topology map to the converted representation, including explicit decimal quantization. Both source and effective geometry radii use the same actual runtime COM and an outward ULP.

This remains a bounded Gazebo algorithmic-extension engineering envelope, not formal or real continuous physical safety. Existing metadata does not directly measure joint parameter values. No ROS, simulation episode, training, build, physical source or historical-file modification occurred. Full actual bag replay and independent review are assigned to historical_audit.

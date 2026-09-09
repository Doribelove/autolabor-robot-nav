# Third-party notices

This distribution preserves existing package metadata and source notices. The
root MIT license does not replace licenses on third-party files.

| Component | Origin / attribution | License and included notice |
|---|---|---|
| Coverage manager, geometry, global planner, command mux | Existing `autolabor_coverage` project and contributors | MIT as declared by package.xml, except the Reeds–Shepp implementation below |
| Reeds–Shepp closed-form families | Rice University, 2010; adapted from OMPL's `ReedsSheppStateSpace` by Mark Moll | BSD 3-clause text retained in `src/application/autolabor_coverage/src/reeds_shepp.cpp` |
| Customized `teb_local_planner` | Christoph Rösmann / TU Dortmund, Lehrstuhl RST, with local coverage tracking changes | BSD 3-clause; full `src/navigation_arena/forks/navigation/local_planner/teb/LICENSE` and source notices retained |
| `autolabor_canbus_driver` | Autolabor and existing contributors, per package metadata | MIT as declared by its package.xml; original files and maintainer attribution preserved |
| Navigation ownership/pause bridge | Existing `autolabor_dual_host` project and contributors | MIT as declared by the source package.xml |

ROS, Navfn, costmap_2d, g2o, SuiteSparse, Eigen and other dependencies are installed
separately and retain their own licenses. Their binaries are not committed here.
`SOURCE_MANIFEST.json` records the source snapshot and SHA-256 of every exported
file; exporting does not transfer ownership or remove upstream attribution.

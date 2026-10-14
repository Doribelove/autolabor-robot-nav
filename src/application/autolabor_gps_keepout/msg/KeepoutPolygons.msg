std_msgs/Header header
geometry_msgs/Polygon[] polygons
# Keep-in union contours (including holes), using even/odd fill.
# Enabled with no contours means no traversable cells, not unrestricted space.
bool keep_in_enabled
geometry_msgs/Polygon[] keep_in_polygons

# solid geometric primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses
std_msgs/ColorRGBA[] primitive_colors # The color is optional

# bounding planes (equation is specified, but the plane can be oriented using an additional pose)
shape_msgs/Plane[] planes
geometry_msgs/Pose[] plane_poses
std_msgs/ColorRGBA[] plane_colors # The color is optional

# Meshes
Mesh[] meshes
geometry_msgs/Pose[] mesh_poses
std_msgs/ColorRGBA[] mesh_colors # The color is optional

# Convex Hull Meshes
Mesh[] convex_meshes
geometry_msgs/Pose[] convex_mesh_poses
std_msgs/ColorRGBA[] convex_mesh_colors # The color is optional

# SDF (Bullet) Meshes
Mesh[] sdf_meshes
geometry_msgs/Pose[] sdf_mesh_poses
std_msgs/ColorRGBA[] sdf_mesh_colors # The color is optional

# The octomap
octomap_msgs/Octomap[] octomaps
geometry_msgs/Pose[] octomap_poses
std_msgs/ColorRGBA[] octomap_colors # The color is optional

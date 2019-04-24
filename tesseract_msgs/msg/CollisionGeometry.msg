# Types for collision objects
byte USE_SHAPE_TYPE = 0          # Infer the type from the type specified in the shapes::Shape class
byte CONVEX_HULL = 1             # Use the mesh in shapes::Shape but make it a convex hulls collision object. (if not convex it will be converted)
byte MULTI_SPHERE = 2            # Use the mesh and represent it by multiple spheres collision object
byte SDF = 3                     # Use the mesh and rpresent it by a signed distance fields collision object

# solid geometric primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses
std_msgs/Int32[] primitive_collision_object_types
std_msgs/ColorRGBA[] primitive_colors # The color is optional

# meshes
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses
std_msgs/Int32[] mesh_collision_object_types
std_msgs/ColorRGBA[] mesh_colors # The color is optional

# bounding planes (equation is specified, but the plane can be oriented using an additional pose)
shape_msgs/Plane[] planes
geometry_msgs/Pose[] plane_poses
std_msgs/Int32[] plane_collision_object_types
std_msgs/ColorRGBA[] plane_colors # The color is optional

# The octomap
octomap_msgs/Octomap[] octomaps
geometry_msgs/Pose[] octomap_poses
std_msgs/Int32[] octomap_collision_object_types
std_msgs/ColorRGBA[] octomap_colors # The color is optional

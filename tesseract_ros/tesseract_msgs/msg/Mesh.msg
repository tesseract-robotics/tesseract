# General definition of a mesh

# the actual vertices that make up the mesh
geometry_msgs/Point[] vertices

# the indicies that make up the faces in the mesh
# the first number indicates the number of vertices that make up the face,
# followed by the indices of the vertices.
uint32[] faces

# Optional, If created from file provide path
string file_path

# Optional, If file_path is provide provide the scale that was used
float32[3] scale

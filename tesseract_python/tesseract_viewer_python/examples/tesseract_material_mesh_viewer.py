from tesseract.tesseract_environment import Environment
from tesseract.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn
import os
import re
import traceback
from tesseract_viewer import TesseractViewer
import numpy as np
import time
import sys

shapes_urdf="""
<robot name="mesh_viewer">
  
  <link name="world"/>
  <link name="mesh_gltf2_link">
    <visual>
      <geometry>
        <mesh filename="package://tesseract_support/meshes/tesseract_material_mesh.glb"/>
      </geometry>
    </visual>
  </link>

  <joint name="mesh_gltf2_joint" type="revolute">
    <parent link="world"/>
    <child link="mesh_gltf2_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 -1 0.25"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="mesh_dae_link">
    <visual>
      <geometry>
        <mesh filename="package://tesseract_support/meshes/tesseract_material_mesh.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="mesh_dae_joint" type="revolute">
    <parent link="world"/>
    <child link="mesh_dae_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 1 0.25"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

</robot>
"""

TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

def _locate_resource(url):
    try:
        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
        if (url_match is None):
            return ""    
        if not "TESSERACT_SUPPORT_DIR" in os.environ:
            return ""
        tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
        return os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
    except:
        traceback.print_exc()

t_env = Environment()

# locator_fn must be kept alive by maintaining a reference
locator_fn = SimpleResourceLocatorFn(_locate_resource)
t_env.init(shapes_urdf, SimpleResourceLocator(locator_fn))

viewer = TesseractViewer()

viewer.update_environment(t_env, [0,0,0])

viewer.start_serve_background()

if sys.version_info[0] < 3:
    raw_input("press enter")
else:
    input("press enter")


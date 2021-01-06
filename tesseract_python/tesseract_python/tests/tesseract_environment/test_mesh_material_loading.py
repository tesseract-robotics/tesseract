from tesseract import tesseract_scene_graph
from tesseract import tesseract_collision
from tesseract import tesseract_environment
from tesseract.tesseract_common import Isometry3d, Translation3d, AngleAxisd
from tesseract import tesseract_common
from tesseract import tesseract_collision
from tesseract import tesseract_collision_bullet
from tesseract import tesseract_urdf
import os
import re
import traceback
import numpy.testing as nptest

mesh_urdf="""
<robot name="mesh_viewer">
  
  <link name="world"/>
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

def get_scene_graph():
    locator_fn = tesseract_scene_graph.SimpleResourceLocatorFn(_locate_resource)
    locator = tesseract_scene_graph.SimpleResourceLocator(locator_fn)    
    return tesseract_urdf.parseURDFString(mesh_urdf, locator)
    
def test_mesh_material_loading():
    scene = get_scene_graph()
    visual = scene.getLinks()[0].visual
    assert len(visual) == 4

    mesh0 = visual[1].geometry
    mesh1 = visual[2].geometry
    mesh2 = visual[3].geometry
    mesh3 = visual[0].geometry

    assert mesh0.getTriangleCount() == 34
    assert mesh0.getVerticeCount() == 68
    assert mesh1.getTriangleCount() == 15
    assert mesh1.getVerticeCount() == 17
    assert mesh2.getTriangleCount() == 15
    assert mesh2.getVerticeCount() == 17
    assert mesh3.getTriangleCount() == 2
    assert mesh3.getVerticeCount() == 4

    mesh0_normals = mesh0.getNormals()
    assert mesh0_normals is not None
    assert len(mesh0_normals) == 68
    mesh1_normals = mesh1.getNormals()
    assert mesh1_normals is not None
    assert len(mesh1_normals) ==  17
    mesh2_normals = mesh2.getNormals()
    assert mesh2_normals is not None
    assert len(mesh2_normals) == 17
    mesh3_normals = mesh3.getNormals()
    assert mesh3_normals is not None
    assert len(mesh3_normals) == 4

    mesh0_material = mesh0.getMaterial()
    nptest.assert_allclose(mesh0_material.getBaseColorFactor().flatten(),[0.7,0.7,0.7,1], atol=0.01)
    nptest.assert_almost_equal(mesh0_material.getMetallicFactor(), 0.0)
    nptest.assert_almost_equal(mesh0_material.getRoughnessFactor(), 0.5)
    nptest.assert_allclose(mesh0_material.getEmissiveFactor().flatten(), [0,0,0,1], atol = 0.01)

    mesh1_material = mesh1.getMaterial()
    nptest.assert_allclose(mesh1_material.getBaseColorFactor().flatten(),[0.8,0.0,0.0,1], atol=0.01)
    nptest.assert_almost_equal(mesh1_material.getMetallicFactor(), 0.0)
    nptest.assert_almost_equal(mesh1_material.getRoughnessFactor(), 0.5)
    nptest.assert_allclose(mesh1_material.getEmissiveFactor().flatten(), [0,0,0,1], atol = 0.01)

    mesh2_material = mesh2.getMaterial()
    nptest.assert_allclose(mesh2_material.getBaseColorFactor().flatten(),[0.05,0.8,0.05,1], atol=0.01)
    nptest.assert_almost_equal(mesh2_material.getMetallicFactor(), 0.0)
    nptest.assert_almost_equal(mesh2_material.getRoughnessFactor(), 0.5)
    nptest.assert_allclose(mesh2_material.getEmissiveFactor().flatten(), [0.1,0.1,0.5,1], atol = 0.01)

    mesh3_material = mesh3.getMaterial()
    nptest.assert_allclose(mesh3_material.getBaseColorFactor().flatten(),[1,1,1,1], atol=0.01)
    nptest.assert_almost_equal(mesh3_material.getMetallicFactor(), 0.0)
    nptest.assert_almost_equal(mesh3_material.getRoughnessFactor(), 0.5)
    nptest.assert_allclose(mesh3_material.getEmissiveFactor().flatten(), [0,0,0,1], atol = 0.01)

    assert mesh0.getTextures() is None
    assert mesh1.getTextures() is None
    assert  mesh2.getTextures() is None

    assert mesh3.getTextures() is not None
    assert len(mesh3.getTextures()) == 1

    texture = mesh3.getTextures()[0]
    assert len(texture.getTextureImage().getResourceContents()) == 38212
    assert len(texture.getUVs()) == 4

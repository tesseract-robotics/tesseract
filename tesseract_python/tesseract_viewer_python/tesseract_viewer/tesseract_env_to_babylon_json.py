# Copyright 2019 Wason Technology, LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: John Wason (wason@wasontech.com)
# Date: 12/10/2019

import json
import numpy as np
import math
import tesseract
import pkgutil

def tesseract_env_to_babylon_json(t_env, origin_offset=[0,0,0]):
    return json.dumps(tesseract_env_to_babylon_json_dict(t_env,origin_offset))

def tesseract_env_to_babylon_json_dict(t_env, origin_offset=[0,0,0]):

    assert len(list(origin_offset)) == 3

    link_names = t_env.getLinkNames()
    joint_names = t_env.getJointNames()

    link_map = dict()
    joint_map = dict()
    for l in link_names:
        link_map[l] = t_env.getLink(l)

    for j in joint_names:
        joint_map[j] = t_env.getJoint(j)

    root_link_name = t_env.getRootLinkName()

    transform_nodes, meshes, materials = _process_link_recursive(link_map, joint_map, root_link_name, None)

    transform_nodes.append({"name": "root", "id": "root", "isVisible": "true", "isEnabled": "true", "position": list(origin_offset), "parentId": "root0"})

    geometries = json.loads(pkgutil.get_data("tesseract_viewer.resources","geometries.json"))["geometries"]
    
    babylon_dict = {"geometries": geometries, "transformNodes": transform_nodes, "meshes": meshes, "materials": materials}
    return babylon_dict


def _find_child_joints(joint_map, parent_link_name):
    return [j for j in joint_map.values() if j.parent_link_name == parent_link_name]

def _np_transform_to_babylon(np_tf):
    p = np_tf[0:3,3]
    q0 = _R2q(np_tf[0:3,0:3])
    q = np.array([q0[1], q0[2], q0[3], q0[0]])
    return list(p),list(q)

def _process_link_recursive(link_map, joint_map, link_name, parent_joint_name):
    transform_nodes = []
    meshes = []
    materials = []
    link = link_map[link_name]

    tf_link = {"name": "link_" + link_name, "id": "link_" + link_name, "isVisible": "true", "isEnabled": "true"}
    if (parent_joint_name is not None):
        tf_link["parentId"] = "joint_" + parent_joint_name
    else:
        tf_link["parentId"] = "root"
    transform_nodes.append(tf_link)
    
    for visual in link.visual:
        visual_name = "link_" + link_name + "_visual_" + visual.name
        np_tf_visual = visual.origin
        visual_p, visual_q = _np_transform_to_babylon(np_tf_visual)
        tf_visual = {"name": visual_name, "isVisible": "true", "isEnabled": "true", 
            "parentId": "link_" + link_name, "materialId": "material_" + visual_name}
        tf_visual["position"] = visual_p
        tf_visual["rotationQuaternion"] = visual_q
        tf_visual["billboardMode"] = 0

        visual_geom = visual.geometry
        if (isinstance(visual_geom,tesseract.Mesh)):
        
            mesh=visual_geom
            positions = list(np.concatenate(list(mesh.getVertices())).flatten())
            triangles = mesh.getTriangles().flatten()
            triangle_count = int(len(triangles)/4)
            indices = [0]*(triangle_count*3)
            for i in range(triangle_count):
                assert(triangles[i*4]==3, "Only triangle meshes supported")
                indices[i*3] = int(triangles[i*4+3])
                indices[i*3+1] = int(triangles[i*4+2])
                indices[i*3+2] = int(triangles[i*4+1])

            tf_visual["positions"] = positions
            tf_visual["indices"] = indices
            
            tf_visual["scaling"] = list(mesh.getScale().flatten())
            tf_visual["normals"] = [0,0,1]*int(len(tf_visual["positions"])/3)
        
            submesh = {}
            submesh["materialIndex"] = 0
            submesh["verticesStart"] = 0
            submesh["verticesCount"] = len(positions)/3
            submesh["indexStart"] = 0
            submesh["indexCount"] = len(indices)
            tf_visual["subMeshes"] = [submesh]
            
            
        elif (isinstance(visual_geom,tesseract.Box)):
            box=visual_geom
            tf_visual["geometryId"] = "cube_geometry"
            tf_visual["scaling"] = [0.5*box.getX(), 0.5*box.getY(), 0.5*box.getZ()]
            submesh = {}
            submesh["materialIndex"] = 0
            submesh["verticesStart"] = 0
            submesh["verticesCount"] = 30
            submesh["indexStart"] = 0
            submesh["indexCount"] = 36
            tf_visual["subMeshes"] = [submesh]

        elif (isinstance(visual_geom,tesseract.Sphere)):
            sphere=visual_geom
            tf_visual["geometryId"] = "sphere_geometry"
            tf_visual["scaling"] = [sphere.getRadius(), sphere.getRadius(), sphere.getRadius()]
            submesh = {}
            submesh["materialIndex"] = 0
            submesh["verticesStart"] = 0
            submesh["verticesCount"] = 2592
            submesh["indexStart"] = 0
            submesh["indexCount"] = 2880
            tf_visual["subMeshes"] = [submesh]

        elif (isinstance(visual_geom,tesseract.Cylinder)):
            cylinder=visual_geom
            tf_visual["geometryId"] = "cylinder_geometry"
            tf_visual["scaling"] = [cylinder.getRadius(), cylinder.getRadius(), 0.5*cylinder.getLength()]
            submesh = {}
            submesh["materialIndex"] = 0
            submesh["verticesStart"] = 0
            submesh["verticesCount"] = 309
            submesh["indexStart"] = 0
            submesh["indexCount"] = 372
            tf_visual["subMeshes"] = [submesh]

            
        else:
            #Unsupported geometry type
            continue

        meshes.append(tf_visual)

        tf_material = {"name": "material_" + visual_name}

        material = visual.material

        if material is None:
            tf_color = [0.5,0.5,0.5]
        else:
            color = material.color.flatten()
            tf_color = [color[0], color[1], color[2]]

        tf_material["ambient"] = tf_color
        tf_material["diffuse"] = tf_color
        tf_material["specular"] = tf_color
        tf_material["alpha"] = 1
        tf_material["backFaceCulling"] = True

        materials.append(tf_material)


    child_joints = _find_child_joints(joint_map, link_name)
    for j in child_joints:
        np_tf_joint = j.parent_to_joint_origin_transform
        joint_p, joint_q = _np_transform_to_babylon(np_tf_joint)
        tf_joint_parent = {"name": "jointparent_" + j.getName(), "isVisible": "true", "isEnabled": "true", "parentId": "link_" + link_name}
        tf_joint_parent["position"] = joint_p
        tf_joint_parent["rotationQuaternion"] = joint_q
        transform_nodes.append(tf_joint_parent)
        tf_joint = {"name": "joint_" + j.getName(), "isVisible": "true", "isEnabled": "true", "parentId": "jointparent_" + j.getName()}
        tf_joint["metadata"] = {"tesseract_joint": {"axis": list(j.axis.flatten()), "type": int(j.type), "name": j.getName()}}
        transform_nodes.append(tf_joint)
        j_transform_nodes, j_meshes, j_materials = _process_link_recursive(link_map, joint_map, j.child_link_name, j.getName())
        transform_nodes.extend(j_transform_nodes)
        meshes.extend(j_meshes)
        materials.extend(j_materials)
        
    return transform_nodes, meshes, materials


def _R2q(R):
    """
    Converts a 3 x 3 rotation matrix into a quaternion.  Quaternion is
    returned in the form q = [q0;qv].
    
    :type    R: numpy.array
    :param   R: 3 x 3 rotation matrix
    :rtype:  numpy.array
    :return: the quaternion as a 4 x 1 vector q = [q0;qv] 
      
    """
    
    tr = np.trace(R)
    if tr > 0:
        S = 2*math.sqrt(tr + 1)
        q = np.array([(0.25*S), \
                      ((R[2,1] - R[1,2]) / S), \
                      ((R[0,2] - R[2,0]) / S), \
                      ((R[1,0] - R[0,1]) / S)])
                      
    elif (R[0,0] > R[1,1] and R[0,0] > R[2,2]):
        S = 2*math.sqrt(1 + R[0,0] - R[1,1] - R[2,2])
        q = np.array([((R[2,1] - R[1,2]) / S), \
                      (0.25*S), \
                      ((R[0,1] + R[1,0]) / S), \
                      ((R[0,2] + R[2,0]) / S)])
    elif (R[1,1] > R[2,2]):
        S = 2*math.sqrt(1 - R[0,0] + R[1,1] - R[2,2])
        q = np.array([((R[0,2] - R[2,0]) / S), \
                      ((R[0,1] + R[1,0]) / S), \
                      (0.25*S), \
                      ((R[1,2] + R[2,1]) / S)])
    else:
        S = 2*math.sqrt(1 - R[0,0] - R[1,1] + R[2,2])
        q = np.array([((R[1,0] - R[0,1]) / S), \
                      ((R[0,2] + R[2,0]) / S), \
                      ((R[1,2] + R[2,1]) / S), \
                      (0.25*S)])
    return q

'''Stack-relationship detector using Ray Casting/Tracing
Intuition: In order to understand which objects are placed on top of which other objects, we can 
simply cast a one ray from each point of a given object vertically downward and among the set of objects 
that are hit by these rays, find the one that is closest to the current object. 

Functionality:
Multiple objects can be under a given object, so we shall maintain a stack for each object which tells 
us which objects are to be placed before we place this object in its destination.

NOTE: RAY CASTING exists only in open3d version>=0.14.0 (Here, it is being tested with v0.15.2 of open3d)

Author: Vishal Reddy Mandadi
'''

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from misc import Object
from rendering_scenes_in_o3d import get_object_list_from_yaml, save_pcd, draw_geometries
import copy

class SGNode:
    def __init__(self, object_name, parent):
        '''
        object_name: Current object name
        parent: Link to parent's node
        '''
        self.object = object_name
        self.parent = parent

def generate_scene_graph_from_object_dict(object_dict, mesh_dir='/home/vishal/Volume_E/Active/Undergrad_research/Ocrtoc/OCRTOC_software_package/ocrtoc_materials/models/'):
    '''Generate scene graph for the given object labels and poses
    Structure:
    level 0:                    scene
      on                          |
    level 1:          __________table____________
      on              |       |        |        |
    level 2:       __obj1_ _obj2_    _obj3_   _obj4_                         
      on           |     | |    |    |    |   |    |
    level 3:  ...

    Parameters:
    object_dict: {
        object_name: object_6D_pose ([x, y, z, roll, pitch, yaw])
    }
    '''
    scene = o3d.t.geometry.RaycastingScene()

    object_info_dict = {}
    mesh_id_dict = {}

    meshes = []
    pcds =[]

    colors = [[0, 0, 0], [0.30, 0, 0], [0, 0.3, 0], [0, 0, 0.3], [0.3, 0.3, 0], [0.3, 0, 0.3]]
    counter = -1

    for i, key in enumerate(object_dict):
        for object_pose in object_dict[key]:
            counter += 1
            mesh = o3d.io.read_triangle_mesh('{}{}/textured.obj'.format(mesh_dir, str(key)))
            mesh2 = o3d.io.read_triangle_mesh('{}{}/textured.obj'.format(mesh_dir, str(key)))
            # pcd = mesh.sample_points_poisson_disk(number_of_points=2000, init_factor=5, pcl=None)
            # object_pose = object_dict[key][0]
            R = mesh.get_rotation_matrix_from_xyz((object_pose[3], object_pose[4], object_pose[5]))
            # print("{} initial mesh center: {}".format(key, mesh.get_center()))
            center = np.array(mesh.get_center())
            # Rotate in place about its own center
            mesh.rotate(R, center=(center[0], center[1], center[2]))
            # o3d.visualization.draw_geometries([mesh, mesh2])
            # Translate the mesh to the desired position
            required_pos = np.array([object_pose[0], object_pose[1], object_pose[2]])
            dt = required_pos - center
            mesh.translate((dt[0], dt[1], dt[2]))

            pcd = mesh.sample_points_poisson_disk(number_of_points=50, init_factor=5, pcl=None)

            pcd.paint_uniform_color(colors[counter])
            mesh.paint_uniform_color(colors[counter])

            meshes.append(mesh)
            pcds.append(pcd)

            mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

            mesh_id = scene.add_triangles(mesh)
            print("Object name: {}\tpose: {}\tMesh id: {}".format(key, object_pose, mesh_id))

            info_dict = {
                'object': key,
                'mesh_id': mesh_id,
                'pcd': np.asarray(pcd.points)
            }
            object_info_dict[str(mesh_id)] = info_dict
            mesh_id_dict[str(mesh_id)] = {
                'object': key,
                'pose': object_pose,
            }

    o3d.visualization.draw_geometries(meshes)
    o3d.visualization.draw_geometries(pcds)

    ray_direction = [0, 0, -1]

    # Now get the object/objects under each object
    object_stacks = {}
    ray_list = []
    for i, key in enumerate(object_info_dict):
        # if i<=2: 
        #     continue
        print("Object: {}\tmesh id{}".format(key, object_info_dict[key]['object']))
        pcd = object_info_dict[key]['pcd']
        ray_list = []
        n_pts = len(pcd)
        for j in range(n_pts):
            x, y, z = pcd[j, :]
            ray_tensor = [x, y, z, ray_direction[0], ray_direction[1], ray_direction[2]]
            ray_list.append(ray_tensor)
            # x, y, z = position[0], position[1], position[2]
            # print("Position: {}".format([x, y, z]))
            # break
        # break
        rays = o3d.core.Tensor(ray_list,
                       dtype=o3d.core.Dtype.Float32)
        ans = scene.cast_rays(rays)
        # print(ans)
        geometry_ids_init = list(set(ans['geometry_ids'].numpy())) # Contain mesh ids of the objects that the rays hit
        # print(type(geometry_ids_init))
        # print(geometry_ids_init)
        geometry_ids = []
        for g_id in geometry_ids_init:
            if g_id <= len(object_info_dict) and g_id >= 0:
                geometry_ids.append(g_id)
        
        # exit()
        object_stacks[key] = {
            'current_object_info': {
                'object': object_info_dict[key]['object'],
                'mesh_id': object_info_dict[key]['mesh_id']
            },
            'mesh_ids_of_objects_under_it': geometry_ids
        }
        # print("Object name: {}\tObject mesh id: {}\nObject stack list: {}\n".format(object_info_dict[key]['object'], object_info_dict[key]['mesh_id'], geometry_ids))

    # rays = o3d.core.Tensor(ray_list,
    #                dtype=o3d.core.Dtype.Float32)
    # ans = scene.cast_rays(rays)
    # geometry_ids_init = ans['geometry_ids'].numpy()
    # n_rays_per_object = 50
    # for i, key in enumerate(object_info_dict):
    #     geometry_ids_temp = list(set(geometry_ids_init[i*n_rays_per_object: (i+1)*n_rays_per_object]))
    #     geometry_ids = []
    #     for g_id in geometry_ids_temp:
    #         if g_id <= len(object_info_dict) and g_id >= 0:
    #             geometry_ids.append(g_id)
    #     object_stacks[key] = {
    #         'current_object_info': {
    #             'object': object_info_dict[key]['object'],
    #             'mesh_id': object_info_dict[key]['mesh_id']
    #         },
    #         'mesh_ids_of_objects_under_it': geometry_ids
    #     }
    #     print("Object name: {}\tObject mesh id: {}\nObject stack list: {}\n".format(object_info_dict[key]['object'], object_info_dict[key]['mesh_id'], geometry_ids))


    # print("Mesh id dict: {}\n".format(mesh_id_dict))
    # print("Object dict: {}\n".format(object_dict))
    print("Object stacking info dict: {}\n".format(object_stacks))
    



if __name__=='__main__':
    scene_dir = '/home/vishal/Volume_E/Active/Undergrad_research/Ocrtoc/OCRTOC_software_package/ocrtoc_materials/targets' # './final_scenes'
    mesh_dir = '/home/vishal/Volume_E/Active/Undergrad_research/Ocrtoc/OCRTOC_software_package/ocrtoc_materials/models/'
    scenes = ['1-1-1', '2-2-2', '5-3-1', '6-1-2']
    yaml_paths = []
    for scene in scenes:
        yaml_paths.append('{}/{}.yaml'.format(scene_dir, scene))
    current_scene_index = 1

    object_dict = get_object_list_from_yaml(yaml_paths[current_scene_index])

    generate_scene_graph_from_object_dict(object_dict, mesh_dir=mesh_dir)

    # meshes = []
    # pcd_dict = {}

    # object_info_dict = {}

    # # Ray casting scene initialization
    # scene = o3d.t.geometry.RaycastingScene()

    # for i, key in enumerate(object_dict):
    #     for object_pose in object_dict[key]:
    #         mesh = o3d.io.read_triangle_mesh('/home/vishal/Volume_E/Active/Undergrad_research/Ocrtoc/OCRTOC_software_package/ocrtoc_materials/models/{}/textured.obj'.format(str(key)))
    #         mesh2 = o3d.io.read_triangle_mesh('/home/vishal/Volume_E/Active/Undergrad_research/Ocrtoc/OCRTOC_software_package/ocrtoc_materials/models/{}/textured.obj'.format(str(key)))
    #         # pcd = mesh.sample_points_poisson_disk(number_of_points=2000, init_factor=5, pcl=None)
    #         # object_pose = object_dict[key][0]
    #         R = mesh.get_rotation_matrix_from_xyz((object_pose[3], object_pose[4], object_pose[5]))
    #         # print("{} initial mesh center: {}".format(key, mesh.get_center()))
    #         center = np.array(mesh.get_center())
    #         # Rotate in place about its own center
    #         mesh.rotate(R, center=(center[0], center[1], center[2]))
    #         # o3d.visualization.draw_geometries([mesh, mesh2])
    #         # Translate the mesh to the desired position
    #         required_pos = np.array([object_pose[0], object_pose[1], object_pose[2]])
    #         dt = required_pos - center
    #         mesh.translate((dt[0], dt[1], dt[2]))

    #         pcd = mesh.sample_points_poisson_disk(number_of_points=50, init_factor=5, pcl=None)

    #         mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

    #         mesh_id = scene.add_triangles(mesh)
    #         print("Object name: {}\tpose: {}\tMesh id: {}".format(key, object_pose, mesh_id))

    #         info_dict = {
    #             'object': key,
    #             'mesh_id': mesh_id,
    #             'pcd': np.asarray(pcd.points)
    #         }
    #         object_info_dict[key] = info_dict

    # ray_direction = [0, 0, -1]

    # ans = None

    # for i, key in enumerate(object_info_dict):
    #     if i==0: 
    #         continue
    #     print("Object: {}\tmesh id{}".format(key, object_info_dict[key]['mesh_id']))
    #     pcd = object_info_dict[key]['pcd']
    #     ray_list = []
    #     n_pts = len(pcd)
    #     for i in range(n_pts):
    #         x, y, z = pcd[i, :]
    #         ray_tensor = [x, y, z, ray_direction[0], ray_direction[1], ray_direction[2]]
    #         ray_list.append(ray_tensor)
    #         # x, y, z = position[0], position[1], position[2]
    #         # print("Position: {}".format([x, y, z]))
    #         # break
    #     # break
    #     rays = o3d.core.Tensor(ray_list,
    #                    dtype=o3d.core.Dtype.Float32)
    #     ans = scene.cast_rays(rays)

    #     break

    # # print(ans)
    # t_hits = ans['t_hit']
    # geometry_ids = ans['geometry_ids']
    # print("T_hits: {}".format(t_hits))
    # print("Geometry ids: {}".format(geometry_ids))
    # # import matplotlib.pyplot as plt
    # plt.imshow(ans['primitive_uvs'].numpy())
    # plt.show()
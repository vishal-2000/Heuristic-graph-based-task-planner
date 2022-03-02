'''Optimal Search methods
This file contains various gradient-free search methods for searching the best buffer position

List of implemented methods:
1. Marching-grid algorithm - https://www.youtube.com/watch?v=CqhCHE5s13k
'''
import open3d as o3d
import copy
import numpy as np
import cv2
from buffer_detection import generate_2D_occupancy_map
from misc import Object
import time

def get_occupancy_percentage(target_pose_6D, scene_omap, entity):
    '''Gets the percentage of target space that is occupied
    '''
    entity_pcd = entity.render_to_pose_and_get_pcd(object_pose=target_pose_6D)
    entity_omap = generate_2D_occupancy_map(world_dat = np.asarray(entity_pcd.points)*100, xy_min_max=[-30, 30, -60, 60], threshold=1, dir_path='./results/object_{}-{}.png'.format('2-2-2', 'green_bowl'), save=False)
    if len(entity_omap) == 0:
        return 100
    occupancy = np.logical_and(scene_omap, entity_omap)
    # print("Occupancy shape: {}".format(occupancy.shape))
    total_occupied_space_of_object = np.sum(entity_omap)
    occ_percent = float(np.sum(occupancy)*100.0)/(total_occupied_space_of_object)

    return occ_percent

def sample_8_pts_around_given_point(given_pt: list, step=0.1):
    '''Samples 9 points symmetrically in a square fashion around the given point in sample plane (z unchanged)
    xxx
    xox
    xxx

    Here, 'x' denote the sampled point, while 'o' denotes the given point

    Parameters:
    given_pt: list (1, 3): given point's 3D position
    step: size of the step

    Return:
    pts = list (9, 3) # Includes the current point
    '''
    pts = []
    for i in range(-1, 2, 1):
        for j in range(-1, 2, 1):
            # print(given_pt)
            x = given_pt[0]+(i*step)
            y = given_pt[1]+(j*step)
            z = given_pt[2]
            # print(x, y, z)
            new_pt = [x, y, z]
            # print(new_pt)
            pts.append(new_pt)
    return pts

def marching_grid(scene_pcd, object_mesh_path:str, target_pose_6D: list, OCC_THRESH=0.0, scene_name='-', object_name='-'):
    '''Marching Grid algorithm
    Returns the closest empty buffer space to the given target using marching grid algorithm

    Parameters:
    scene_pcd: Point cloud of the 3D scene
    object_mesh_path: Path to the mesh file of the object
    target_position_3D: list [x, y, z, roll, pitch, yaw] - Target 6D pose of object in world frame
    OCC_THRESH: The maximum % for which a pose is considered to be free

    Return:
    buffer_spot: np.ndarray [x, y, z, roll, pitch, yaw] in world frame (same frame as the given world_data and 
                    target_position 3D)
    '''
    scene_omap = generate_2D_occupancy_map(np.asarray(scene_pcd.points)*100, xy_min_max=[-30, 30, -60, 60], threshold=1, dir_path='./results/{}-{}.png'.format(scene_name, object_name), save=False) # xy_min_max=[-29.97, 29.97, -59.96, 59.99]
    entity = Object(mesh_path=object_mesh_path)

    occ_percent = get_occupancy_percentage(target_pose_6D=target_pose_6D, scene_omap=scene_omap, entity=entity)
    print("Occupancy %: {}%".format(occ_percent))
    if occ_percent<=OCC_THRESH:
        print("Found!")
        return target_pose_6D


    done = False
    search_step = 0.1
    current_position = [target_pose_6D[0], target_pose_6D[1], target_pose_6D[2]]
    prev_min_val = 100
    n_steps = 3
    while not done and n_steps>0:
        n_steps = n_steps - 1
        sampled_pts = sample_8_pts_around_given_point(given_pt=current_position, step=search_step)
        occ_percents = []
        for i, pt in enumerate(sampled_pts):
            pose_6d = [pt[0], pt[1], pt[2], target_pose_6D[3], target_pose_6D[4], target_pose_6D[5]]
            occ_percent = get_occupancy_percentage(target_pose_6D=pose_6d, scene_omap=scene_omap, entity=entity)
            occ_percents.append(occ_percent)
        print(occ_percents)
        
        min_val = min(occ_percents)
        min_index = occ_percents.index(min_val)
        print("min_index: {}\tmin_val: {}".format(min_index, min_val))
        if min_val <= OCC_THRESH:
            print("Min pose found")
            done = True
            current_position = sampled_pts[min_index]
        elif min_val==prev_min_val:
            current_position = sampled_pts[min_index]
            search_step = search_step/2
        elif min_val > prev_min_val:
            search_step = search_step/2

    buffer_spot = [current_position[0], current_position[1], current_position[2], 
                target_pose_6D[3], target_pose_6D[4], target_pose_6D[5]]
    print(buffer_spot)

    return buffer_spot


    

if __name__=='__main__':
    start_time = time.time()

    scene_pcd_path = './2-2-2_final.pcd'
    pcd = o3d.io.read_point_cloud(scene_pcd_path)
    object_mesh_path = '/home/vishal/Volume_E/Active/Undergrad_research/Ocrtoc/OCRTOC_software_package/ocrtoc_materials/models/{}/textured.obj'.format('green_bowl')
    target_pose_6D = [0.1, 0.35, 0.03, 0.0, -0.01, 1.57]
    buffer_spot = marching_grid(scene_pcd=pcd, object_mesh_path=object_mesh_path, target_pose_6D=target_pose_6D, scene_name='2-2-2', object_name='green_bowl')

    end_time = time.time()
    print("Time taken: {}".format(end_time - start_time))

    # Visualize the result
    entity = Object(mesh_path=object_mesh_path)
    entity_pcd = entity.render_to_pose_and_get_pcd(object_pose=buffer_spot)
    entity_omap = generate_2D_occupancy_map(world_dat = np.asarray(entity_pcd.points)*100, xy_min_max=[-30, 30, -60, 60], threshold=1, dir_path='./results/object_{}-{}.png'.format('2-2-2', 'green_bowl'), save=True)
    scene_omap = generate_2D_occupancy_map(np.asarray(pcd.points)*100, xy_min_max=[-30, 30, -60, 60], threshold=1, dir_path='./results/{}-{}.png'.format('2-2-2', 'green_bowl'), save=True)
    fused_omap = np.logical_or(entity_omap, scene_omap)
    cv2.imwrite('./results/buffer_{}-{}.png'.format('2-2-2', 'green_bowl'),(fused_omap * 255).astype(np.uint8))
    pass

from matplotlib.pyplot import get
import open3d as o3d
import numpy as np
import cv2
import time

def convert_to_occupancy_map(world_dat, threshold=3, dir_path='./results/occ_map.png'):
    '''
    Given the data in world frame of reference, this function produces 2D occupancy map
    (No bayesian update rules are considered as the assignment asks us to leave it)
    Note: Represent each point in (x, z) plane as a pixel. Strategy: We first round off 
    all the coordinates as pixels in images have integral coordinates. Now, we count number 
    of points for each pixel. If the number of points > threshold, then we set that pixel as
    occupied. Threshold is introduced to mark the difference between ground plane points and 
    actual obstacle points.

    Input: (np.array (x*3) data in world frame), (threshold - min number points to consider a pixel as occupied)
    Return: np.array of occupancy grid (0-> pixel empty, 1-> pixel occupied)
    '''
    x_range = int(np.round(np.max(world_dat[:, 0]))-np.round(np.min(world_dat[:, 0])))
    y_range = int(np.round(np.max(world_dat[:, 1]))-np.round(np.min(world_dat[:, 1])))
    # Since, pixels have integral coordinates, let us round off all the values in world_dat and remove y coordinates
    world_dat_rounded = np.round(world_dat)
    world_dat_rounded = (np.delete(world_dat_rounded, 2, 1)).astype(int) # Remove y coordinates column

    x_min = (np.round(np.min(world_dat[:, 0]))).astype(int)
    y_min = (np.round(np.min(world_dat[:, 1]))).astype(int)

    pixel_counts = np.zeros(shape=(x_range+1, y_range+1), dtype=float)
    count = 0
    for point in world_dat_rounded:
        #print(point-np.array([x_min, z_min], dtype=int))
        pixel_counts[point[0]-x_min, point[1]-y_min]+=1

    mean_points = 1# np.mean(pixel_counts)
    occ_map = np.zeros(shape=pixel_counts.shape, dtype=np.uint8)
    for i in range(x_range):
        for j in range(y_range):
            if pixel_counts[i, j] >= threshold:
                occ_map[i, j] = 1 # Occupied

    cv2.imwrite(f'{dir_path}',(occ_map * 255).astype(np.uint8))

    # print(pixel_counts)

    return occ_map

def generate_2D_occupancy_map(world_dat, xy_min_max = [], threshold=3, dir_path='./results/occ_map.png', save=True):
    '''
    A non-traditional way to mark occupied areas on a grid. In this method, we simply look for 
    areas with z>threshold (threshold~0) and mark them as occupied. This is expected to be highly
    effective for OCRTOC
    '''
    print("world_dat_min_max_x: {} - {}".format(np.min(world_dat[:, 0]), np.max(world_dat[:, 0])))
    print("world_dat_min_max_y: {} - {}".format(np.min(world_dat[:, 1]), np.max(world_dat[:, 1])))
    if len(xy_min_max)==0:
        x_range = int(np.round(np.max(world_dat[:, 0]))-np.round(np.min(world_dat[:, 0])))
        y_range = int(np.round(np.max(world_dat[:, 1]))-np.round(np.min(world_dat[:, 1])))
    else:
        if np.max(world_dat[:, 0]) > 30 or np.min(world_dat[:, 0]) < -30 or np.max(world_dat[:, 1]) > 60 or np.min(world_dat[:, 1]) < -60:
            print("yes!")
            return []
        x_range = int(np.round(xy_min_max[1]) - np.round(xy_min_max[0]))
        y_range = int(np.round(xy_min_max[3]) - np.round(xy_min_max[2]))
    # Since, pixels have integral coordinates, let us round off all the values in world_dat and remove y coordinates
    world_dat_rounded = (np.round(world_dat)).astype(int)
    # world_dat_rounded = (np.delete(world_dat_rounded, 2, 1)).astype(int) # Remove y coordinates column

    if len(xy_min_max)==0:
        x_min = (np.round(np.min(world_dat[:, 0]))).astype(int)
        y_min = (np.round(np.min(world_dat[:, 1]))).astype(int)
    else:
        x_min = (np.round(xy_min_max[0])).astype(int)
        y_min = (np.round(xy_min_max[2])).astype(int)

    pixel_counts = np.zeros(shape=(x_range+1, y_range+1), dtype=float)
    count = 0
    for point in world_dat_rounded:
        #print(point-np.array([x_min, z_min], dtype=int))
        if point[2] > 0:
            pixel_counts[point[0]-x_min, point[1]-y_min]+=1

    mean_points = 1# np.mean(pixel_counts)
    occ_map = np.zeros(shape=pixel_counts.shape, dtype=np.uint8)
    for i in range(x_range):
        for j in range(y_range):
            if pixel_counts[i, j] >= threshold:
                occ_map[i, j] = 1 # Occupied

    if save==True:
        cv2.imwrite(f'{dir_path}',(occ_map * 255).astype(np.uint8))

    # print(pixel_counts)

    return occ_map

def visualize_pcd_with_global_coordinate_frame(pcd):
    mesh_frame1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    mesh_frame2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[-0.3, -.6, 0])
    mesh_frame3 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[.3, -.6, 0])
    mesh_frame4 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[.3, .6, 0])
    mesh_frame5 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[-.3, .6, 0])
    o3d.visualization.draw_geometries([pcd, mesh_frame1, mesh_frame2, mesh_frame3, mesh_frame4, mesh_frame5])

def get_closest_to_naive(empty_spots, target_pos = np.array([.0, .0]), collision_diameter=0.2):
    distances = np.linalg.norm(empty_spots - target_pos, axis=1) 
    valid_empties = []
    valid_distances = []
    for i in range(len(empty_spots)):
        if distances[i] > collision_diameter:
            valid_empties.append(empty_spots[i])
            valid_distances.append(distances[i])
    # print("Valid distances: {}".format(valid_distances))
    closest_ind = np.argmin(np.array(valid_distances))
    # print("Closest_ind: {}".format(closest_ind))
    # print("Closest vector: {}".format(valid_empties[closest_ind]))
    # print(distances)
    # print("Valid empties: {}".format(valid_empties))
    return valid_empties[closest_ind]

def get_empty_spot(pcd, debug=False):
    '''
    First get occupancy grid for the given point cloud. Now, use the coordinates of unoccupied cells
    as buffers (scale them down and transform them approximately).
    '''
    if debug==True:
        visualize_pcd_with_global_coordinate_frame(pcd)
    occ_map = generate_2D_occupancy_map(np.asarray(pcd.points)*100, threshold=1, dir_path='./results/occ_map=2-2-2.png')

    if  debug==True:
        print("occ_map shape: {}".format(occ_map.shape))
    
    # Show occ_map
    if debug==True:
        cv2.imshow("Octomap", occ_map*255)
        cv2.waitKey(0)

    # convert_to_occupancy_map(np.asarray(pcd.points)*100, threshold=500, dir_path='./results/occ_map_thr=1.png')
    # x_limits = [-.3, .3]
    # y_limits = [-.6, .6]

    # Generating empty spots (random sampling)
    x_lims, y_lims = occ_map.shape
    zero_coords = np.where(occ_map == 0)
    # print(zero_coords, type(zero_coords))
    empty_spots = np.zeros(shape=(len(zero_coords[0]), 2), dtype=float)
    empty_spots[:, 0] = zero_coords[0]
    empty_spots[:, 1] = zero_coords[1]
    empty_spots = (empty_spots/100) - np.array([0.3, 0.6], dtype=float)
    # print(empty_spots, type(empty_spots), empty_spots.shape)

    closest_empty_spot = get_closest_to_naive(empty_spots)

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[closest_empty_spot[0], closest_empty_spot[1], 0])
    if debug==True:
        o3d.visualization.draw_geometries([pcd, mesh_frame])




    

    

if __name__=="__main__":
    start_time = time.time()
    pcd = o3d.io.read_point_cloud("./2-2-2.pcd")
    get_empty_spot(pcd)
    end_time = time.time()
    print("Time taken: {}".format(end_time - start_time))
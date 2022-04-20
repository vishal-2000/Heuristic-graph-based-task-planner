'''Modify the predicted grasp poses using the heightmap information

Get the height map of the scene that is focussed on the current object (remove other parts). Check
various grasp positions and orientations that are closest to the object using collision_map and 
validity_maps for the gripper. Then choose the best option among them.

Author: Vishal Reddy Mandadi
'''

import numpy as np
from PIL import Image
from time import sleep
import cv2
import open3d as o3d
# from sklearn.covariance import graphical_lasso

class GraspModification:
    def __init__(self, search_space_diameter = 20):
        '''Grasp modification class

        Modifies or finds a better grasp at a given location
        search_space_diameter: the max width and length of the heigtmap of the scene, to be searched in for
        '''
        self.ee_length = 14
        self.ee_width = 4
        self.finger_max_dist = 8
        self.finger_len = 1 # Length of the face of finger facing the ground
        self.finger_bre = 1 # Breadth of the face of the finger facing the ground
        self.finger_height = 8
        self.max_penetration_len = 4 # Max amount of penetration of the object between the two fingers
    def get_gripper_standard_collision_map_1cm_resolution(self):
        '''Gets the gripper collision map that can be used to evaluate collisions at different points. 
        The produced map is for the gripper in its canonical/standard pose (it's length parellel to 
        the length of the table)

        The resolution of the map -> 1 pixel occupies 1cm*1cm square box in the real world
        '''
        collision_map = np.full(shape=(self.ee_length, self.ee_length), fill_value=10) 
        for i in range(int(self.ee_length//2 - self.ee_width//2), int(self.ee_length//2 + self.ee_width//2)):
            collision_map[i:i+1] = 0
            # print("ee - {}".format(i))
            if i == (self.ee_length//2 - 1) or i == self.ee_length//2:
                collision_map[i, 2] = -1*self.finger_height # self.max_penetration_len # or use self.finger_height, if full penetration is allowed
                collision_map[i, 11] = -1*self.finger_height # self.max_penetration_len
                collision_map[i, 3:11] = -1*self.max_penetration_len # (not allowing any object to penetrate more than 4 cm towards the EE, between the fingers)
                # print("finger - {}".format(i))
            
        print("Final collision map:\n{}".format(collision_map))
        return collision_map

    def get_gripper_standard_collision_map_5mm_resolution(self):
        '''Gets the gripper collision map that can be used to evaluate collisions at different points. 
        The produced map is for the gripper in its canonical/standard pose (it's length parellel to 
        the length of the table)

        The resolution of the map -> 1 pixel occupies 0.5cm*0.5cm square box in the real world
        '''
        collision_map = np.full(shape=(self.ee_length*2, self.ee_length*2), fill_value=10) 
        for i in range(int(2*self.ee_length//2 - 2*self.ee_width//2), int(2*self.ee_length//2 + 2*self.ee_width//2)):
            collision_map[i:i+1] = 0
            print("ee - {}".format(i))
            # print(range((2*self.ee_length//2 - 2),  2*self.ee_length//2+2))
            if i in range((2*self.ee_length//2 - 2),  2*self.ee_length//2+2):
                collision_map[i, 4:6] = -1*self.finger_height# self.max_penetration_len # or use self.finger_height, if full penetration is allowed
                collision_map[i, 22:24] = -1*self.finger_height # self.max_penetration_len
                print("finger - {}".format(i))
            
        print("Final collision map:\n{}".format(collision_map))
        return collision_map

    def get_grip_validation_map_1cm_res(self):
        '''Generates grip validation map with 1cm resolution (1 pixel == 1cm*1cm area)
        The produced map is for the gripper in its canonical/standard pose (it's length parellel to 
        the length of the table)

        Used for grasp validation 

        Size = 14*14 (general)
        '''
        v_map = np.full(shape=(self.ee_length, self.ee_length), fill_value=10) 
        for i in range(int(self.ee_length//2 - self.ee_width//2), int(self.ee_length//2 + self.ee_width//2)):
            # print("ee - {}".format(i))
            if i == (self.ee_length//2 - 1) or i == self.ee_length//2:
                v_map[i:i+1, 3:11] = 0
            
        print("Final validation map:\n{}".format(v_map))
        return v_map


    def rotate_kernel_map(self, kernel_map, angle):
        '''Converts the given map into an image and rotates it by the given angle

        Parameters:
        kernel_map = np.ndarray (2D array)
        angle: in degrees
        '''
        # modified_k_map = kernel_map + 10
        # print("Modified map: {}".format(modified_k_map))
        print("Kernel map: {}".format(abs(kernel_map+self.finger_height)))
        k_img = Image.fromarray(np.uint8(kernel_map+self.finger_height)) # Converts all negative elements to +ve (will be reversed later)
        r_img = k_img.rotate(angle=angle, fillcolor=10+self.finger_height) # Angle in degrees
        rk_map = np.array(r_img, dtype=float) 
        print("Rotated kernel: {}".format(rk_map-self.finger_height))
        
        cv2.imshow("Kernel image", np.uint8((kernel_map)*15))
        cv2.waitKey(0)

        cv2.imshow("Rotated image", np.uint8((rk_map-self.finger_height)*15))
        cv2.waitKey(0)
        # k_img.show()
        # sleep(10)
        # r_img.show()
        # sleep(30)
        return rk_map-self.finger_height

    def rotate_v_map(self, kernel_map, angle):
        '''Converts the given map into an image and rotates it by the given angle

        Parameters:
        kernel_map = np.ndarray (2D array)
        angle: in degrees
        '''
        # modified_k_map = kernel_map + 10
        # print("Modified map: {}".format(modified_k_map))
        print("Valid map: {}".format(abs(kernel_map+self.finger_height)))
        k_img = Image.fromarray(np.uint8(kernel_map+self.finger_height)) # Converts all negative elements to +ve (will be reversed later)
        r_img = k_img.rotate(angle=angle, fillcolor=10+self.finger_height) # Angle in degrees
        rk_map = np.array(r_img, dtype=float) 
        print("Rotated Valid: {}".format(rk_map-self.finger_height))
        
        cv2.imshow("Valid image", np.uint8((kernel_map)*15))
        cv2.waitKey(0)

        cv2.imshow("Rotated Valid image", np.uint8((rk_map-self.finger_height)*15))
        cv2.waitKey(0)
        return rk_map-self.finger_height

    def generate_random_h_map_1cm_res(self):
        '''Generates a height map (random) of size (20*20) with resolution of 1cm
        '''
        h_map = np.zeros(shape=(20, 20), dtype=float)

        h_map[3:10] = 6

        print("Height map (random): {}".format(h_map))
        return h_map

    def return_collision_and_valid_maps(self, scene_hmap, gripper_map, v_map, grasp_height):
        '''Checks for collisions between scene and gripper and returns a map, with 1s placed at 
        collision free areas, and 0s placed in all the other places

        scene_hmap: 20*20
        gripper_map: 14*14 map
        '''
        n_r, n_c = scene_hmap.shape
        g_nr, g_nc = gripper_map.shape
        collision_map = np.zeros(shape=(n_r, n_c))
        valid_map = np.zeros(shape=(n_r, n_c))

        for i in range(0, n_r - g_nr):
            for j in range(0, n_c - g_nc):
                sub_map = scene_hmap[i:i+g_nr, j:j+g_nc]
                print("Sub map shape: {}".format(sub_map.shape))
                # Collision checking
                if np.any(gripper_map + grasp_height - sub_map<0): # Collision detected
                    collision_map[i+int(g_nr/2), j+int(g_nc/2)] = 0
                else: # No collision
                    collision_map[i+int(g_nr/2), j+int(g_nc/2)] = 1
                    if np.min(v_map+grasp_height-self.finger_height - sub_map) < -1:# Valid grasp pose
                        valid_map[i+int(g_nr/2), j+int(g_nc/2)] = np.max(-1*(v_map+grasp_height-self.finger_height - sub_map))# 1
    
        return collision_map, valid_map

    def pcd_to_height_map_1cm_res(self, pcd, target_pos, x_range=0.30, y_range=0.30):
        '''Cuts a 30cm*30cm boundary around target pos in the given pcd and converts it into a height map
        This gives an effective search space of size - 17cm*17cm (Assuming the kernel map to be of size 
        14cm * 14cm)

        Height map resolution: 1cm (1 pixel == 1cm*1cm)

        Parameters
        pcd: pcd.points (np.array) (point cloud of the scene)
        target_pos: [x, y, z]: list (target grasp pose)

        Returns:
        (height_map, (height_map_origin_coords_world_frame)) - 
        '''
        # For resolution of 1cm*1cm, multiply pcd by 100
        pcd_m = (np.round(pcd * 100))
        pcd_m[:, 2] = pcd[:, 2] # Only x and y coordinates are scaled up to represent pixels. Z values retain their meaning (value in cm)
        x_minmax = [target_pos[0] - (x_range/2), target_pos[0]+(x_range/2 - 0.01)]*100
        y_minmax = [target_pos[1] - (y_range/2), target_pos[1]+(y_range/2 - 0.01)]*100
        # x_range = 0.6
        # y_range = 1.2

        pixel_maxes = np.zeros(shape=(int(x_range*100), int(y_range*100)), dtype=float)

        for point in pcd:
            # print(point)
            if point[0] >= x_minmax[0] and point[0] <= x_minmax[1] and point[1] >= y_minmax[0] and point[1] <= y_minmax[1]:
                # Valid point
                # print("Yes")
                pixel_coord = [int(100*(point[0] - x_minmax[0])), int(100*(point[1] - y_minmax[0]))]
                print(point, pixel_coord)
                if pixel_maxes[pixel_coord[0], pixel_coord[1]] < point[2]:
                    # print("Yes")
                    pixel_maxes[pixel_coord[0], pixel_coord[1]] = point[2]
        
        cv2.imshow("Scene hmap", np.uint8(pixel_maxes*255/np.max(pixel_maxes)))
        cv2.waitKey(0)

        return pixel_maxes

if __name__=='__main__':
    pcd_path = '/home/vishal/Volume_E/Active/Undergrad_research/Ocrtoc/Vishal_planner_stuff/Heuristic-graph-based-task-planner/buffer_detection/2-2-2.pcd'

    pcd = o3d.io.read_point_cloud(pcd_path)
    o3d.visualization.draw_geometries([pcd])
    grasp_cl = GraspModification()

    scene_test_hmap = (100*grasp_cl.pcd_to_height_map_1cm_res(np.asarray(pcd.points), target_pos=[-0.05, 0.15, 0], x_range=0.25, y_range=0.25)).astype(int)
    print(scene_test_hmap)
    # scene_test_hmap = grasp_cl.generate_random_h_map_1cm_res()

    # cv2.imshow("scENE HEIGHT_map", np.uint8(scene_test_hmap*255))
    # cv2.waitKey(0)

    k_map = grasp_cl.get_gripper_standard_collision_map_1cm_resolution()
    v_map = grasp_cl.get_grip_validation_map_1cm_res()
    # Now, rotate it
    angles = [0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 175]
    # k_maps = {
    #     '0': k_map
    # }
    for angle in angles:
        print("Angle: {}".format(angle))
        rk_map = grasp_cl.rotate_kernel_map(k_map, angle)
        vk_map = grasp_cl.rotate_v_map(v_map, angle)
        collision_map, valid_map = grasp_cl.return_collision_and_valid_maps(scene_test_hmap, rk_map, v_map, 12)
        cv2.imshow("Collision_map", np.uint8(collision_map*255))
        cv2.waitKey(0)
        print("Max valid: {}".format(np.max(valid_map)))
        print("Validation map final : {}".format(valid_map))
        cv2.imshow("Valid_map", np.uint8((valid_map/max(0.001, np.max(valid_map)))*255))
        cv2.waitKey(0)
        # k_maps[str(angle)] = rk_map
    # grasp_cl.get_gripper_standard_collision_map_5mm_resolution()

'''

# import the Python Image
# processing Library
from PIL import Image
 
# Giving The Original image Directory
# Specified
Original_Image = Image.open("./gfgrotate.jpg")
 
# Rotate Image By 180 Degree
rotated_image1 = Original_Image.rotate(180)
 
# This is Alternative Syntax To Rotate
# The Image
rotated_image2 = Original_Image.transpose(Image.ROTATE_90)
 
# This Will Rotate Image By 60 Degree
rotated_image3 = Original_Image.rotate(60)
 
rotated_image1.show()
rotated_image2.show()
rotated_image3.show()
'''
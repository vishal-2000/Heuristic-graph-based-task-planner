'''Stack+buffer planner
Handles both stacked objects sequencing and buffer sampling

Team Lumos
'''

def get_object_queues_from_stack_dict():
    '''Gets object queues 
    '''

def solve(self, node_queues, occ_map = []):
    '''Solver
    Parameters:
    node_queues: dict: {
        '<queue_key>': [lowest obj, middle obj1, middle obj2, ..., highest obj],
        ....
    } # queue keys are simple integers in string format (starting from 0)
    '''
    # dfs 
    completed_objects = []
    if len(occ_map) == 0:
        print("No occupancy map recieved, no buffer spots will be generated")
    sequence = []
    done = False
    nodes = self.red_nodes







    counter = 3
    while (not done) and counter > 0:
        current_pcd = self.get_point_cloud_from_kinect()
        print("Total number of points in pcd: {}".format(len(current_pcd.points)))
        occ_map = self.occ_and_buffer.generate_2D_occupancy_map(np.asarray(current_pcd.points)*100, x_min=-30, y_min=-60, x_range=60, y_range=120)
        self.update_black_nodes(current_pcd, occ_map)
        
        head = self.find_red_node(nodes)
        
        if head == None:
            # if len(occ_map) == 0:
            #     done = True
            #     continue
            head = self.just_find_red(nodes)
            if head == None:
                done = True
                continue
            
            # First pick the object
            res = self.go_pick_object(object_name=head.object)
            if res==False:
                head.pickable = False
                self._motion_planner.place()
                continue
            
            
            buffer = RedBlackNode(name='{}_buffer'.format(head.name), node_type='r')
            buffer.occupied = copy.deepcopy(head.occupied)
            buffer.target_black = [head.target_black[0]]
            buffer.done = False
            # buffer.attached_red_nodes= head.attached_red_nodes
            head.done = True
            head.type = 'b'
            print(occ_map)
            # target_cart_pose = np.array([head.target_black[0].pose.position.x, head.target_black[0].pose.position.y])
            target_cart_pose = np.array([head.target_black[0].pose.position.x, head.target_black[0].pose.position.y, head.target_black[0].pose.position.z,
                                            head.target_black[0].pose.orientation.x, head.target_black[0].pose.orientation.y, 
                                            head.target_black[0].pose.orientation.z, head.target_black[0].pose.orientation.w])
            print("object_mesh_path={}".format(self.object_label_mesh_path_dict[head.object]))
            buffer_spot_2d = self.occ_and_buffer.marching_grid(scene_pcd=current_pcd, object_mesh_path=self.object_label_mesh_path_dict[head.object],
                                                                target_pose_6D= target_cart_pose, OCC_THRESH=1.0, scene_name='-', object_name='-')
            # buffer_spot_2d = self.occ_and_buffer.get_empty_spot(occ_map=occ_map, closest_target=target_cart_pose)
            buffer_pose = copy.deepcopy(self.object_init_pose_dict[head.object])
            buffer_pose.position.x = buffer_spot_2d[0]
            buffer_pose.position.y = buffer_spot_2d[1]
            buffer.pickable = False
            
            # Pick and place in buffer
            print("Generated buffer. Now, pick and place the object in buffer spot!")
            # res = self.go_pick_object(object_name=head.object)
            # if res == True:
            self.go_place_object(object_name=head.object, final_place_pose=buffer_pose)
            print("Placed in buffer!")
            # import time
            # time.sleep(1)
            # rospy.sleep(1)
            
            sequence.append(head.name)
            sequence.append(buffer.name)
            buffer.prev_node_in_stack = head.prev_node_in_stack
            buffer.next_node_in_stack = head.next_node_in_stack
            if head.next_node_in_stack != None:
                head.next_node_in_stack.prev_node_in_stack = buffer
            if head.prev_node_in_stack != None:
                head.prev_node_in_stack.next_node_in_stack = buffer

            # head = self.find_red_node(nodes)
            nodes.append(buffer)
            # self.red_nodes.append(buffer)
            # nodes_labelled.append((buffer, 0))
            print("*************************++++++++**************************\nSequence: {}".format(sequence))
            # counter -= 1
            continue
        # print("Node: {}".format(head.name))
        # Find an undone red
        print("Picking and placing in target pose (since target pose is empty!)")
        self.go_pick_object(object_name=head.object)
        self.go_place_object(object_name=head.object)
        print("Placed in target pose!")
        sequence.append(head.name)
        sequence.append(head.target_black[0].name)
        head.type = 'b'
        head.done = True
        head.target_black[0].done = True
        
        completed_objects.append(head.object)
        # head = self.find_red_node(nodes)
        # if head == None:
        #     done = True
        print("Sequence: {}".format(sequence))
        # counter -= 1
        
    print('Sequence: {}'.format(sequence))  
    print("Completed objects: {}".format(completed_objects))
    print(" red nodes: {}".format(self.red_nodes))
    print("Nodes: {}".format(nodes))
    return completed_objects
'''Red Black Graph 2 Data Structure
Handles stacking cases very well and is to be traversed using rbgraph_solver2.py
'''
from os import name


class RedBlackNode:
    def __init__(self, name, type):
        '''
        Parameters
        value: 
        type: 'r' or 'b' (red implies occupied, black implies unoccupied)

        Attributes:
        1. name: Name of the node
        2. type: Type of the node (either red ('r') (occupied) or black ('b') (free))
        3. attached_red_nodes: List of attached red nodes
        4. target_black: List of target black nodes
        5. done: Node's work is over (either an object is picked up from its init pose, or placed in its final pose)
        6. occupied: States the occupancy of the node (If true, implies occupied)
        7. prev_node_in_stack: (None or RedBlackNode) (If none, the current pose does not involve any stacking conditions.
            If it is a RedBlackNode, then the current pose is involved in stacking and will be allowed to use only if the node mentioned 
            in this variable is done)
        8. next_node_in_stack: This is the next node that in final stack. This next_node has to be placed only after the current node is 
            appropriately placed in its target position
        '''
        self.name = name
        self.type = type #'R' or 'B'
        self.attached_red_nodes = []
        self.target_black = []
        self.done = False
        self.occupied = False
        self.prev_node_in_stack = None 
        self.next_node_in_stack = None

    def convert_to_red(self, target_black_node = None):
        # self.attached_red_nodes = []
        self.type = 'r'
        # self.attached_red_nodes.append(target_black_node)
        self.target_black = [target_black_node]
    
    def convert_to_black(self, attached_red_nodes = None):
        self.attached_red_nodes = []
        self.type = 'b'
        self.attached_red_nodes.extend(attached_red_nodes) 
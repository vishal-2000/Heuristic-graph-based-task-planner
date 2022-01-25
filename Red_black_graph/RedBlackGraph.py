# Data structure for node
from os import name


class RedBlackNode:
    def __init__(self, name, type):
        '''
        Parameters
        value: 
        type: 'r' or 'b' (red implies occupied, black implies unoccupied)
        '''
        self.name = name
        self.type = type #'R' or 'B'
        self.attached_red_nodes = []
        self.target_black = []
        self.done = False
        self.occupied = False
    
    def convert_to_red(self, target_black_node = None):
        # self.attached_red_nodes = []
        self.type = 'r'
        # self.attached_red_nodes.append(target_black_node)
        self.target_black = [target_black_node]
    
    def convert_to_black(self, attached_red_nodes = None):
        self.attached_red_nodes = []
        self.type = 'b'
        self.attached_red_nodes.extend(attached_red_nodes) 
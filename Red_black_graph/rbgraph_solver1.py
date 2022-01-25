'''
Author: Vishal Reddy Mandadi (Original work, literature review required)

Algorithm: Red Black Graph
Use case: Partially heuristic model-based task planner for OCRTOC (and other similar environments)
Concept: https://www.notion.so/Red-Black-Graphs-e0e88279b8884638bdcf0969a362dfd0
'''

# Data structure for node
from RedBlackGraph import RedBlackNode
from os import name

def check_done(node_list):
    done = True
    for node in node_list:
        if node.type=='r' and node.done == False:
            done = False
            return done
    return done

def find_undone_red(node_list, excluded=[]):
    for node in node_list:
        if node.type=='r' and node.done == False and (node not in excluded):
            return node
    return None

def find_red_node(node_list):
    for node in node_list:
        if node.type == 'r' and node.target_black[0].type=='b':
            return node

    return None

def just_find_red(node_list):
    for node, type in node_list:
        if node.type == 'r' and type==0:
            return node
    return None

def get_target_black_node(node_list):
    for node in node_list:
        if node.type=='b' and node.occupied==False:
            return node
    return None

if __name__=='__main__':
    rnode1 = RedBlackNode('A', 'r')
    rnode2 = RedBlackNode('B', 'r')
    rnode3 = RedBlackNode('C', 'r')
    bnode1 = rnode2 # rnode2 # RedBlackNode('B', 'b')
    bnode2 = rnode3 # rnode3 # RedBlackNode('C', 'b')
    bnode3 = RedBlackNode('C*', 'b')# rnode1 # RedBlackNode('C*', 'b')
    

    rnode1.target_black = [bnode1]
    rnode2.target_black = [bnode2]
    rnode3.target_black = [bnode3]

    bnode1.attached_red_nodes = [rnode2, rnode3]
    bnode2.attached_red_nodes = [rnode1, rnode3]
    bnode3.attached_red_nodes = [rnode1, rnode2]

    nodes_labelled = [(rnode1, 0), (rnode2, 0), (rnode3, 0), (bnode1, 1), (bnode2, 1), (bnode3, 1)]
    nodes = [rnode1, rnode2, rnode3, bnode1, bnode2, bnode3]

    # dfs 
    sequence = []
    done = False
    head = find_red_node(nodes)
    while not done:
        if head == None:
            head = just_find_red(nodes_labelled)
            if head == None:
                done = True
                continue
            buffer = RedBlackNode(name='{}_buffer'.format(head.name), type='r')
            buffer.occupied = head.occupied
            buffer.target_black = head.target_black
            buffer.done = head.done
            buffer.attached_red_nodes= head.attached_red_nodes
            head.done = True
            head.type = 'b'
            sequence.append(head.name)
            sequence.append(buffer.name)
            head = find_red_node(nodes)
            nodes.append(buffer)
            nodes_labelled.append((buffer, 0))
            continue

        # Find an undone red
        sequence.append(head.name)
        sequence.append(head.target_black[0].name)
        head.type = 'b'
        head = find_red_node(nodes)
        if head == None:
            done = True
        
    print('Sequence: {}'.format(sequence))  

        # target_black = get_target_black_node(head.target_black)
        # if target_black == None:
        #     head = find_undone_red(nodes, excluded=[head])
        #     continue
        # done = check_done(nodes)


# class RedBlackNode:
#     def __init__(self, value, type):
#         '''
#         Parameters
#         value: 
#         type: 'r' or 'b' (red implies occupied, black implies unoccupied)
#         '''
#         self.data = value
#         self.node_type = type #'R' or 'B'
#         self.target_black_node = None
#         self.attached_red_nodes = None
    
#     def convert_to_red(self, target_black_node = None):
#         self.attached_red_nodes = None
#         self.node_type = 'r'
#         self.target_black_node = target_black_node
    
#     def convert_to_black(self, attached_red_nodes = None):
#         self.target_black_node = None
#         self.node_type = 'b'
#         self.attached_red_nodes = attached_red_nodes

#     def next_node(self):
#         if type=='r':
#             return self.target_black_node
#         else:
#             for i in self.attached_red_nodes:
#                 if i.node_type == 'r':
#                     return i
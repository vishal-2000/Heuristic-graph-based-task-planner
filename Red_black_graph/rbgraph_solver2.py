'''Red Black Graph 2
This graph-based planner handles final scene stacking very robustly

Author: Vishal Reddy Mandadi

Assumptions:
1. We know the semantic information on final stacking using Scene graphs or related heuristic-based algorithms
2. We can sample empty buffer spots at any point of time
'''

from RedBlackGraph2 import RedBlackNode
virtual_buffer_indicator = '^'

def find_red_node(node_list):
    for node in node_list:
        if node.type == 'r' and node.target_black[0].type=='b' and node.target_black[0].occupied == False:
            if node.target_black[0].prev_node_in_stack == None:
                # print("yes {}".format(node.name))
                return node
            else:
                if node.target_black[0].prev_node_in_stack.done == True:
                    return node
    return None

def just_find_red(node_list):
    for node in node_list:
        if node.type == 'r' and len(node.target_black) > 0:
            return node
    return None

def solve(nodes: list):
    '''Solver
    Somewhat DFS
    '''
    # dfs 
    sequence = []
    done = False
    head = find_red_node(nodes)
    counter = 10
    while (not done) and counter > 0:
        if head == None:
            head = just_find_red(nodes)
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
            buffer.prev_node_in_stack = head.prev_node_in_stack
            buffer.next_node_in_stack = head.next_node_in_stack
            if head.next_node_in_stack != None:
                head.next_node_in_stack.prev_node_in_stack = buffer
            if head.prev_node_in_stack != None:
                head.prev_node_in_stack.next_node_in_stack = buffer

            head = find_red_node(nodes)
            nodes.append(buffer)
            # nodes_labelled.append((buffer, 0))
            print(sequence)
            counter -= 1
            continue
        # print("Node: {}".format(head.name))
        # Find an undone red
        sequence.append(head.name)
        sequence.append(head.target_black[0].name)
        head.type = 'b'
        head.done = True
        head.target_black[0].done = True
        head = find_red_node(nodes)
        # if head == None:
        #     done = True
        print(sequence)
        counter -= 1
        
    print('Sequence: {}'.format(sequence)) 

def problem_case1():
    '''Problem
    A, B and C are initial poses for the objects a, b, and c.
    A*, B*, C* are final poses for the objects a, b, c
    A* is on B* (known stacking information)
    B*, C* are free (no obstructions)
    '''
    node_names = ['A', 'B', 'C', 'A*', 'B*', 'C*']
    nodes = []
    for node_name in node_names:
        new_node = None
        if len(node_name) == 1: # implies red node
            new_node = RedBlackNode(node_name, 'r')
        else:
            new_node = RedBlackNode(node_name, 'b')
        if node_name=='B*':
            new_node.next_node_in_stack = nodes[-1]
            nodes[-1].prev_node_in_stack = new_node
        nodes.append(new_node)
    nodes[0].target_black.append(nodes[3])
    nodes[1].target_black.append(nodes[4])
    nodes[2].target_black.append(nodes[5])
    solve(nodes)


def problem_case2():
    '''Problem
    A, B and C are initial poses for the objects a, b, and c.
    A*, B*, C* are final poses for the objects a, b, c
    A* is on B* (known stacking information)
    B* is partially occupied by C (no obstructions)
    '''
    node_names = ['A', 'B', 'C', 'A*', 'B*', 'C*']
    nodes = []
    for node_name in node_names:
        new_node = None
        if len(node_name) == 1: # implies red node
            new_node = RedBlackNode(node_name, 'r')
        else:
            new_node = RedBlackNode(node_name, 'b')
        if node_name=='B*':
            new_node.next_node_in_stack = nodes[-1]
            nodes[-1].prev_node_in_stack = new_node
        nodes.append(new_node)
    nodes[0].target_black.append(nodes[3])
    nodes[1].target_black.append(nodes[4])
    nodes[2].target_black.append(nodes[5])
    nodes[4].occupied = True
    solve(nodes) 
    
    

if __name__=='__main__':
    problem_case2()
    
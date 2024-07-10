import math
from obj_functions import open_obj_file
import plotly.graph_objects as go
from ray_intersection.ray_intersection_utils import intersect_line_triangle_boolean, intersect_line_triangle_closest

def ray_intersection_AABB(ray_origin, ray_dest, mins, maxs):

    rect_len_x = abs(mins[0]) + abs(maxs[0])
    rect_len_y = abs(mins[1]) + abs(maxs[1])
    rect_len_z = abs(mins[2]) + abs(maxs[2])

    magnitude = math.sqrt(pow(ray_dest[0] - ray_origin[0], 2) + pow(ray_dest[1] - ray_origin[1], 2) + pow(ray_dest[2] - ray_origin[2], 2))
    vector_x_len = ray_dest[0] - ray_origin[0]
    vector_y_len = ray_dest[1] - ray_origin[1]
    vector_z_len = ray_dest[2] - ray_origin[2]

    print(f"magnitude: {magnitude}")
    print(f"vector_x_len: {vector_x_len}")
    print(f"vector_y_len: {vector_y_len}")
    print(f"vector_z_len: {vector_z_len}")

    dir_ray_X = vector_x_len / magnitude
    dir_ray_Y = vector_y_len / magnitude
    dir_ray_Z = vector_z_len / magnitude

    print(f"dir_ray_X: {dir_ray_X}")
    print(f"dir_ray_Y: {dir_ray_Y}")
    print(f"dir_ray_Z: {dir_ray_Z}")
    
    t_X_min = (mins[0] - ray_origin[0]) / dir_ray_X
    t_Y_min = (mins[1] - ray_origin[1]) / dir_ray_Y
    t_Z_min = (mins[2] - ray_origin[2]) / dir_ray_Z

    t_X_max = ((mins[0] + rect_len_x) - ray_origin[0]) / dir_ray_X
    t_Y_max = ((mins[1] + rect_len_y) - ray_origin[1]) / dir_ray_Y
    t_Z_max = ((mins[2] + rect_len_z) - ray_origin[2]) / dir_ray_Z

    t_X_near = min(t_X_min, t_X_max)
    t_X_far = max(t_X_min, t_X_max)

    t_Y_near = min(t_Y_min, t_Y_max)
    t_Y_far = max(t_Y_min, t_Y_max)

    t_Z_near = min(t_Z_min, t_Z_max)
    t_Z_far = max(t_Z_min, t_Z_max)

    t_near = max(t_X_near, t_Y_near, t_Z_near)
    t_far = min(t_X_far, t_Y_far, t_Z_far)

    if t_near < t_far:
        return True, t_near
    else:
        return False, t_near

def plot_BVH_from_obj_with_ray(filename, ray_origin, ray_dest):
    
    verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK = open_obj_file(filename)

    fig = go.Figure(data=[
    go.Mesh3d(
        x=verticesX,
        y=verticesY,
        z=verticesZ,
            
        color='blue',
        opacity=0.2,
            
        i=verticesI[0:-1],
        j=verticesJ[0:-1],
        k=verticesK[0:-1]
        )
    ])

    fig.add_trace(
    go.Scatter3d(x=[ray_origin[0], ray_dest[0]],
                 y=[ray_origin[1], ray_dest[1]],
                 z=[ray_origin[2], ray_dest[2]],
                 mode='lines'))

    fig.show()


def intersection_AABB_boolean(ray_origin, ray_dest, node_list):

    current_node = node_list[0]
    node_stack = []

    mins = current_node.get_bbox().mins
    maxs = current_node.get_bbox().maxs

    print("--------------------------------------------")

    if not ray_intersection_AABB(ray_origin, ray_dest, mins, maxs):
        print("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node.is_leaf():

            print("Current node is not leaf, checking children of this node.")
            left_child_intersect = ray_intersection_AABB(ray_origin, ray_dest, current_node.left.get_bbox().mins, current_node.left.get_bbox().maxs)
            right_child_intersect = ray_intersection_AABB(ray_origin, ray_dest, current_node.right.get_bbox().mins, current_node.right.get_bbox().maxs)
            print(f"Left child intersect? {left_child_intersect}")
            print(f"Right child intersect? {right_child_intersect}")
            if left_child_intersect and right_child_intersect:
                print("Both intersect.")
                node_stack.append(current_node.right)
                print(f"Node stack is: {node_stack}")
                current_node = current_node.left
                print(f"Curent node is: {current_node}, when both node were intersected.")
                continue
            elif left_child_intersect and not right_child_intersect:
                current_node = current_node.left
                print(f"Curent node is: {current_node}, when only one node was intersected (left).")
                continue
            elif not left_child_intersect and right_child_intersect:
                current_node = current_node.right
                print(f"Curent node is: {current_node}, when only one node was intersected (right).")
                continue
            else:
                print("Both nodes were not hit, do nothing!")
        else:
            print("Final intersection!!!")
            is_hit = intersect_line_triangle_boolean(current_node, ray_origin, ray_dest)
            
            print(f"is_hit after final intersection: {is_hit}")

            if is_hit == True:
                return True

        current_node = node_stack.pop(0)
        print(f"current_node after no hit: {current_node}")

        if len(node_stack) == 0:
            return False

def intersection_AABB_closest(ray_origin, ray_dest, node_list):

    current_node = [node_list[0], 0]
    node_stack = []
    
    mins = current_node[0].get_bbox().mins
    maxs = current_node[0].get_bbox().maxs
    
    print("--------------------------------------------")

    ray_dest_new = ray_dest 
    closest_hit = None

    if not ray_intersection_AABB(ray_origin, ray_dest, mins, maxs):
        print("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node[0].is_leaf():
            print("Current node is not leaf, checking children of this node.")
            left_child_intersect, t_near_left = ray_intersection_AABB(ray_origin, ray_dest_new, current_node[0].left.get_bbox().mins, current_node[0].left.get_bbox().maxs)
            right_child_intersect, t_near_right = ray_intersection_AABB(ray_origin, ray_dest_new, current_node[0].right.get_bbox().mins, current_node[0].right.get_bbox().maxs)
            print(f"Left child intersect? {left_child_intersect}")
            print(f"Right child intersect? {right_child_intersect}")
            if left_child_intersect and right_child_intersect:
                print("Both intersect.")

                if t_near_left < t_near_right:
                    node_stack.append([current_node[0].right, t_near_right])
                    current_node[0] = current_node[0].left
                    print("Left node is closer, traverse left node, right on stack.")
                else:
                    node_stack.append([current_node[0].left, t_near_left])
                    current_node[0] = current_node[0].right
                    print("Right node is closer, traverse right node, left on stack.")
                
                print(f"Node stack is: {node_stack}")
                print(f"Curent node is: {current_node}, when both node were intersected.")
                continue
            elif left_child_intersect and not right_child_intersect:
                current_node[0] = current_node[0].left
                current_node[1] = t_near_left
                print(f"Curent node is: {current_node}, when only one node was intersected (left).")
                continue
            elif not left_child_intersect and right_child_intersect:
                current_node[0] = current_node[0].right
                current_node[1] = t_near_right
                print(f"Curent node is: {current_node}, when only one node was intersected (right).")
                continue
            else:
                print("Both nodes were not hit, do nothing!")
        else:
            print("Final intersection!!!")
            ray_dest_new, closest_hit = intersect_line_triangle_closest(current_node, ray_origin, ray_dest_new, closest_hit)
            print(f"Function exit: ray_dest_new: {ray_dest_new}, closest_hit: {closest_hit}")
        print(f"node_stack before while loop: {node_stack}")

        while node_stack:

            print(f"Ultimate node stack is: {node_stack}")

            node = node_stack.pop(0)

            print(f"node is in popping: {node}")
            print(f"node is in popping [0]: {node[0]}")
            print(f"node is in popping [1]: {node[1]}")
            print(f"math.dist(ray_origin, ray_dest_new): {math.dist(ray_origin, ray_dest_new)}")

            if node[1] < math.dist(ray_origin, ray_dest_new):
                current_node = node
                print(f"Current node: {current_node}, node[1] < ray_dest_new!")
                break
            elif node[1] > math.dist(ray_origin, ray_dest_new):
                print(f"Current node: {current_node}, node[1] > ray_dest_new!!")

            print(f"node stack after loop is: {node_stack}")

        print(f"length of node stack {len(node_stack)}")
        if len(node_stack) == 0:
                print(f"{closest_hit}, {ray_dest_new}")
                if closest_hit != None:
                    return True, ray_dest_new, closest_hit
                else:
                    return False, ray_dest_new, closest_hit
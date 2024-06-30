import math
import plotly.graph_objects as go
import numpy as np

def ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation):

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

    ray_dir = np.array([dir_ray_X, dir_ray_Y, dir_ray_Z])

    local_ray_origin = np.dot(obb_rotation.T, ray_origin - obb_center)
    local_ray_dir = np.dot(obb_rotation.T, ray_dir)

    print(f"local_ray_origin: {local_ray_origin}")
    print(f"local_ray_dir: {local_ray_dir}")

    print("----------------------------------")

    t_X_min = (-obb_half_extents[0] - local_ray_origin[0]) / local_ray_dir[0]
    t_Y_min = (-obb_half_extents[1] - local_ray_origin[1]) / local_ray_dir[1]
    t_Z_min = (-obb_half_extents[2] - local_ray_origin[2]) / local_ray_dir[2]

    t_X_max = (obb_half_extents[0] - local_ray_origin[0]) / local_ray_dir[0]
    t_Y_max = (obb_half_extents[1] - local_ray_origin[1]) / local_ray_dir[1]
    t_Z_max = (obb_half_extents[2] - local_ray_origin[2]) / local_ray_dir[2]

    t_X_near = min(t_X_min, t_X_max)
    t_X_far = max(t_X_min, t_X_max)

    t_Y_near = min(t_Y_min, t_Y_max)
    t_Y_far = max(t_Y_min, t_Y_max)

    t_Z_near = min(t_Z_min, t_Z_max)
    t_Z_far = max(t_Z_min, t_Z_max)

    t_near = max(t_X_near, t_Y_near, t_Z_near)
    t_far = min(t_X_far, t_Y_far, t_Z_far)

    print(f"t_near: {t_near}")
    print(f"t_far: {t_far}")

    if t_near < t_far:
        print("There is intersection!")
        return True
    else:
        print("There is no intersection!")
        return False
    

def plot_OBB_ray(corners, ray_origin, ray_dest):
    
    fig = go.Figure()

    fig.add_trace(go.Mesh3d(
                
                x = corners[:, 0],
                y = corners[:, 1],
                z = corners[:, 2],

                i = [0, 5, 1, 2, 2, 3, 6, 1, 7, 4, 5, 5],
                j = [6, 0, 4, 4, 3, 0, 5, 2, 0, 7, 7, 1],
                k = [5, 7, 2, 3, 6, 6, 2, 5, 3, 3, 4, 4],
                color='gray',
                opacity=1,
                alphahull = 44,
                flatshading = True,
                
                lighting=dict(ambient=0.1,
                                diffuse=1,
                                fresnel=4,
                                specular=0.5,
                                roughness=0.05),
                lightposition=dict(x=100,
                                y=200,
                                z=100)
            ))
    
    fig.add_trace(
    go.Scatter3d(x=[ray_origin[0], ray_dest[0]],
                 y=[ray_origin[1], ray_dest[1]],
                 z=[ray_origin[2], ray_dest[2]],
                 mode='lines'))
    
    fig.show()

def intersection_OBB(ray_origin, ray_dest, node_list):

    current_node = node_list[0]
    node_stack = []
    
    #To do -> add these to OBB
    obb_center = current_node.get_bbox().centre
    obb_half_extents = current_node.get_bbox().half_extents
    obb_rotation = current_node.get_bbox().rotation

    if not ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation):
        print("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node.is_leaf():
            print("Current node is not leaf, checking children of this node.")
            left_child_intersect = ray_intersection_OBB(ray_origin, ray_dest, current_node.left.get_bbox().centre, current_node.left.get_bbox().half_extents, current_node.left.get_bbox().rotation)
            right_child_intersect = ray_intersection_OBB(ray_origin, ray_dest, current_node.right.get_bbox().centre, current_node.right.get_bbox().centre, current_node.right.get_bbox().centre)
            print(f"Left child intersect? {left_child_intersect}")
            print(f"Right child intersect? {right_child_intersect}")
            if left_child_intersect and right_child_intersect:
                print("Both intersect.")
                node_stack.append(current_node.right)
                print(f"Node stack is: {node_stack}")
                current_node = current_node.left
                print(f"Curent node is: {current_node}, when both node were intersected.")
            elif left_child_intersect and not right_child_intersect:
                current_node = current_node.left
                print(f"Curent node is: {current_node}, when only one node was intersected (left).")
            elif not left_child_intersect and right_child_intersect:
                current_node = current_node.right
                print(f"Curent node is: {current_node}, when only one node was intersected (right).")
        else:
            print("Final intersection!!!")
            return True
        if len(node_stack) == 0:
            print("Stack is empty -> False.")
            False
        else:
            print("Pop stack_node element.")
            current_node = node_stack.pop()
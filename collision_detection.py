import math
from obj_functions import plot_BVH_from_obj, open_obj_file
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

def calculate_ray_sphere_intersection(centre, radius, ray_origin, ray_dest):

    x1 = ray_origin[0]
    y1 = ray_origin[1] 
    z1 = ray_origin[2]

    x2 = ray_dest[0]
    y2 = ray_dest[1]
    z2 = ray_dest[2]
    
    x3 = centre[0]
    y3 = centre[1]
    z3 = centre[2]

    print(f"x3: {x3}")
    print(f"y3: {y3}")
    print(f"z3: {z3}")

    a = pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2)
    b = 2 * ((x2 - x1)*(x1 - x3) + (y2 - y1)*(y1 - y3) + (z2 - z1)*(z1 - z3))
    c = pow(x3, 2) + pow(y3, 2) + pow(z3, 2) + pow(x1, 2) + pow(y1, 2) + pow(z1, 2) - 2*((x3 * x1) + (y3*y1) + (z3*z1)) - pow(radius, 2)
    end_val = b * b - 4 * a * c

    print(f"end_val: {end_val}")
    
    if end_val < 0:
        print("There is no intersection!")
        return False
    elif end_val >= 0:
        print("There is an intersection!")
        return True

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
    
    # fig.add_trace(
    # go.Scatter3d(x=[ray_dest[0]],
    #              y=[ray_dest[1]],
    #              z=[ray_dest[2]],
    #              mode='markers'))

    fig.show()

def plot_OBB_ray(corners, ray_origin, ray_dest):
    
    fig = go.Figure()

    fig.add_trace(go.Mesh3d(
                
                x = corners[:, 0],
                y = corners[:, 1],
                z = corners[:, 2],

                # i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                # j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                # k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],

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

def calculate_t_min_max(ray_origin, ray_dest, mins, maxs):

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

    #plot_BVH_from_obj_with_ray(r"D:\OneDrive\Pulpit\Praca_Magisterska\obj\all.obj", ray_origin, ray_dest)

    if t_near < t_far:
        return True
    else:
        return False

def ray_intersection(ray_origin, ray_dest, node_list):

    current_node = node_list[0]
    node_stack = []

    if not calculate_t_min_max(ray_origin, ray_dest, current_node.get_bbox().mins, current_node.get_bbox().mins):
        print("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node.is_leaf():
            print("Current node is not leaf, checking children of this node.")
            left_child_intersect = calculate_t_min_max(ray_origin, ray_dest, current_node.left.get_bbox().mins, current_node.left.get_bbox().mins)
            right_child_intersect = calculate_t_min_max(ray_origin, ray_dest, current_node.right.get_bbox().mins, current_node.right.get_bbox().mins)
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
    

def ray_intersection_sphere(ray_origin, ray_dest, node_list):

    current_node = node_list[0]
    node_stack = []

    if not calculate_ray_sphere_intersection(current_node.get_bbox().centre, current_node.get_bbox().radius, ray_origin, ray_dest):
        print("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node.is_leaf():
            print("Current node is not leaf, checking children of this node.")
            left_child_intersect = calculate_ray_sphere_intersection(current_node.left.get_bbox().centre, current_node.left.get_bbox().radius, ray_origin, ray_dest)
            right_child_intersect = calculate_ray_sphere_intersection(current_node.right.get_bbox().centre, current_node.right.get_bbox().radius, ray_origin, ray_dest)
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
                print("Something went wrong!")
                return False
        else:
            print("Final intersection!!!")
            return True
        if len(node_stack) == 0:
            print("Stack is empty -> False.")
            False
        else:
            print("Pop stack_node element.")
            current_node = node_stack.pop()

def main():

    print("Collision detection!!!")

    ray_origin = [1.223, 6, 6.34]
    ray_dest = [-3, 10.5, -8]

    mins = [-4.445835, -3.637036, -1.701405]
    maxs = [5.998088, 2.75972, 1.701405]

    calculate_t_min_max(ray_origin, ray_dest, mins, maxs)
    
    obb_center = np.array([6.24302726e-01,  5.26056430e-01, -1.52765957e-04])
    obb_half_extents = np.array([5.42687831, 3.63361918, 1.70140113])

    obb_rotation = np.array([9.31177438e-01,  3.64563462e-01, -1.43582103e-03, 3.64565494e-01, -9.31176412e-01,
                      1.57796366e-03, 7.61734782e-04,  1.99281496e-03,  9.99997724e-01]).reshape(3,3)
    
    print(f"obb_rotation: {obb_rotation}")

    corners = np.array([-5.60251073, 1.1401508, -1.71140111,
                        7.1487463, -1.6646555, 1.71414335,
                        4.49937673, 5.10242544, 1.69966109,
                        4.50426254, 5.09705594, -1.70313342,
                        7.15363212, -1.670025, -1.68865116,
                        -2.95802697, -5.62156064, 1.70587567,
                        -5.60739654, 1.1455203, 1.6913934,
                        -2.95314116, -5.62693014, -1.69691885]).reshape(8,3)

    plot_OBB_ray(corners, ray_origin, ray_dest)
    ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation)

    print("OK")

if __name__ == "__main__":
    main()




import math
import plotly.graph_objects as go
import numpy as np

def ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation):

    print(f"obb_center: {obb_center}")
    print(f"obb_half_extents: {obb_half_extents}")
    print(f"obb_rotation: {obb_rotation}")

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
        return True, t_near
    else:
        print("There is no intersection!")
        return False, t_near
    

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

def intersection_OBB_boolean(ray_origin, ray_dest, node_list):

    current_node = node_list[0]
    node_stack = []
    
    #To do -> add these to OBB
    obb_center = current_node.get_bbox().centre
    obb_half_extents = current_node.get_bbox().half_extents
    obb_rotation = current_node.get_bbox().rotation

    print("--------------------------------------------")

    if not ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation):
        print("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node.is_leaf():

            print("Current node is not leaf, checking children of this node.")
            left_child_intersect = ray_intersection_OBB(ray_origin, ray_dest, current_node.left.get_bbox().centre, current_node.left.get_bbox().half_extents, current_node.left.get_bbox().rotation)
            right_child_intersect = ray_intersection_OBB(ray_origin, ray_dest, current_node.right.get_bbox().centre, current_node.right.get_bbox().half_extents, current_node.right.get_bbox().rotation)
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
            is_hit, s, g = intersect_line_triangle(current_node, ray_origin, ray_dest)
            
            print(f"is_hit after final intersection: {is_hit}")

            if is_hit == True:
                return True

        current_node = node_stack.pop(0)
        print(f"current_node after no hit: {current_node}")

        if len(node_stack) == 0:
            return False

def plot_triangle_ray(triangle, ray_origin, ray_dest):
    
    x = [triangle.vertices[0][0], triangle.vertices[1][0], triangle.vertices[2][0]]
    y = [triangle.vertices[0][1], triangle.vertices[1][1], triangle.vertices[2][1]]
    z = [triangle.vertices[0][2], triangle.vertices[1][2], triangle.vertices[2][2]]

    print(f"x: {x}")
    print(f"y: {y}")
    print(f"z: {z}")

    i = np.array([0])
    j = np.array([1])
    k = np.array([2])

    fig = go.Figure()
    fig.add_trace(go.Mesh3d(x=x, y=y, z=z, alphahull=5, opacity=0.4, color='cyan', i=i, j=j, k=k))
    fig.add_trace(
    go.Scatter3d(x=[ray_origin[0], ray_dest[0]],
                 y=[ray_origin[1], ray_dest[1]],
                 z=[ray_origin[2], ray_dest[2]],
                 mode='lines'))
    fig.show()


def intersect_line_triangle(current_node, ray_origin, ray_dest):

    test = "boolean_test"

    if test == "boolean_test":
        triangles = current_node.get_triangles()
    else:
        triangles = current_node[0].get_triangles()

    print(f"triangles: {triangles}")

    print("Testing a line and a triangle!")

    ray_dest_new = ray_dest
    closest_hit = None
    is_hit = False

    for triangle in triangles:

        print(f"triangle.vertices: {triangle.vertices}")

        a = np.array([triangle.vertices[0][0], triangle.vertices[0][1], triangle.vertices[0][2]])
        b = np.array([triangle.vertices[1][0], triangle.vertices[1][1], triangle.vertices[1][2]])
        c = np.array([triangle.vertices[2][0], triangle.vertices[2][1], triangle.vertices[2][2]])

        p = np.array(ray_origin)
        q = np.array(ray_dest)

        pq = q - p
        pa = a - p
        pb = b - p
        pc = c - p

        cross_pc = np.cross(pq, pc)
        cross_pa = np.cross(pq, pa)
        cross_pb = np.cross(pq, pb)

        u = np.dot(pb, cross_pc)
        v = np.dot(pc, cross_pa)
        w = np.dot(pa, cross_pb)

        uvw = [u, v, w]

        if all(item >= 0 for item in uvw) or all(item < 0 for item in uvw):
            print("u,v,w have the same sign.")
        else:
            print("u,v,w does not have the same sign.")
            continue

        denom = 1 / (u + v + w)
        u *= denom
        v *= denom
        w *= denom

        r = u * a + v * b + w * c

        print(f"r: {r}")

        if math.dist(ray_dest_new, ray_origin) > math.dist(r, ray_origin):
            ray_dest_new = r
            closest_hit = triangle

        plot_triangle_ray(triangle, ray_origin, ray_dest_new)

        print(f"ray_dest_new: {ray_dest_new}")

    if closest_hit == None and ray_dest_new == ray_dest:
        is_hit = False
    else:
        is_hit = True

    return is_hit, ray_dest_new, closest_hit


def intersection_OBB_closest(ray_origin, ray_dest, node_list):

    current_node = [node_list[0], 0]
    node_stack = []
    
    obb_center = current_node[0].get_bbox().centre
    obb_half_extents = current_node[0].get_bbox().half_extents
    obb_rotation = current_node[0].get_bbox().rotation

    print("--------------------------------------------")

    is_hit = False
    ray_dest_new = ray_dest 
    closest_hit = None

    if not ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation):
        print("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node[0].is_leaf():
            print("Current node is not leaf, checking children of this node.")
            left_child_intersect, t_near_left = ray_intersection_OBB(ray_origin, ray_dest, current_node[0].left.get_bbox().centre, current_node[0].left.get_bbox().half_extents, current_node[0].left.get_bbox().rotation)
            right_child_intersect, t_near_right = ray_intersection_OBB(ray_origin, ray_dest, current_node[0].right.get_bbox().centre, current_node[0].right.get_bbox().half_extents, current_node[0].right.get_bbox().rotation)
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
            is_hit, ray_dest_new, closest_hit = intersect_line_triangle(current_node, ray_origin, ray_dest)
        
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

            if len(node_stack) == 0:
                if closest_hit != None:
                    return is_hit, ray_dest_new, closest_hit
        
        

import logging
from ray_intersection.ray_intersection_utils import intersect_line_triangle_closest, plot_triangle_ray
import math
import numpy as np
import plotly.graph_objects as go
import time

def ray_intersect_sphere_dist(centre, radius, ray_origin, ray_dest):
    
    logging.debug(f"ray_intersect_sphere_dist")

    magnitude = math.sqrt(pow(ray_dest[0] - ray_origin[0], 2) + pow(ray_dest[1] - ray_origin[1], 2) + pow(ray_dest[2] - ray_origin[2], 2))
    vector_x_len = ray_dest[0] - ray_origin[0]
    vector_y_len = ray_dest[1] - ray_origin[1]
    vector_z_len = ray_dest[2] - ray_origin[2]

    logging.debug(f"magnitude: {magnitude}")
    logging.debug(f"vector_x_len: {vector_x_len}")
    logging.debug(f"vector_y_len: {vector_y_len}")
    logging.debug(f"vector_z_len: {vector_z_len}")

    dir_ray_X = vector_x_len / magnitude
    dir_ray_Y = vector_y_len / magnitude
    dir_ray_Z = vector_z_len / magnitude

    d = np.array([dir_ray_X, dir_ray_Y, dir_ray_Z])
    logging.debug(f"d: {d}")

    m = ray_origin - np.array(centre)
    b = np.dot(m, d)
    c = np.dot(m, m) - radius * radius
    
    logging.debug(f"m: {m}")
    logging.debug(f"b: {b}")
    logging.debug(f"c: {c}")

    if c > 0 and b > 0:
        return False, None, None
    discr = b*b - c

    logging.debug(f"discr: {discr}")

    if discr < 0:
        return False, None, None

    t = -b - np.sqrt(discr)
    logging.debug(f"t: {t}")

    if t < 0: t = 0
    q = ray_origin + t * d
    logging.debug(f"q: {q}")

    return True, t, q

def plot_2sphere_ray(ray_origin, ray_dest, centre_A, radius_A, centre_B, radius_B, q_left, q_right):

    fig = go.Figure()
    
    resolution=101
    xA = centre_A[0]
    yA = centre_A[1]
    zA = centre_A[2]

    xB = centre_B[0]
    yB = centre_B[1]
    zB = centre_B[2]

    uA, vA = np.mgrid[0:2*np.pi:resolution*2j, 0:np.pi:resolution*1j]
    XA = radius_A * np.cos(uA)*np.sin(vA) + xA
    YA = radius_A * np.sin(uA)*np.sin(vA) + yA
    ZA = radius_A * np.cos(vA) + zA

    uB, vB = np.mgrid[0:2*np.pi:resolution*2j, 0:np.pi:resolution*1j]
    XB = radius_B * np.cos(uB)*np.sin(vB) + xB
    YB = radius_B * np.sin(uB)*np.sin(vB) + yB
    ZB = radius_B * np.cos(vB) + zB

    data_plotly = go.Surface(x=XA, y=YA, z=ZA, opacity=0.5)

    fig = go.Figure(data=data_plotly)

    fig.add_trace(go.Surface(x=XB, y=YB, z=ZB, opacity=0.5))

    fig.add_trace(
    go.Scatter3d(x=[ray_origin[0], ray_dest[0]],
                 y=[ray_origin[1], ray_dest[1]],
                 z=[ray_origin[2], ray_dest[2]],
                 mode='lines'))
    
    if not q_left is None:
        fig.add_trace(
        go.Scatter3d(x=[q_left[0]],
                    y=[q_left[1]],
                    z=[q_left[2]],
                    mode='markers'))
        
    if not q_right is None:
        fig.add_trace(
        go.Scatter3d(x=[q_right[0]],
                    y=[q_right[1]],
                    z=[q_right[2]],
                    mode='markers'))

    fig.show()
    time.sleep(5)

def print_triangles_ray(current_node, ray_origin, ray_dest):

    fig = go.Figure()

    triangles = current_node[0].get_triangles()

    x = []
    y = []
    z = []    

    i = []
    j = []
    k = []

    num = 0
    for triangle in triangles:

        x.append(triangle.vertices[0][0])
        x.append(triangle.vertices[1][0])
        x.append(triangle.vertices[2][0])

        y.append(triangle.vertices[0][1])
        y.append(triangle.vertices[1][1])
        y.append(triangle.vertices[2][1])

        z.append(triangle.vertices[0][2])
        z.append(triangle.vertices[1][2])
        z.append(triangle.vertices[2][2])

        logging.debug(f"---------------------------------")

        logging.debug(f"x: {x}")
        logging.debug(f"y: {y}")
        logging.debug(f"z: {z}")

    
        i.append([3 * num])
        j.append([(3 * num) + 1])
        k.append([(3 * num) + 2])
        num += 1 

    fig.add_trace(go.Mesh3d(x=x, y=y, z=z, alphahull=5, opacity=0.4, color='red', i=i, j=j, k=k))

    fig.add_trace(
    go.Scatter3d(x=[ray_origin[0], ray_dest[0]],
                 y=[ray_origin[1], ray_dest[1]],
                 z=[ray_origin[2], ray_dest[2]],
                 mode='lines'))

    fig.show()

def intersection_sphere_closest(ray_origin, ray_dest, node_list):

    current_node = [node_list[0], 0]
    node_stack = []
    
    centre = current_node[0].get_bbox().centre
    radius = current_node[0].get_bbox().radius
    
    logging.debug("--------------------------------------------")

    ray_dest_new = ray_dest 
    closest_hit = None

    is_intersect, t, q = ray_intersect_sphere_dist(centre, radius, ray_origin, ray_dest)

    if not is_intersect:
        logging.debug("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node[0].is_leaf():
            logging.debug("Current node is not leaf, checking children of this node.")
            left_child_intersect, t_near_left, q_left = ray_intersect_sphere_dist(current_node[0].left.get_bbox().centre, current_node[0].left.get_bbox().radius, ray_origin, ray_dest_new)
            right_child_intersect, t_near_right, q_right = ray_intersect_sphere_dist(current_node[0].right.get_bbox().centre, current_node[0].right.get_bbox().radius, ray_origin, ray_dest_new)
            logging.debug(f"Left child intersect? {left_child_intersect}")
            logging.debug(f"Right child intersect? {right_child_intersect}")
            if left_child_intersect and right_child_intersect:
                # Put the furthest on the stack
                if t_near_left < t_near_right:
                    node_stack.append([current_node[0].right, t_near_right])
                    current_node = [current_node[0].left, t_near_left]
                else:
                    node_stack.append([current_node[0].left, t_near_left])
                    current_node = [current_node[0].right, t_near_right]
            elif left_child_intersect:
                current_node = [current_node[0].left, t_near_left]
            elif right_child_intersect:
                current_node = [current_node[0].right, t_near_right]
            else:
                # No intersections, pop the next node if available
                if node_stack:
                    current_node = node_stack.pop()
                else:
                    break
        else:
            # Leaf node - perform intersection with each primitive
            ray_dest_new, closest_hit = intersect_line_triangle_closest(current_node, ray_origin, ray_dest_new, closest_hit)

            if closest_hit:
                # plot_triangle_ray(closest_hit, ray_origin, ray_dest_new)
                return True, ray_dest_new, closest_hit

            # Pop the next node if available
            if node_stack:
                current_node = node_stack.pop()
            else:
                return False, ray_dest_new, closest_hit if closest_hit else None
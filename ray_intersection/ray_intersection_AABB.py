import math
from obj_functions import open_obj_file
import plotly.graph_objects as go
from ray_intersection.ray_intersection_utils import intersect_line_triangle_closest, plot_triangle_ray
import logging
import sys
import numpy as np


def ray_intersection_AABB_CD(ray_origin, ray_dest, mins, maxs):

    EPSILON = 1e-80
    t_min = 0
    t_max = sys.maxsize

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

    dir_ray = np.array([dir_ray_X, dir_ray_Y, dir_ray_Z])

    for i in range(3):
        if np.abs(dir_ray[i]) < EPSILON:
            if ray_origin[i] < mins[i] or ray_origin[i] > maxs[i]: return False
        else:
            ood = 1/dir_ray[i]
            t1 = (mins[i] - ray_origin[i]) * ood
            t2 = (maxs[i] - ray_origin[i]) * ood

            if t1 > t2: t1, t2 = t2, t1

            if t1 > t_min: t_min = t1
            if t2 > t_max: t_max = t2
            if t_min > t_max: return False

    q = ray_origin + dir_ray * t_min
    return True, t_min, q

def ray_intersection_AABB(ray_origin, ray_dest, mins, maxs):

    logging.debug(f"ray_intersection_AABB")

    EPSILON = 1e-8  # Smaller epsilon for better precision

    # Convert lists to NumPy arrays for element-wise operations
    ray_origin = np.array(ray_origin)
    ray_dest = np.array(ray_dest)
    mins = np.array(mins)
    maxs = np.array(maxs)

    magnitude = np.linalg.norm(ray_dest - ray_origin)
    dir_ray = (ray_dest - ray_origin) / (magnitude + EPSILON)  # Normalize ray direction

    # Calculate intersections with box planes, handling divide-by-zero cases
    t_min_vals = np.where(dir_ray != 0, (mins - ray_origin) / dir_ray, -np.inf)
    t_max_vals = np.where(dir_ray != 0, (maxs - ray_origin) / dir_ray, np.inf)

    # Determine the near and far intersection distances for each axis
    t_near = max(min(t_min_vals[0], t_max_vals[0]),
                 min(t_min_vals[1], t_max_vals[1]),
                 min(t_min_vals[2], t_max_vals[2]))

    t_far = min(max(t_min_vals[0], t_max_vals[0]),
                max(t_min_vals[1], t_max_vals[1]),
                max(t_min_vals[2], t_max_vals[2]))

    # Check if there's a valid intersection
    if t_near < t_far - EPSILON and 0 <= t_near <= magnitude + EPSILON:
        return True, t_near
    else:
        return False, None

def plot_BVH_aabb_from_obj_with_ray(filename, ray_origin, ray_dest):
    
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

def intersection_AABB_closest(ray_origin, ray_dest, node_list):
    current_node = [node_list[0], 0]
    node_stack = []
    ray_dest_new = ray_dest 
    closest_hit = None

    # Initial root intersection check
    intersection_root, t_near_root = ray_intersection_AABB(
        ray_origin, ray_dest, 
        current_node[0].get_bbox().mins, 
        current_node[0].get_bbox().maxs
    )

    if not intersection_root:
        return False, None, None

    while True:
        # If node stack and current_node are empty, exit
        if not node_stack and current_node is None:
            return False, ray_dest_new, closest_hit if closest_hit else None

        if not current_node[0].is_leaf():
            logging.debug(f"No leaf check intersecton")
            # Check intersections with left and right children
            left_child_intersect, t_near_left = ray_intersection_AABB(
                ray_origin, ray_dest_new, 
                current_node[0].left.get_bbox().mins, 
                current_node[0].left.get_bbox().maxs
            )
            right_child_intersect, t_near_right = ray_intersection_AABB(
                ray_origin, ray_dest_new, 
                current_node[0].right.get_bbox().mins, 
                current_node[0].right.get_bbox().maxs
            )

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
                    logging.debug(f"pop stack 1")
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
                logging.debug(f"pop stack 2")
                current_node = node_stack.pop()
            else:
                return False, ray_dest_new, closest_hit if closest_hit else None

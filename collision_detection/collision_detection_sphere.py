import logging
import time
import numpy as np
from collision_detection.collision_detection_utils import closest_pt_point_obb, test_triangle_against_triangle, descend_A, test_sphere_AABB, descend_A_iter
import itertools
import plotly.graph_objects as go
import numba
from numba import njit
import cProfile
import re

def test_sphere_sphere(sphere_A, sphere_B):

    center_A = np.array(sphere_A.get_bbox().centre)
    center_B = np.array(sphere_B.get_bbox().centre)
    radius_A = sphere_A.get_bbox().radius
    radius_B = sphere_B.get_bbox().radius

    d = center_A - center_B
    dist2 = np.dot(d, d)
    radius_sum = radius_A + radius_B

    return dist2 <= radius_sum * radius_sum

def plot_sphere_obb(sphere, obb, result, q):

    box_a = obb.get_bbox()

    fig = go.Figure()
    fig.add_trace(go.Mesh3d(
                
        x = box_a.corners[:, 0],
        y = box_a.corners[:, 1],
        z = box_a.corners[:, 2],

        i = [0, 5, 1, 2, 2, 3, 6, 1, 7, 4, 5, 5],
        j = [6, 0, 4, 4, 3, 0, 5, 2, 0, 7, 7, 1],
        k = [5, 7, 2, 3, 6, 6, 2, 5, 3, 3, 4, 4],
        color='gray',
        opacity=0.1,
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
    
    resolution=101
    x = sphere.get_bbox().centre[0]
    y = sphere.get_bbox().centre[1]
    z = sphere.get_bbox().centre[2]

    u, v = np.mgrid[0:2*np.pi:resolution*2j, 0:np.pi:resolution*1j]
    X = sphere.get_bbox().radius * np.cos(u)*np.sin(v) + x
    Y = sphere.get_bbox().radius * np.sin(u)*np.sin(v) + y
    Z = sphere.get_bbox().radius * np.cos(v) + z

    fig.add_trace(go.Surface(x=X, y=Y, z=Z, opacity=0.1))

    fig.add_trace(go.Scatter3d(x=[q[0]], y=[q[1]], z = [q[2]], marker_size=2))

    fig.update_layout(title_text=f"Sphere x OBB test result: {result}.")
    fig.show()

def test_sphere_obb(sphere, obb):

    logging.debug("test_sphere_obb")

    q = 0

    center = sphere.get_bbox().centre
    radius = sphere.get_bbox().radius

    logging.debug(f"center, sphere x obb: {center}")
    logging.debug(f"radius: {radius}")

    q = closest_pt_point_obb(center, obb)

    logging.debug(f"q: {q}")

    v = q - center

    logging.debug(f"v: {v}")
    logging.debug(f"np.dot(v, v): {np.dot(v, v)}, radius^2: {radius * radius}, np.dot(v, v) <= radius * radius: {np.dot(v, v) <= radius * radius}")

    return np.dot(v, v) <= radius * radius, q

def BVH_collision_sphere_iterative(tree_a, tree_b, bbox_type_B):
    logging.debug("iterative BVH traversal")

    stack = []
    collisions = []

    aabb_a_temp = tree_a[0]
    aabb_b_temp = tree_b[0]    

    stack.append((aabb_a_temp, aabb_b_temp))

    while stack:

        logging.debug(f"stack before pop: {stack}")
        aabb_a_temp, aabb_b_temp = stack.pop()
        logging.debug(f"stack after pop: {stack}")

        if bbox_type_B == "aabb":
            if not test_sphere_AABB(aabb_a_temp.get_bbox(), aabb_b_temp): continue
        elif bbox_type_B == "sphere":
            result = test_sphere_sphere(aabb_a_temp, aabb_b_temp)
            if not result: continue 
        elif bbox_type_B == "obb":
            result, q = test_sphere_obb(aabb_a_temp, aabb_b_temp)
            if not result: continue

        if aabb_a_temp.is_leaf() and aabb_b_temp.is_leaf():
            logging.debug("Checking collisions on objects level.")
            tri_collisions = test_triangle_against_triangle(aabb_a_temp, aabb_b_temp)
            logging.debug(f"tri_collisions: {tri_collisions}")
            if not len(tri_collisions) == 0:
                logging.debug(f"tri_collisions: {len(tri_collisions)}")
                collisions.extend(tri_collisions)
                
                logging.debug(f"collisions: {len(collisions)}")
                logging.debug(f"len collisions: {len(collisions)}")
        else:
            if descend_A_iter(aabb_a_temp):
                logging.debug("Descend A")
                stack.append((aabb_a_temp.right, aabb_b_temp))
                stack.append((aabb_a_temp.left, aabb_b_temp))
                continue
                
            else:
                logging.debug("Descend B")
                stack.append((aabb_a_temp, aabb_b_temp.right))
                stack.append((aabb_a_temp, aabb_b_temp.left))
                continue

    logging.debug(f"Collisions before exit: {collisions}")

    return collisions

def collision_detection_sphere(node_list_A, node_list_B, bbox_type_B):
    logging.debug("collision_detection_sphere")

    collisions = []

    collisions = BVH_collision_sphere_iterative(node_list_A, node_list_B, bbox_type_B)

    logging.debug(f"collisions: {collisions}")
    logging.debug(f"number of collisions: {len(collisions)}")
    return collisions

   
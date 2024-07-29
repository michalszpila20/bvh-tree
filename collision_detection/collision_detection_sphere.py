import logging
import numpy as np
from collision_detection.collision_detection_utils import test_triangle_against_triangle, descend_A
import itertools
import plotly.graph_objects as go

def test_sphere_sphere(sphere_A, sphere_B):

    center_A = np.array([sphere_A.get_bbox().centre[0], sphere_A.get_bbox().centre[1], sphere_A.get_bbox().centre[2]])
    center_B = np.array([sphere_B.get_bbox().centre[0], sphere_B.get_bbox().centre[1], sphere_B.get_bbox().centre[2]])

    d = center_A - center_B
    dist2 = np.dot(d, d)

    logging.debug(f"dist2: {dist2}")

    radius_sum = sphere_A.get_bbox().radius + sphere_B.get_bbox().radius

    logging.debug(f"radius_sum: {radius_sum}")

    logging.debug(f"dist2 <= radius_sum * radius_sum: {dist2 <= radius_sum * radius_sum}")

    return dist2 <= radius_sum * radius_sum

def BVH_collision_sphere(tree_a, tree_b, index_a, index_b, collisions):

    logging.debug(f"index_a: {index_a}")
    logging.debug(f"index_b: {index_b}")

    logging.debug(f"tree_a: {tree_a}")
    logging.debug(f"tree_b: {tree_b}")

    sphere_A = tree_a[index_a]
    sphere_B = tree_b[index_b]

    if not test_sphere_sphere(sphere_A, sphere_B): return None

    if sphere_A.is_leaf() and sphere_B.is_leaf():
        logging.debug("Checking collisions on objects level.")
        test_triangle_against_triangle(sphere_A, sphere_B, collisions)
    else:
        if descend_A(tree_a, index_a):
            index_a_one = tree_a.index(sphere_A.left)
            index_a_two = tree_a.index(sphere_A.right)
            BVH_collision_sphere(tree_a, tree_b, index_a_one, index_b, collisions)
            BVH_collision_sphere(tree_a, tree_b, index_a_two, index_b, collisions)
        else:
            index_b_one = tree_b.index(sphere_B.left)
            index_b_two = tree_b.index(sphere_B.right)
            BVH_collision_sphere(tree_a, tree_b, index_a, index_b_one, collisions)
            BVH_collision_sphere(tree_a, tree_b, index_a, index_b_two, collisions)
    
    return collisions

def collision_detection_sphere(node_list_A, node_list_B):
    logging.debug("collision_detection_sphere")

    collisions = []

    collisions = BVH_collision_sphere(node_list_A, node_list_B, 0, 0, collisions)

    logging.debug(f"collisions: {collisions}")
    logging.debug(f"number of collisions: {len(collisions)}")
    return collisions

   
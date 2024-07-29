import logging
import numpy as np
import sys
from collision_detection.collision_detection_utils import test_triangle_against_triangle, descend_A

def test_obb_obb(obb_a, obb_b):

    radius_a = 0
    radius_b = 0

    radius = np.arange(2, 11).reshape(3, 3)
    abs_radius = np.arange(2, 11).reshape(3, 3)

    logging.debug(f"obb_a.rotation: {obb_a.get_bbox().rotation}")
    logging.debug(f"obb_a.rotation[0]: {obb_a.get_bbox().rotation[0]}")

    for i in range(0, 3):
        for j in range(0, 3):
            radius[i][j] = np.dot(obb_a.get_bbox().rotation[i], obb_a.get_bbox().rotation[j])

    logging.debug(f"radius: {radius}")

    t = obb_b.get_bbox().centre - obb_a.get_bbox().centre

    logging.debug(f"t: {t}")

    t = np.array([np.dot(t, obb_a.get_bbox().rotation[0]), np.dot(t, obb_a.get_bbox().rotation[2]), np.dot(t, obb_a.get_bbox().rotation[2])])

    logging.debug(f"t: {t}")

    for i in range(0, 3):
        for j in range(0, 3):
            abs_radius[i][j] = abs(radius[i][j]) + sys.float_info.epsilon

    logging.debug(f"abs_radius: {abs_radius}")

    ################ axes L = A0, L = A1, L = A2
    logging.debug("axes L = A0, L = A1, L = A2")

    for i in range(3):
        radius_a = obb_a.get_bbox().half_extents[i]
        radius_b = obb_b.get_bbox().half_extents[0] * abs_radius[i][0] + obb_b.get_bbox().half_extents[1] * abs_radius[i][1] + obb_b.get_bbox().half_extents[2] * abs_radius[i][2]
        logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
        logging.debug(f"abs(t[i]): {abs(t[i])}, radius_a + radius_b: {radius_a + radius_b}")
        if abs(t[i]) > radius_a + radius_b: return False

    ################ axes L = B0, L = B1, L = B2
    logging.debug("axes L = B0, L = B1, L = B2")

    for i in range(3):
        radius_a = obb_a.get_bbox().half_extents[0] * abs_radius[0][i] + obb_a.get_bbox().half_extents[1] * abs_radius[1][i] + obb_a.get_bbox().half_extents[2] * abs_radius[2][i]
        radius_b = obb_b.get_bbox().half_extents[i]
        logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
        logging.debug(f"abs(t[0] * radius[0][i] + t[1]*radius[1][i] + t[2]*radius[2][i]): {abs(t[0] * radius[0][i] + t[1]*radius[1][i] + t[2]*radius[2][i])}")
        logging.debug(f"radius_a + radius_b: {radius_a + radius_b}")
        if abs(t[0] * radius[0][i] + t[1]*radius[1][i] + t[2]*radius[2][i]) > radius_a + radius_b: return False
    
    ################ axis L = A0 x B0
    logging.debug("axis L = A0 x B0")
    radius_a = obb_a.get_bbox().half_extents[1] * abs_radius[2][0] + obb_a.get_bbox().half_extents[2] * abs_radius[1][0]
    radius_b = obb_b.get_bbox().half_extents[1] * abs_radius[0][2] + obb_b.get_bbox().half_extents[2] * abs_radius[0][1]
    logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
    logging.debug(f"abs(t[2] * radius[1][0] - t[1]*radius[2][0]): {abs(t[2] * radius[1][0] - t[1]*radius[2][0])}, radius_a + radius_b: {radius_a + radius_b}")
    if abs(t[2] * radius[1][0] - t[1]*radius[2][0]) > radius_a + radius_b: return False

    ################ axis L = A0 x B1
    logging.debug("axis L = A0 x B1")
    radius_a = obb_a.get_bbox().half_extents[1] * abs_radius[2][1] + obb_a.get_bbox().half_extents[2] * abs_radius[1][1]
    radius_b = obb_b.get_bbox().half_extents[0] * abs_radius[0][2] + obb_b.get_bbox().half_extents[2] * abs_radius[0][0]
    logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
    logging.debug(f"abs(t[2] * radius[1][1] - t[1] * radius[2][1]): {abs(t[2] * radius[1][1] - t[1] * radius[2][1])}, radius_a + radius_b: {radius_a + radius_b}")
    if abs(t[2] * radius[1][1] - t[1] * radius[2][1]) > radius_a + radius_b: return False

    ################ axis L = A0 x B2
    logging.debug("axis L = A0 x B2")
    radius_a = obb_a.get_bbox().half_extents[1] * abs_radius[2][2] + obb_a.get_bbox().half_extents[2] * abs_radius[1][2]
    radius_b = obb_b.get_bbox().half_extents[0] * abs_radius[0][1] + obb_b.get_bbox().half_extents[1] * abs_radius[0][0]
    logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
    logging.debug(f"abs(t[2] * radius[1][2] - t[1] * radius[2][2]): {abs(t[2] * radius[1][2] - t[1] * radius[2][2])}, radius_a + radius_b: {radius_a + radius_b}")
    if abs(t[2] * radius[1][2] - t[1] * radius[2][2]) > radius_a + radius_b: return False

    ################ axis L = A1 x B0
    logging.debug("axis L = A1 x B0")
    radius_a = obb_a.get_bbox().half_extents[0] * abs_radius[2][0] + obb_a.get_bbox().half_extents[2] * abs_radius[0][0]
    radius_b = obb_b.get_bbox().half_extents[1] * abs_radius[1][2] + obb_b.get_bbox().half_extents[2] * abs_radius[1][1]
    logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
    logging.debug(f"abs(t[0] * radius[2][0] - t[2] * radius[0][0]): {abs(t[0] * radius[2][0] - t[2] * radius[0][0])}, radius_a + radius_b: {radius_a + radius_b}")
    if abs(t[0] * radius[2][0] - t[2] * radius[0][0]) > radius_a + radius_b: return False

    ################ axis L = A1 x B1
    logging.debug("axis L = A1 x B1")
    radius_a = obb_a.get_bbox().half_extents[0] * abs_radius[2][1] + obb_a.get_bbox().half_extents[2] * abs_radius[0][1]
    radius_b = obb_b.get_bbox().half_extents[0] * abs_radius[1][2] + obb_b.get_bbox().half_extents[2] * abs_radius[1][0]
    logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
    logging.debug(f"abs(t[0] * radius[2][1] - t[2] * radius[0][1]): {abs(t[0] * radius[2][1] - t[2] * radius[0][1])}, radius_a + radius_b: {radius_a + radius_b}")
    if abs(t[0] * radius[2][1] - t[2] * radius[0][1]) > radius_a + radius_b: return False

    ################ axis L = A1 x B2
    logging.debug("axis L = A1 x B2")
    radius_a = obb_a.get_bbox().half_extents[0] * abs_radius[2][2] + obb_a.get_bbox().half_extents[2] * abs_radius[0][2]
    radius_b = obb_b.get_bbox().half_extents[0] * abs_radius[1][1] + obb_b.get_bbox().half_extents[1] * abs_radius[1][0]
    logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
    logging.debug(f"abs(t[0] * radius[2][2] - t[2] * radius[0][2]): {abs(t[0] * radius[2][2] - t[2] * radius[0][2])}, radius_a + radius_b: {radius_a + radius_b}")
    if abs(t[0] * radius[2][2] - t[2] * radius[0][2]) > radius_a + radius_b: return False

    ################ axis L = A2 x B0
    logging.debug("axis L = A2 x B0")
    radius_a = obb_a.get_bbox().half_extents[0] * abs_radius[1][0] + obb_a.get_bbox().half_extents[1] * abs_radius[0][0]
    radius_b = obb_b.get_bbox().half_extents[1] * abs_radius[2][2] + obb_b.get_bbox().half_extents[2] * abs_radius[2][1]
    logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
    logging.debug(f"abs(t[1] * radius[0][0] - t[0] * radius[1][0]): {abs(t[1] * radius[0][0] - t[0] * radius[1][0])}, radius_a + radius_b: {radius_a + radius_b}")
    if abs(t[1] * radius[0][0] - t[0] * radius[1][0]) > radius_a + radius_b: return False

    ################ axis L = A2 x B1
    logging.debug("axis L = A2 x B1")
    radius_a = obb_a.get_bbox().half_extents[0] * abs_radius[1][1] + obb_a.get_bbox().half_extents[1] * abs_radius[0][1]
    radius_b = obb_b.get_bbox().half_extents[0] * abs_radius[2][2] + obb_b.get_bbox().half_extents[2] * abs_radius[2][0]
    logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
    logging.debug(f"abs(t[1] * radius[0][1] - t[0] * radius[1][1]): {abs(t[1] * radius[0][1] - t[0] * radius[1][1])}, radius_a + radius_b: {radius_a + radius_b}")
    if abs(t[1] * radius[0][1] - t[0] * radius[1][1]) > radius_a + radius_b: return False

    ################ axis L = A2 x B2
    logging.debug("axis L = A2 x B2")
    radius_a = obb_a.get_bbox().half_extents[0] * abs_radius[1][2] + obb_a.get_bbox().half_extents[1] * abs_radius[0][2]
    radius_b = obb_b.get_bbox().half_extents[0] * abs_radius[2][1] + obb_b.get_bbox().half_extents[1] * abs_radius[2][0]
    logging.debug(f"radius_a: {radius_a}, radius_b: {radius_b}")
    logging.debug(f"abs(t[1] * radius[0][2] - t[0] * radius[1][2]): {abs(t[1] * radius[0][2] - t[0] * radius[1][2])}, radius_a + radius_b: {radius_a + radius_b}")
    if abs(t[1] * radius[0][2] - t[0] * radius[1][2]) > radius_a + radius_b: return False

    return True

def BVH_collision_obb(tree_a, tree_b, index_a, index_b, collisions):

    logging.debug(f"index_a: {index_a}")
    logging.debug(f"index_b: {index_b}")

    logging.debug(f"tree_a: {tree_a}")
    logging.debug(f"tree_b: {tree_b}")

    obb_A = tree_a[index_a]
    obb_B = tree_b[index_b]

    if not test_obb_obb(obb_A, obb_B): return None

    if obb_A.is_leaf() and obb_B.is_leaf():
        logging.debug("Checking collisions on objects level.")
        test_triangle_against_triangle(obb_A, obb_B, collisions)
    else:
        if descend_A(tree_a, index_a):
            index_a_one = tree_a.index(obb_A.left)
            index_a_two = tree_a.index(obb_A.right)
            BVH_collision_obb(tree_a, tree_b, index_a_one, index_b, collisions)
            BVH_collision_obb(tree_a, tree_b, index_a_two, index_b, collisions)
        else:
            index_b_one = tree_b.index(obb_B.left)
            index_b_two = tree_b.index(obb_B.right)
            BVH_collision_obb(tree_a, tree_b, index_a, index_b_one, collisions)
            BVH_collision_obb(tree_a, tree_b, index_a, index_b_two, collisions)
    
    return collisions

def collision_detection_obb(node_list_A, node_list_B):
    
    logging.debug("collision_detection_obb")

    collisions = []

    collisions = BVH_collision_obb(node_list_A, node_list_B, 0, 0, collisions)
    logging.debug(f"collisions: {collisions}")
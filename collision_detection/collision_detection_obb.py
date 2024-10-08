import logging
import numpy as np
import sys
from OBB import OBB

from bvh import BVHNode
from collision_detection.collision_detection_sphere import test_sphere_obb
from collision_detection.collision_detection_utils import test_triangle_against_triangle, descend_A, build_obb_from_aabb, descend_A_iter
from obj_functions import plot_2OBB
import time

def test_obb_obb(obb_a, obb_b):

    radius_a = 0
    radius_b = 0

    radius = np.zeros((3, 3), dtype = 'complex_')
    abs_radius = np.zeros((3, 3), dtype = 'complex_')
    EPSILON = 1e-6

    u_a = np.array([obb_a.get_bbox().rotation[:, 0], obb_a.get_bbox().rotation[:, 1], obb_a.get_bbox().rotation[:, 2]])
    u_b = np.array([obb_b.get_bbox().rotation[:, 0], obb_b.get_bbox().rotation[:, 1], obb_b.get_bbox().rotation[:, 2]])
    logging.debug(f"obb_a.rotation: {obb_a.get_bbox().rotation}")
    logging.debug(f"obb_a.rotation[0]: {obb_a.get_bbox().rotation[0]}")

    for i in range(0, 3):
        for j in range(0, 3):
            logging.debug(f"obb_a.get_bbox().rotation[i]: {obb_a.get_bbox().rotation[i]}")
            radius[i][j] = np.dot(u_a[i], u_b[j])

    logging.debug(f"radius: {radius}")

    t = obb_b.get_bbox().centre - obb_a.get_bbox().centre

    logging.debug(f"t: {t}")

    ## check
    t = np.array([np.dot(t, u_a[0]), np.dot(t, u_a[1]), np.dot(t, u_a[2])])

    logging.debug(f"t: {t}")

    for i in range(0, 3):
        for j in range(0, 3):
            abs_radius[i][j] = abs(radius[i][j]) + EPSILON

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

def BVH_collision_obb(tree_a, tree_b, index_a, index_b, bbox_type_B, collisions):

    logging.debug("===============================================")
    
    obb_A = tree_a[index_a]
    obb_B = tree_b[index_b]

    logging.debug(f"OBB_A: {obb_A}, OBB_B: {obb_B}")

    if bbox_type_B == "obb":
        if not test_obb_obb(obb_A, obb_B): 
            logging.debug("No intersection between two obb. Return with None.")
            return
    elif bbox_type_B == "sphere":

        result, q = test_sphere_obb(obb_B, obb_A)
        if not result: return None

    elif bbox_type_B == "aabb":

        corners, center, diff, rotation = build_obb_from_aabb(obb_B)
        obb_from_aabb = OBB(corners, center, diff, rotation)
        temp_node = BVHNode(None)
        temp_node.set_bbox(obb_from_aabb)
        if not test_obb_obb(obb_A, temp_node): return None

    if obb_A.is_leaf() and obb_B.is_leaf():
        logging.debug("Checking collisions on objects level.")
        test_triangle_against_triangle(obb_A, obb_B, collisions)
    else:
        if descend_A(tree_a, index_a):
            logging.debug("descend_A")
            index_a_one = tree_a.index(obb_A.left)
            index_a_two = tree_a.index(obb_A.right)
            BVH_collision_obb(tree_a, tree_b, index_a_one, index_b, bbox_type_B, collisions)
            BVH_collision_obb(tree_a, tree_b, index_a_two, index_b, bbox_type_B, collisions)
        else:
            logging.debug("descend_B")
            index_b_one = tree_b.index(obb_B.left)
            index_b_two = tree_b.index(obb_B.right)
            BVH_collision_obb(tree_a, tree_b, index_a, index_b_one, bbox_type_B, collisions)
            BVH_collision_obb(tree_a, tree_b, index_a, index_b_two, bbox_type_B, collisions)
    
    return collisions

def BVH_collision_obb_iterative(tree_a, tree_b, bbox_type_B):
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
            logging.debug("Add aabb - obb intersection test")
            corners, center, diff, rotation = build_obb_from_aabb(aabb_b_temp)
            obb_from_aabb = OBB(corners, center, diff, rotation)
            temp_node = BVHNode(None)
            temp_node.set_bbox(obb_from_aabb)
            if not test_obb_obb(temp_node, aabb_a_temp): continue
        elif bbox_type_B == "sphere":
            result, q = test_sphere_obb(aabb_b_temp, aabb_a_temp)
            if not result: continue
        elif bbox_type_B == "obb":
            if not test_obb_obb(aabb_a_temp, aabb_b_temp): continue

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


def collision_detection_obb(node_list_A, node_list_B, bbox_type_B):
    
    logging.debug("collision_detection_obb")

    collisions = []
    leaves_a = []
    leaves_b = []

    logging.debug(f"node_list_A: {len(node_list_A)}, node_list_B: {len(node_list_B)}")

    for node in node_list_A:
        if node.leaf == True:
            leaves_a.append(node)

    for node in node_list_B:
        if node.leaf == True:
            leaves_b.append(node)

    logging.debug(f"leaves_a: {len(leaves_a)}, leaves_b: {len(leaves_b)}")

    # collisions = BVH_collision_obb(node_list_A, node_list_B, 0, 0, bbox_type_B, collisions)
    collisions = BVH_collision_obb_iterative(node_list_A, node_list_B, bbox_type_B)
    logging.debug(f"collisions: {collisions}")
    logging.debug(f"size of collisions: {len(collisions)}")

    return collisions
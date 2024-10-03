import logging
import numpy as np
import sys
from OBB import OBB
from node import Node
from collision_detection.collision_detection_sphere import test_sphere_obb
from collision_detection.collision_detection_utils import test_triangle_against_triangle, descend_A, build_obb_from_aabb, descend_A_iter
from obj_functions import plot_2OBB
import time

def test_obb_obb(obb_a, obb_b):
    # Extract necessary components from obb_a and obb_b (rotation, half_extents, and centre)
    u_a = obb_a.get_bbox().rotation
    u_b = obb_b.get_bbox().rotation

    center_a = obb_a.get_bbox().centre
    center_b = obb_b.get_bbox().centre

    half_extents_a = obb_a.get_bbox().half_extents
    half_extents_b = obb_b.get_bbox().half_extents

    EPSILON = 1e-6
    radius = np.zeros((3, 3), dtype=np.float64)
    abs_radius = np.zeros((3, 3), dtype=np.float64)

    # Compute the rotation matrix expressing obb_b in obb_a's coordinate frame
    for i in range(3):
        for j in range(3):
            radius[i, j] = np.dot(u_a[:, i], u_b[:, j])
            abs_radius[i, j] = abs(radius[i, j]) + EPSILON

    # Compute translation vector t between OBB centers and project it onto obb_a's local axes
    t = center_b - center_a
    t = np.array([np.dot(t, u_a[:, 0]), np.dot(t, u_a[:, 1]), np.dot(t, u_a[:, 2])])

    # Axes L = A0, A1, A2
    for i in range(3):
        radius_a = half_extents_a[i]
        radius_b = (half_extents_b[0] * abs_radius[i, 0] +
                    half_extents_b[1] * abs_radius[i, 1] +
                    half_extents_b[2] * abs_radius[i, 2])
        if abs(t[i]) > radius_a + radius_b:
            return False

    # Axes L = B0, B1, B2
    for i in range(3):
        radius_a = (half_extents_a[0] * abs_radius[0, i] +
                    half_extents_a[1] * abs_radius[1, i] +
                    half_extents_a[2] * abs_radius[2, i])
        radius_b = half_extents_b[i]
        if abs(t[0] * radius[0, i] + t[1] * radius[1, i] + t[2] * radius[2, i]) > radius_a + radius_b:
            return False

    # Cross products of axes L = A0 x B0, A0 x B1, ..., A2 x B2
    for i in range(3):
        for j in range(3):
            radius_a = (half_extents_a[(i+1)%3] * abs_radius[(i+2)%3, j] +
                        half_extents_a[(i+2)%3] * abs_radius[(i+1)%3, j])
            radius_b = (half_extents_b[(j+1)%3] * abs_radius[i, (j+2)%3] +
                        half_extents_b[(j+2)%3] * abs_radius[i, (j+1)%3])
            if abs(t[(i+2)%3] * radius[(i+1)%3, j] - t[(i+1)%3] * radius[(i+2)%3, j]) > radius_a + radius_b:
                return False

    return True

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
            temp_node = Node(None)
            temp_node.set_bbox(obb_from_aabb)
            if not test_obb_obb(temp_node, aabb_a_temp): continue
        elif bbox_type_B == "sphere":
            result, q = test_sphere_obb(aabb_b_temp, aabb_a_temp)
            if not result: continue
        elif bbox_type_B == "obb":
            result = test_obb_obb(aabb_a_temp, aabb_b_temp)
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

    collisions = BVH_collision_obb_iterative(node_list_A, node_list_B, bbox_type_B)
    logging.debug(f"collisions: {collisions}")
    logging.debug(f"size of collisions: {len(collisions)}")

    return collisions
import logging
from OBB import OBB
from node import Node
from collision_detection.collision_detection_obb import test_obb_obb
import numpy as np
from collision_detection.collision_detection_utils import build_obb_from_aabb, descend_A, test_sphere_AABB, test_triangle_against_triangle, descend_A_iter, test_AABB_OBB
import plotly.graph_objects as go

def test_AABB_AABB(aabb_a, aabb_b):

    logging.debug("test_AABB_AABB")

    if (aabb_a.get_bbox().maxs[0] < aabb_b.get_bbox().mins[0] or aabb_a.get_bbox().mins[0] > aabb_b.get_bbox().maxs[0]): return False
    if (aabb_a.get_bbox().maxs[1] < aabb_b.get_bbox().mins[1] or aabb_a.get_bbox().mins[1] > aabb_b.get_bbox().maxs[1]): return False
    if (aabb_a.get_bbox().maxs[2] < aabb_b.get_bbox().mins[2] or aabb_a.get_bbox().mins[2] > aabb_b.get_bbox().maxs[2]): return False

    return True

def plot_2aabb(aabb_a, aabb_b):

    min_xa = aabb_a.get_bbox().mins[0]
    min_ya = aabb_a.get_bbox().mins[1]
    min_za = aabb_a.get_bbox().mins[2]

    max_xa = aabb_a.get_bbox().maxs[0]
    max_ya = aabb_a.get_bbox().maxs[1]
    max_za = aabb_a.get_bbox().maxs[2]

    min_xb = aabb_b.get_bbox().mins[0]
    min_yb = aabb_b.get_bbox().mins[1]
    min_zb = aabb_b.get_bbox().mins[2]

    max_xb = aabb_b.get_bbox().maxs[0]
    max_yb = aabb_b.get_bbox().maxs[1]
    max_zb = aabb_b.get_bbox().maxs[2]

    fig = go.Figure()

    fig.add_trace(go.Mesh3d(
        x = [min_xa, min_xa, max_xa, max_xa, min_xa, min_xa, max_xa, max_xa],
        y = [min_ya, max_ya, max_ya, min_ya, min_ya, max_ya, max_ya, min_ya],
        z = [min_za, min_za, min_za, min_za, max_za, max_za, max_za, max_za],

        i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
        j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
        k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
        color='cyan',
        opacity=0.50,
        flatshading = True))
    
    fig.add_trace(go.Mesh3d(
        x = [min_xb, min_xb, max_xb, max_xb, min_xb, min_xb, max_xb, max_xb],
        y = [min_yb, max_yb, max_yb, min_yb, min_yb, max_yb, max_yb, min_yb],
        z = [min_zb, min_zb, min_zb, min_zb, max_zb, max_zb, max_zb, max_zb],

        i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
        j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
        k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
        color='cyan',
        opacity=0.50,
        flatshading = True))
    
    return fig

def plot_triangles_in_aabb(fig, aabb_a):

    triangles = aabb_a.triangles

    for triangle in triangles:

        x = np.array([triangle.vertices[0][0], triangle.vertices[1][0], triangle.vertices[2][0]])
        y = np.array([triangle.vertices[0][1], triangle.vertices[1][1], triangle.vertices[2][1]])
        z = np.array([triangle.vertices[0][2], triangle.vertices[1][2], triangle.vertices[2][2]])

        i = np.array([0])
        j = np.array([1])
        k = np.array([2])

        fig.add_trace(go.Mesh3d(x=x, y=y, z=z, alphahull=5, opacity=0.4, color='sienna', i=i, j=j, k=k))

    return fig

def BVH_collision_aabb_iterative(tree_a, tree_b, bbox_type_B):
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
            if not test_AABB_AABB(aabb_a_temp, aabb_b_temp): continue
        elif bbox_type_B == "sphere":
            if not test_sphere_AABB(aabb_b_temp.get_bbox(), aabb_a_temp): continue
        elif bbox_type_B == "obb":
            logging.debug("Add aabb - obb intersection test")
            if not test_AABB_OBB(aabb_a_temp, aabb_b_temp): continue

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

def collision_detection_AABB(node_list_A, node_list_B, bbox_type_B):

    logging.debug("collision_detection for AABB")

    collisions = []

    collisions = BVH_collision_aabb_iterative(node_list_A, node_list_B, bbox_type_B)
    logging.debug(f"collisions: {collisions}")
    logging.debug(f"size of collisions: {len(collisions)}")

    return collisions

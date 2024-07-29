import logging
from obj_functions import save_obj, plot_two_obj_file
import numpy as np
import math
from collision_detection.collision_detection_utils import descend_A, test_triangle_against_triangle
import plotly.graph_objects as go
import time

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

def BVH_collision_aabb(tree_a, tree_b, index_a, index_b, collisions):

    logging.debug(f"index_a: {index_a}")
    logging.debug(f"index_b: {index_b}")

    logging.debug(f"tree_a: {tree_a}")
    logging.debug(f"tree_b: {tree_b}")

    aabb_a_temp = tree_a[index_a]
    aabb_b_temp = tree_b[index_b]

    if not test_AABB_AABB(aabb_a_temp, aabb_b_temp): return None

    if aabb_a_temp.is_leaf() and aabb_b_temp.is_leaf():
        logging.debug("Checking collisions on objects level.")
        fig = plot_2aabb(aabb_a_temp, aabb_b_temp)
        fig = plot_triangles_in_aabb(fig, aabb_a_temp)
        fig = plot_triangles_in_aabb(fig, aabb_b_temp)
        tri_collisions = test_triangle_against_triangle(aabb_a_temp, aabb_b_temp, collisions)
        logging.debug(f"tri_collisions: {len(tri_collisions)}")
    else:
        if descend_A(tree_a, index_a):
            index_a_one = tree_a.index(aabb_a_temp.left)
            index_a_two = tree_a.index(aabb_a_temp.right)
            BVH_collision_aabb(tree_a, tree_b, index_a_one, index_b, collisions)
            BVH_collision_aabb(tree_a, tree_b, index_a_two, index_b, collisions)
        else:
            index_b_one = tree_b.index(aabb_b_temp.left)
            index_b_two = tree_b.index(aabb_b_temp.right)
            BVH_collision_aabb(tree_a, tree_b, index_a, index_b_one, collisions)
            BVH_collision_aabb(tree_a, tree_b, index_a, index_b_two, collisions)
    
    return collisions
    

def collision_detection_AABB(node_list_A, node_list_B):

    logging.debug("collision_detection for AABB")

    filename_A = save_obj(node_list_A, 'A')
    filename_B = save_obj(node_list_B, 'B')

    aabb_a = node_list_A[0]
    aabb_b = node_list_B[0]

    logging.debug(f"aabb_a: {aabb_a}")
    logging.debug(f"aabb_b: {aabb_b}")

    logging.debug(f"aabb_a mins and maxs: {aabb_a.get_bbox().mins}, {aabb_a.get_bbox().maxs}")
    logging.debug(f"aabb_b mins and maxs: {aabb_b.get_bbox().mins}, {aabb_b.get_bbox().maxs}")

    collisions = []

    collisions = BVH_collision_aabb(node_list_A, node_list_B, 0, 0, collisions)
    logging.debug(f"collisions: {collisions}")
    logging.debug(f"size of collisions: {len(collisions)}")

    return collisions

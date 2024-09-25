import logging
import numpy as np
import math
import sys

def test_triangle_against_triangle(aabb_a_temp, aabb_b_temp):

    triangles_a = aabb_a_temp.triangles
    triangles_b = aabb_b_temp.triangles
    logging.debug(f"size of triangles_a: {len(triangles_a)}")
    logging.debug(f"size of triangles_b: {len(triangles_b)}")
    tri_collisions = []

    logging.debug(f"triangles_a: {triangles_a}")

    for triangle_a in triangles_a:
        for triangle_b in triangles_b:
            logging.debug(f"triangle_a: {triangle_a}")
        
            triangle_intersect = triangles_intersection(triangle_a, triangle_b)
            logging.debug(f"triangle_intersect: {triangle_intersect}")
            if triangle_intersect:
                tri_collisions.append([triangle_a, triangle_b])
                logging.debug(f"appedned tri_collisions: {tri_collisions}")

    return tri_collisions

def triangles_intersection(triangle_a, triangle_b):

    # Unpack vertices of triangles
    logging.debug(f"triangle_a in function: {triangle_a}")

    V0 = np.array([triangle_a.vertices[0][0], triangle_a.vertices[0][1], triangle_a.vertices[0][2]])
    V1 = np.array([triangle_a.vertices[1][0], triangle_a.vertices[1][1], triangle_a.vertices[1][2]])
    V2 = np.array([triangle_a.vertices[2][0], triangle_a.vertices[2][1], triangle_a.vertices[2][2]])
     
    #V0, V1, V2 = triangle_a
    #U0, U1, U2 = triangle_b

    U0 = np.array([triangle_b.vertices[0][0], triangle_b.vertices[0][1], triangle_b.vertices[0][2]])
    U1 = np.array([triangle_b.vertices[1][0], triangle_b.vertices[1][1], triangle_b.vertices[1][2]])
    U2 = np.array([triangle_b.vertices[2][0], triangle_b.vertices[2][1], triangle_b.vertices[2][2]])

    logging.debug(f"V0: {V0}")
    logging.debug(f"V1: {V1}")
    logging.debug(f"V2: {V2}")

    # Edge vectors for tri1
    E1 = subtract(V1, V0)
    E2 = subtract(V2, V0)
    E3 = subtract(V2, V1)

    # Edge vectors for tri2
    F1 = subtract(U1, U0)
    F2 = subtract(U2, U0)
    F3 = subtract(U2, U1)

    # Axes to test
    axes = [
        cross_product(E1, E2),   # Normal of tri1
        cross_product(F1, F2),   # Normal of tri2
        cross_product(E1, F1),
        cross_product(E1, F2),
        cross_product(E1, F3),
        cross_product(E2, F1),
        cross_product(E2, F2),
        cross_product(E2, F3),
        cross_product(E3, F1),
        cross_product(E3, F2),
        cross_product(E3, F3)
    ]

    for axis in axes:
        if np.linalg.norm(axis) < sys.float_info.epsilon:
            continue
        min1, max1 = project(triangle_a, axis)
        min2, max2 = project(triangle_b, axis)
        if not overlap(min1, max1, min2, max2):
            return False

    return True

def descend_larger_method(tree_a, tree_b, index_a, index_b):

    descend = tree_b[index_b].is_leaf() or (not tree_a[index_a].is_leaf() and len(tree_a) >= len(tree_b))
    
    return descend

def descend_A(tree_a, index_a):

    return not tree_a[index_a].is_leaf()

def descend_A_iter(aabb_a_temp):

    return not aabb_a_temp.is_leaf()

def cross_product(a, b):
    return np.array([a[1]*b[2] - a[2]*b[1],
                     a[2]*b[0] - a[0]*b[2],
                     a[0]*b[1] - a[1]*b[0]])

def dot_product(a, b):
    return np.dot(a, b)

def subtract(a, b):
    return np.array([a[0] - b[0], a[1] - b[1], a[2] - b[2]])

def project(triangle, axis):
    dots = [dot_product(vertex, axis) for vertex in triangle.vertices]
    return min(dots), max(dots)

def overlap(min1, max1, min2, max2):
    return not (max1 < min2 or max2 < min1)

def build_obb_from_aabb(aabb_b_temp):

    mins = aabb_b_temp.get_bbox().mins
    maxs = aabb_b_temp.get_bbox().maxs

    logging.debug(f"mins: {mins}")
    logging.debug(f"maxs: {maxs}")

    half_extents = (np.array(maxs) - np.array(mins)) / 2
    center = mins + half_extents
    diff = half_extents

    corners = np.array([center + [-diff[0], -diff[1], -diff[2]],
                        center + [diff[0], diff[1], diff[2]],
                        center + [diff[0], -diff[1], diff[2]],
                        center + [diff[0], -diff[1], -diff[2]],
                        center + [diff[0], diff[1], -diff[2]],
                        center + [-diff[0], diff[1], diff[2]],
                        center + [-diff[0], -diff[1], diff[2]],
                        center + [-diff[0], diff[1], -diff[2]]])
    
    rotation = np.eye(3)
    logging.debug(f"rotation of aabb turned into obb: {rotation}")
    logging.debug(f"center of aabb turned into obb: {center}")
    logging.debug(f"corners of aabb turned into obb: {corners}")
    logging.debug(f"diff of aabb turned into obb: {diff}")

    return corners, center, diff, rotation

def closest_pt_point_obb(center, obb):

    logging.debug("closest_pt_point_obb")
    logging.debug(f"sphere center: {center}, obb centre: {obb.get_bbox().centre}")

    d = np.array(center) - np.array(obb.get_bbox().centre)
    q = np.array(obb.get_bbox().centre)

    logging.debug(f"d: {d}")
    logging.debug(f"q: {q}")

    u = np.array([obb.get_bbox().rotation[:, 0], obb.get_bbox().rotation[:, 1], obb.get_bbox().rotation[:, 2]])
    logging.debug(f"u: {u}")

    for i in range(3):
        dist = np.dot(d, u[i])
        if dist > obb.get_bbox().half_extents[i]: dist = obb.get_bbox().half_extents[i]
        if dist < -obb.get_bbox().half_extents[i]: dist = -obb.get_bbox().half_extents[i]
        q += (dist * u[i])
        logging.debug(f"q: {q}")


    logging.debug(f"q: {q}")

    return q

def sq_dist_point_AABB(p, aabb):

    sq_dist = 0

    for i in range(3):
        v = p[i]
        if v < aabb.mins[i]:
            sq_dist += (aabb.mins[i] - v) * (aabb.mins[i] - v)
        if v > aabb.maxs[i]:
            sq_dist += (v - aabb.maxs[i]) * (v - aabb.max[i])
    return sq_dist

def closest_pt_point_AABB(p, aabb):

    q = np.zeros(3)

    for i in range(3):
        v = p[i]
        if v < aabb.get_bbox().mins[i]: v = aabb.get_bbox().mins[i]
        if v > aabb.get_bbox().maxs[i]: v = aabb.get_bbox().maxs[i]
        q[i] = v

    return q

def test_sphere_AABB(sphere, aabb):

    p = sphere.centre
    q = closest_pt_point_AABB(p, aabb)
    v = q - p
    return np.dot(v,v) <= sphere.radius * sphere.radius
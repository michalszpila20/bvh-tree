import logging
import numpy as np
import math
import sys

def test_triangle_against_triangle(aabb_a_temp, aabb_b_temp, collisions):

    triangles_a = aabb_a_temp.triangles
    triangles_b = aabb_b_temp.triangles
    logging.debug(f"size of triangles_a: {len(triangles_a)}")
    logging.debug(f"size of triangles_b: {len(triangles_b)}")
    tri_collisions = collisions

    logging.debug(f"triangles_a: {triangles_a}")

    for triangle_a in triangles_a:
        for triangle_b in triangles_b:
            logging.debug(f"triangle_a: {triangle_a}")
        
            triangle_intersect = triangles_intersection(triangle_a, triangle_b)
            logging.debug(f"triangle_intersect: {triangle_intersect}")
            if triangle_intersect:
                tri_collisions.append([triangle_a, triangle_b])
    
    return tri_collisions

def intersect_line_triangle(ray_origin, ray_dest, triangle):

    a = np.array([triangle.vertices[0][0], triangle.vertices[0][1], triangle.vertices[0][2]])
    b = np.array([triangle.vertices[1][0], triangle.vertices[1][1], triangle.vertices[1][2]])
    c = np.array([triangle.vertices[2][0], triangle.vertices[2][1], triangle.vertices[2][2]])

    p = np.array(ray_origin)
    q = np.array(ray_dest)

    pq = q - p
    pa = a - p
    pb = b - p
    pc = c - p

    cross_pc = np.cross(pq, pc)
    cross_pa = np.cross(pq, pa)
    cross_pb = np.cross(pq, pb)

    u = np.dot(pb, cross_pc)
    v = np.dot(pc, cross_pa)
    w = np.dot(pa, cross_pb)

    uvw = [u, v, w]

    if all(item >= 0 for item in uvw) or all(item < 0 for item in uvw):
        logging.debug("u,v,w have the same sign.")
    else:
        logging.debug("u,v,w does not have the same sign.")
        return False

    denom = 1 / (u + v + w)
    u *= denom
    v *= denom
    w *= denom

    r = u * a + v * b + w * c
    logging.debug(f"r: {r}")

    # if math.dist(ray_origin, ray_dest) < math.dist(ray_origin, r):
    #     return False
    # else:
    #     return True

    if np.linalg.norm(pq) < np.linalg.norm(r - p):
        return False
    else:
        return True

def triangles_intersection(triangle_a, triangle_b):

    # if intersect_line_triangle(np.array([triangle_a.vertices[0][0], triangle_a.vertices[0][1], triangle_a.vertices[0][2]]),
    #                            np.array([triangle_a.vertices[1][0], triangle_a.vertices[1][1], triangle_a.vertices[1][2]]),
    #                             triangle_b): return True
    
    # if intersect_line_triangle(np.array([triangle_a.vertices[1][0], triangle_a.vertices[1][1], triangle_a.vertices[1][2]]),
    #                            np.array([triangle_a.vertices[2][0], triangle_a.vertices[2][1], triangle_a.vertices[2][2]]),
    #                             triangle_b): return True
    
    # if intersect_line_triangle(np.array([triangle_a.vertices[0][0], triangle_a.vertices[0][1], triangle_a.vertices[0][2]]),
    #                            np.array([triangle_a.vertices[2][0], triangle_a.vertices[2][1], triangle_a.vertices[2][2]]),
    #                             triangle_b): return True
    
    # if intersect_line_triangle(np.array([triangle_b.vertices[0][0], triangle_b.vertices[0][1], triangle_b.vertices[0][2]]),
    #                            np.array([triangle_b.vertices[1][0], triangle_b.vertices[1][1], triangle_b.vertices[1][2]]),
    #                             triangle_a): return True
    
    # if intersect_line_triangle(np.array([triangle_b.vertices[1][0], triangle_b.vertices[1][1], triangle_b.vertices[1][2]]),
    #                            np.array([triangle_b.vertices[2][0], triangle_b.vertices[2][1], triangle_b.vertices[2][2]]),
    #                             triangle_a): return True
    
    # if intersect_line_triangle(np.array([triangle_b.vertices[0][0], triangle_b.vertices[0][1], triangle_b.vertices[0][2]]),
    #                            np.array([triangle_b.vertices[2][0], triangle_b.vertices[2][1], triangle_b.vertices[2][2]]),
    #                             triangle_a): return True

    # return False

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
import logging
import numpy as np
from CGAL.CGAL_Kernel import Point_3, Triangle_3, do_intersect

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
        
            triangle_intersect = triangles_intersection_CGAL(triangle_a, triangle_b)
            logging.debug(f"triangle_intersect: {triangle_intersect}")
            if triangle_intersect:
                tri_collisions.append([triangle_a, triangle_b])
                logging.debug(f"appedned tri_collisions: {tri_collisions}")

    return tri_collisions

def triangles_intersection_CGAL(triangle_a, triangle_b):

    a = Point_3(triangle_a.vertices[0][0], triangle_a.vertices[0][1], triangle_a.vertices[0][2])
    b = Point_3(triangle_a.vertices[1][0], triangle_a.vertices[1][1], triangle_a.vertices[1][2])
    c = Point_3(triangle_a.vertices[2][0], triangle_a.vertices[2][1], triangle_a.vertices[2][2])

    p = Point_3(triangle_b.vertices[0][0], triangle_b.vertices[0][1], triangle_b.vertices[0][2])
    q = Point_3(triangle_b.vertices[1][0], triangle_b.vertices[1][1], triangle_b.vertices[1][2])
    k = Point_3(triangle_b.vertices[2][0], triangle_b.vertices[2][1], triangle_b.vertices[2][2])

    triangle1 = Triangle_3(a, b, c)
    triangle2 = Triangle_3(p, q, k)

    result = do_intersect(triangle1, triangle2)
    logging.debug(f"Intersection result: {result}")
    if result:
        return True
    else:
        return False

def descend_larger_method(tree_a, tree_b, index_a, index_b):

    descend = tree_b[index_b].is_leaf() or (not tree_a[index_a].is_leaf() and len(tree_a) >= len(tree_b))
    
    return descend

def descend_A(tree_a, index_a):

    return not tree_a[index_a].is_leaf()

def descend_A_iter(aabb_a_temp):

    return not aabb_a_temp.is_leaf()

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
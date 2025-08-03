import logging
import numpy as np

def test_triangle_against_triangle(aabb_a_temp, aabb_b_temp):

    triangles_a = aabb_a_temp.triangles
    triangles_b = aabb_b_temp.triangles
    logging.debug(f"size of triangles_a: {len(triangles_a)}")
    logging.debug(f"size of triangles_b: {len(triangles_b)}")
    tri_collisions = []

    logging.debug(f"triangles_a: {triangles_a}")

    for triangle_a in triangles_a:
        for triangle_b in triangles_b:
            axes = calculate_axes(triangle_a, triangle_b)
            logging.debug(f"triangle_a: {triangle_a}")
            triangle_intersect = not test_separating_axis(axes, triangle_a, triangle_b)
            logging.debug(f"triangle_intersect: {triangle_intersect}")
            if triangle_intersect:
                tri_collisions.append([triangle_a, triangle_b])
                logging.debug(f"appedned tri_collisions: {tri_collisions}")

    return tri_collisions

def test_separating_axis(axes, triangle_A, triangle_B):
    
    p1, p2, p3 = triangle_A.vertices
    q1, q2, q3 = triangle_B.vertices

    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)

    q1 = np.array(q1)
    q2 = np.array(q2)
    q3 = np.array(q3)

    for axis in axes:

        dot_A_1 = np.dot(p1, axis)
        dot_A_2 = np.dot(p2, axis)
        dot_A_3 = np.dot(p3, axis)

        min_A = min(dot_A_1, dot_A_2, dot_A_3)
        max_A = max(dot_A_1, dot_A_2, dot_A_3)

        dot_B_1 = np.dot(q1, axis)
        dot_B_2 = np.dot(q2, axis)
        dot_B_3 = np.dot(q3, axis)

        min_B = min(dot_B_1, dot_B_2, dot_B_3)
        max_B = max(dot_B_1, dot_B_2, dot_B_3)

        if max_A < min_B or max_B < min_A:
            return True
        
    return False

def calculate_axes(triangle_A, triangle_B):

    p1, p2, p3 = triangle_A.vertices
    q1, q2, q3 = triangle_B.vertices

    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)

    q1 = np.array(q1)
    q2 = np.array(q2)
    q3 = np.array(q3)

    triangle_p_normal = np.cross(p2 - p1, p3 - p1)
    triangle_q_normal = np.cross(q2 - q1, q3 - q1)

    edges_p = [p2 - p1, p3 - p2, p1 - p3]
    edges_q = [q2 - q1, q3 - q2, q1 - q3]

    axes = [triangle_p_normal, triangle_q_normal]
    for edge_p in edges_p:
        for edge_q in edges_q:
            cross_product = np.cross(edge_p, edge_q)
            magnitude = np.linalg.norm(cross_product)

            if magnitude <= 1e-10:
                continue

            axis_norm = cross_product / magnitude
            axes.append(axis_norm)

    return axes

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

def test_AABB_OBB(aabb, obb):
    
    rotation_obb = obb.get_bbox().rotation

    center_a = aabb.get_bbox().centre
    center_b = obb.get_bbox().centre

    half_extents_aabb = aabb.get_bbox().half_extents
    half_extents_obb = obb.get_bbox().half_extents

    EPSILON = 1e-6
    radius = rotation_obb
    abs_radius = np.abs(rotation_obb) + EPSILON

    # Compute translation vector t between OBB centers and project it onto obb_a's local axes
    t = center_b - center_a

    # Axes L = A0, A1, A2
    for i in range(3):
        radius_a = half_extents_aabb[i]
        radius_b = (half_extents_obb[0] * abs_radius[i, 0] +
                    half_extents_obb[1] * abs_radius[i, 1] +
                    half_extents_obb[2] * abs_radius[i, 2])
        if abs(t[i]) > radius_a + radius_b:
            return False

    # Axes L = B0, B1, B2
    for i in range(3):
        radius_a = (half_extents_aabb[0] * abs_radius[0, i] +
                    half_extents_aabb[1] * abs_radius[1, i] +
                    half_extents_aabb[2] * abs_radius[2, i])
        radius_b = half_extents_obb[i]
        if abs(t[0] * radius[0, i] + t[1] * radius[1, i] + t[2] * radius[2, i]) > radius_a + radius_b:
            return False

    # Cross products of axes L = A0 x B0, A0 x B1, ..., A2 x B2
    for i in range(3):
        for j in range(3):
            radius_a = (half_extents_aabb[(i+1)%3] * abs_radius[(i+2)%3, j] +
                        half_extents_aabb[(i+2)%3] * abs_radius[(i+1)%3, j])
            radius_b = (half_extents_obb[(j+1)%3] * abs_radius[i, (j+2)%3] +
                        half_extents_obb[(j+2)%3] * abs_radius[i, (j+1)%3])
            if abs(t[(i+2)%3] * radius[(i+1)%3, j] - t[(i+1)%3] * radius[(i+2)%3, j]) > radius_a + radius_b:
                return False

    return True
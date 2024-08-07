import logging
import time
import numpy as np
from collision_detection.collision_detection_utils import closest_pt_point_obb, test_triangle_against_triangle, descend_A, test_sphere_AABB
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

def plot_sphere_obb(sphere, obb, result, q):

    box_a = obb.get_bbox()

    fig = go.Figure()
    fig.add_trace(go.Mesh3d(
                
        x = box_a.corners[:, 0],
        y = box_a.corners[:, 1],
        z = box_a.corners[:, 2],

        i = [0, 5, 1, 2, 2, 3, 6, 1, 7, 4, 5, 5],
        j = [6, 0, 4, 4, 3, 0, 5, 2, 0, 7, 7, 1],
        k = [5, 7, 2, 3, 6, 6, 2, 5, 3, 3, 4, 4],
        color='gray',
        opacity=0.1,
        alphahull = 44,
        flatshading = True,
                
        lighting=dict(ambient=0.1,
                    diffuse=1,
                    fresnel=4,
                    specular=0.5,
                    roughness=0.05),
        lightposition=dict(x=100,
                            y=200,
                            z=100)
            ))
    
    resolution=101
    x = sphere.get_bbox().centre[0]
    y = sphere.get_bbox().centre[1]
    z = sphere.get_bbox().centre[2]

    u, v = np.mgrid[0:2*np.pi:resolution*2j, 0:np.pi:resolution*1j]
    X = sphere.get_bbox().radius * np.cos(u)*np.sin(v) + x
    Y = sphere.get_bbox().radius * np.sin(u)*np.sin(v) + y
    Z = sphere.get_bbox().radius * np.cos(v) + z

    fig.add_trace(go.Surface(x=X, y=Y, z=Z, opacity=0.1))

    fig.add_trace(go.Scatter3d(x=[q[0]], y=[q[1]], z = [q[2]], marker_size=2))

    fig.update_layout(title_text=f"Sphere x OBB test result: {result}.")
    fig.show()

def test_sphere_obb(sphere, obb):

    logging.debug("test_sphere_obb")

    q = 0

    center = sphere.get_bbox().centre
    radius = sphere.get_bbox().radius

    logging.debug(f"center, sphere x obb: {center}")
    logging.debug(f"radius: {radius}")

    q = closest_pt_point_obb(center, obb)

    logging.debug(f"q: {q}")

    v = q - center

    logging.debug(f"v: {v}")
    logging.debug(f"np.dot(v, v): {np.dot(v, v)}, radius^2: {radius * radius}, np.dot(v, v) <= radius * radius: {np.dot(v, v) <= radius * radius}")


    return np.dot(v, v) <= radius * radius, q

def BVH_collision_sphere(tree_a, tree_b, index_a, index_b, bbox_type_B, collisions):

    logging.debug(f"index_a: {index_a}")
    logging.debug(f"index_b: {index_b}")

    logging.debug(f"tree_a: {tree_a}")
    logging.debug(f"tree_b: {tree_b}")

    sphere_A = tree_a[index_a]
    sphere_B = tree_b[index_b]

    if bbox_type_B == "sphere":
        if not test_sphere_sphere(sphere_A, sphere_B): return None
    elif bbox_type_B == "aabb":
        if not test_sphere_AABB(sphere_A.get_bbox(), sphere_B): return None
    elif bbox_type_B == "obb":
        result, q = test_sphere_obb(sphere_A, sphere_B)
        if not result: return None

    if sphere_A.is_leaf() and sphere_B.is_leaf():
        logging.debug("Checking collisions on objects level.")
        test_triangle_against_triangle(sphere_A, sphere_B, collisions)
    else:
        if descend_A(tree_a, index_a):
            logging.debug("descend_A")
            index_a_one = tree_a.index(sphere_A.left)
            index_a_two = tree_a.index(sphere_A.right)
            BVH_collision_sphere(tree_a, tree_b, index_a_one, index_b, bbox_type_B, collisions)
            BVH_collision_sphere(tree_a, tree_b, index_a_two, index_b, bbox_type_B, collisions)
        else:
            logging.debug("descend_B")
            index_b_one = tree_b.index(sphere_B.left)
            index_b_two = tree_b.index(sphere_B.right)
            BVH_collision_sphere(tree_a, tree_b, index_a, index_b_one, bbox_type_B, collisions)
            BVH_collision_sphere(tree_a, tree_b, index_a, index_b_two, bbox_type_B, collisions)
    
    return collisions

def collision_detection_sphere(node_list_A, node_list_B, bbox_type_B):
    logging.debug("collision_detection_sphere")

    collisions = []

    collisions = BVH_collision_sphere(node_list_A, node_list_B, 0, 0, bbox_type_B, collisions)

    logging.debug(f"collisions: {collisions}")
    logging.debug(f"number of collisions: {len(collisions)}")
    return collisions

   
import numpy as np
import math
import plotly.graph_objects as go
import logging

def plot_triangle_ray(triangle, ray_origin, ray_dest):
    
    x = [triangle.vertices[0][0], triangle.vertices[1][0], triangle.vertices[2][0]]
    y = [triangle.vertices[0][1], triangle.vertices[1][1], triangle.vertices[2][1]]
    z = [triangle.vertices[0][2], triangle.vertices[1][2], triangle.vertices[2][2]]

    logging.debug(f"x: {x}")
    logging.debug(f"y: {y}")
    logging.debug(f"z: {z}")

    i = np.array([0])
    j = np.array([1])
    k = np.array([2])

    fig = go.Figure()
    fig.add_trace(go.Mesh3d(x=x, y=y, z=z, alphahull=5, opacity=0.4, color='cyan', i=i, j=j, k=k))
    fig.add_trace(
    go.Scatter3d(x=[ray_origin[0], ray_dest[0]],
                 y=[ray_origin[1], ray_dest[1]],
                 z=[ray_origin[2], ray_dest[2]],
                 mode='lines'))
    fig.show()

def plot_OBB_ray(node_list_A, ray_origin, ray_dest):
    
    corners = []

    for node in node_list_A:
        corner = node.get_bbox().corners
        corners.append(corner)

    fig = go.Figure()

    fig.add_trace(go.Mesh3d(
                
                x = corners[:, 0],
                y = corners[:, 1],
                z = corners[:, 2],

                i = [0, 5, 1, 2, 2, 3, 6, 1, 7, 4, 5, 5],
                j = [6, 0, 4, 4, 3, 0, 5, 2, 0, 7, 7, 1],
                k = [5, 7, 2, 3, 6, 6, 2, 5, 3, 3, 4, 4],
                color='gray',
                opacity=1,
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
    
    fig.add_trace(
    go.Scatter3d(x=[ray_origin[0], ray_dest[0]],
                 y=[ray_origin[1], ray_dest[1]],
                 z=[ray_origin[2], ray_dest[2]],
                 mode='lines'))
    
    fig.show()

def intersect_line_triangle_boolean(current_node, ray_origin, ray_dest):

    triangles = current_node.get_triangles()
    logging.debug(f"triangles: {triangles}")

    logging.debug("Testing a line and a triangle!")

    ray_dest_new = ray_dest

    for triangle in triangles:

        logging.debug(f"triangle.vertices: {triangle.vertices}")

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
            continue

        denom = 1 / (u + v + w)
        u *= denom
        v *= denom
        w *= denom

        r = u * a + v * b + w * c

        logging.debug(f"r: {r}")

        if math.dist(ray_dest_new, ray_origin) > math.dist(r, ray_origin):
            ray_dest_new = r
            plot_triangle_ray(triangle, ray_origin, ray_dest_new)
            return True
        else:
            continue

    return False

def intersect_line_triangle_closest(current_node, ray_origin, ray_dest, closest_hit):

    triangles = current_node[0].get_triangles()

    logging.debug(f"triangles: {triangles}")

    logging.debug("Testing a line and a triangle!")

    ray_dest_new = ray_dest
    closest_hit_new = closest_hit

    for triangle in triangles:

        logging.debug(f"triangle.vertices: {triangle.vertices}")

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
            continue

        denom = 1 / (u + v + w)
        u *= denom
        v *= denom
        w *= denom

        r = u * a + v * b + w * c

        logging.debug(f"r: {r}")

        if math.dist(ray_dest_new, ray_origin) > math.dist(r, ray_origin):
            ray_dest_new = r
            closest_hit_new = triangle
            logging.debug(f"closest_hit_new: {closest_hit_new} inside if")

        plot_triangle_ray(triangle, ray_origin, ray_dest_new)

    return ray_dest_new, closest_hit_new
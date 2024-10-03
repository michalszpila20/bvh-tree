import time
import plotly.graph_objects as go
import numpy as np
from AABB import AABB
from OBB import OBB
import math
from sphere import sphere 
import logging
from collections import defaultdict
import os

def open_obj_file(filename):

    # vertices coordiantes
    verticesX = []
    verticesY = []
    verticesZ = []

    # faces (triangles) 
    verticesI = []
    verticesJ = []
    verticesK = []

    # options: bear / cow / teapot / pumpkin
    file = open(filename)

    Lines = file.readlines()

    for line in Lines:
        if line.__contains__('f'):
            if line.__contains__('/'):
                new_list = [line.strip().split()[1].split('/')[0], line.strip().split()[2].split('/')[0], line.strip().split()[3].split('/')[0]]
                verticesI.append(int(new_list[0]) -1)
                verticesJ.append(int(new_list[1]) -1)
                verticesK.append(int(new_list[2]) -1)
            else:
                verticesI.append(int(line.strip().split()[1]) -1)
                verticesJ.append(int(line.strip().split()[2]) -1)
                verticesK.append(int(line.strip().split()[3]) -1)   
        elif line.__contains__('vn'):
            continue
        elif line.__contains__('vt'):
            continue
        elif line.__contains__('v'):
            verticesX.append(float(line.strip().split()[1]))
            verticesY.append(float(line.strip().split()[2]))
            verticesZ.append(float(line.strip().split()[3]))

 
    return verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK

def plot_obj_file(verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK):
    fig = go.Figure(data=[
    go.Mesh3d(
        x=verticesX,
        y=verticesY,
        z=verticesZ,
            
        color='blue',
        opacity=0.1,
            
        i=verticesI[0:-1],
        j=verticesJ[0:-1],
        k=verticesK[0:-1]
        )
    ])


def plot_2obj_file(verticesX_rotated_moved_A, verticesY_rotated_moved_A, verticesZ_rotated_moved_A, verticesI_A, verticesJ_A, verticesK_A,
                   verticesX_rotated_moved_B, verticesY_rotated_moved_B, verticesZ_rotated_moved_B, verticesI_B, verticesJ_B, verticesK_B):
    fig = go.Figure(data=[
    go.Mesh3d(
        x=verticesX_rotated_moved_A,
        y=verticesY_rotated_moved_A,
        z=verticesZ_rotated_moved_A,
            
        color='blue',
        opacity=0.1,
            
        i=verticesI_A[0:-1],
        j=verticesJ_A[0:-1],
        k=verticesK_A[0:-1]
        )
    ])

    fig.add_trace(go.Mesh3d(
        x=verticesX_rotated_moved_B,
        y=verticesY_rotated_moved_B,
        z=verticesZ_rotated_moved_B,
            
        color='red',
        opacity=0.1,
            
        i=verticesI_B[0:-1],
        j=verticesJ_B[0:-1],
        k=verticesK_B[0:-1]
        )
    )

    fig.show()

def plot_two_obj_file(filename_A, filename_B):
    
    verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK = open_obj_file(filename_A)
    verticesX1, verticesY1, verticesZ1, verticesI1, verticesJ1, verticesK1 = open_obj_file(filename_B)

    fig = go.Figure(data=[
    go.Mesh3d(
        x=verticesX,
        y=verticesY,
        z=verticesZ,
            
        color='blue',
        opacity=0.1,
            
        i=verticesI[0:-1],
        j=verticesJ[0:-1],
        k=verticesK[0:-1]
        )
    ])

    fig.add_trace(go.Mesh3d(
        x=verticesX1,
        y=verticesY1,
        z=verticesZ1,
            
        color='red',
        opacity=0.1,
            
        i=verticesI1[0:-1],
        j=verticesJ1[0:-1],
        k=verticesK1[0:-1]
        )
    )

    return fig

def calculate_box_AABB(obj_list_copy):

    vertices_x = []
    vertices_y = []
    vertices_z = []

    for triangle in obj_list_copy:
        #Triangle vertex 1
        vertices_x.append(triangle.vertices[0][0])
        vertices_y.append(triangle.vertices[0][1])
        vertices_z.append(triangle.vertices[0][2])
        #Triangle vertex 2
        vertices_x.append(triangle.vertices[1][0])
        vertices_y.append(triangle.vertices[1][1])
        vertices_z.append(triangle.vertices[1][2])
        #Triangle vertex 3
        vertices_x.append(triangle.vertices[2][0])
        vertices_y.append(triangle.vertices[2][1])
        vertices_z.append(triangle.vertices[2][2])

    min_s = [min(vertices_x), min(vertices_y), min(vertices_z)]
    max_s = [max(vertices_x), max(vertices_y), max(vertices_z)]

    world_box = AABB(min_s, max_s)

    return world_box

def adjust_sphere(sphere, point):

    d = point - sphere.centre
    dist2 = np.dot(d, d)

    if dist2 > sphere.radius * sphere.radius:
        logging.debug("Inside!")
        dist = np.sqrt(dist2)
        new_radius = (sphere.radius + dist) * 0.5
        k = (new_radius - sphere.radius) / dist
        sphere.radius = new_radius
        sphere.centre += d * k
    
    return sphere

def calculate_box_sphere_ritter(obj_list_copy):

    logging.debug("Ritter method!")

    vertices_x = []
    vertices_y = []
    vertices_z = []
    points = []

    for triangle in obj_list_copy:
        #Triangle vertex 1
        vertices_x.append(triangle.vertices[0][0])
        vertices_y.append(triangle.vertices[0][1])
        vertices_z.append(triangle.vertices[0][2])
        points.append([triangle.vertices[0][0], triangle.vertices[0][1], triangle.vertices[0][2]])
        #Triangle vertex 2
        vertices_x.append(triangle.vertices[1][0])
        vertices_y.append(triangle.vertices[1][1])
        vertices_z.append(triangle.vertices[1][2])
        points.append([triangle.vertices[1][0], triangle.vertices[1][1], triangle.vertices[1][2]])
        #Triangle vertex 3
        vertices_x.append(triangle.vertices[2][0])
        vertices_y.append(triangle.vertices[2][1])
        vertices_z.append(triangle.vertices[2][2])
        points.append([triangle.vertices[2][0], triangle.vertices[2][1], triangle.vertices[2][2]])

    min_x = 0
    max_x = 0
    min_y = 0
    max_y = 0
    min_z = 0
    max_z = 0

    logging.debug(f"len(points): {len(points)}")

    for i in range(1, len(points)):
        if points[i][0] < points[min_x][0]: min_x = i
        if points[i][0] > points[max_x][0]: max_x = i
        if points[i][1] < points[min_y][1]: min_y = i
        if points[i][1] > points[max_y][1]: max_y = i
        if points[i][2] < points[min_z][2]: min_z = i
        if points[i][2] > points[max_z][2]: max_z = i

    logging.debug(f"min_x :{min_x}")
    logging.debug(f"max_x :{max_x}")
    logging.debug(f"min_y :{min_y}")
    logging.debug(f"max_y :{max_y}")
    logging.debug(f"min_z :{min_z}")
    logging.debug(f"max_z :{max_z}")

    logging.debug(f"points[max_x]: {points[max_x]}, points[min_x]: {points[min_x]}")

    point_x1 = np.array([points[min_x][0], points[min_x][1], points[min_x][2]])
    point_x2 = np.array([points[max_x][0], points[max_x][1], points[max_x][2]])

    point_y1 = np.array([points[min_y][0], points[min_y][1], points[min_y][2]])
    point_y2 = np.array([points[max_y][0], points[max_y][1], points[max_y][2]])

    point_z1 = np.array([points[min_z][0], points[min_z][1], points[min_z][2]])
    point_z2 = np.array([points[max_z][0], points[max_z][1], points[max_z][2]])
    
    dist2x = np.linalg.norm(point_x1 - point_x2)
    dist2y = np.linalg.norm(point_y1 - point_y2)
    dist2z = np.linalg.norm(point_z1 - point_z2)
    logging.debug(f"dist2x: {dist2x}")
    logging.debug(f"dist2y: {dist2y}")
    logging.debug(f"dist2z: {dist2z}")

    min = min_x
    max = max_x

    if dist2y > dist2x and dist2y > dist2y:
        max = max_y
        min = min_y
    
    if dist2z > dist2x and dist2z > dist2y:
        max = max_z
        min = min_z

    logging.debug(f"min: {min}")
    logging.debug(f"max: {max}")

    centre = (np.array([points[min][0], points[min][1], points[min][2]]) + np.array([points[max][0], points[max][1], points[max][2]])) * 0.5
    radius = np.sqrt(np.dot(points[max] - centre, points[max] - centre))
    logging.debug(f"centre: {centre}, radius: {radius}") 

    base_sphere = sphere(centre, radius)

    for i in range(1, len(points)):
        base_sphere = adjust_sphere(base_sphere, points[i])

    logging.debug(f"base_sphere: {base_sphere.centre} , {base_sphere.radius}")

def calculate_box_sphere(obj_list_copy):

    vertices_x = []
    vertices_y = []
    vertices_z = []
    points = []

    for triangle in obj_list_copy:
        #Triangle vertex 1
        vertices_x.append(triangle.vertices[0][0])
        vertices_y.append(triangle.vertices[0][1])
        vertices_z.append(triangle.vertices[0][2])
        points.append([triangle.vertices[0][0], triangle.vertices[0][1], triangle.vertices[0][2]])
        #Triangle vertex 2
        vertices_x.append(triangle.vertices[1][0])
        vertices_y.append(triangle.vertices[1][1])
        vertices_z.append(triangle.vertices[1][2])
        points.append([triangle.vertices[1][0], triangle.vertices[1][1], triangle.vertices[1][2]])
        #Triangle vertex 3
        vertices_x.append(triangle.vertices[2][0])
        vertices_y.append(triangle.vertices[2][1])
        vertices_z.append(triangle.vertices[2][2])
        points.append([triangle.vertices[2][0], triangle.vertices[2][1], triangle.vertices[2][2]])

    sphere_centre = [(min(vertices_x) + max(vertices_x)) / 2,
                     (min(vertices_y) + max(vertices_y)) / 2,
                     (min(vertices_z) + max(vertices_z)) / 2]

    logging.debug(f"sphere_centre: {sphere_centre}")

    radius = 0

    for point in points:
        distance = math.dist(sphere_centre, [point[0], point[1], point[2]])
        if distance > radius:
            radius = distance

    logging.debug(f"radius: {radius}")

    world_box = sphere(sphere_centre, radius)

    return world_box

def build_triangles(verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK):

    triangles = []

    for i in range(len(verticesI)):
        triangle_A = [verticesX[verticesI[i]], verticesY[verticesI[i]], verticesZ[verticesI[i]]]
        triangle_B = [verticesX[verticesJ[i]], verticesY[verticesJ[i]], verticesZ[verticesJ[i]]]
        triangle_C = [verticesX[verticesK[i]], verticesY[verticesK[i]], verticesZ[verticesK[i]]]
        triangles.append([triangle_A, triangle_B, triangle_C])

    return triangles

def find_centroids(verticesI, triangles):

    centroids = []

    for i in range(len(verticesI)):
        cen_X = (triangles[i][0][0] + triangles[i][1][0] + triangles[i][2][0]) / 3
        cen_Y = (triangles[i][0][1] + triangles[i][1][1] + triangles[i][2][1]) / 3
        cen_Z = (triangles[i][0][2] + triangles[i][1][2] + triangles[i][2][2]) / 3
        centroids.append([cen_X, cen_Y, cen_Z])

    return centroids

def plot_centroid(triangles, centroids):

    x = [triangles[50][0][0], triangles[50][1][0], triangles[50][2][0]]
    y = [triangles[50][0][1], triangles[50][1][1], triangles[50][2][1]]
    z = [triangles[50][0][2], triangles[50][1][2], triangles[50][2][2]]

    logging.debug(f"x: {x}")
    logging.debug(f"y: {y}")
    logging.debug(f"z: {z}")

    i = np.array([0])
    j = np.array([1])
    k = np.array([2])

    fig = go.Figure()
    fig.add_trace(go.Mesh3d(x=x, y=y, z=z, alphahull=5, opacity=0.4, color='cyan', i=i, j=j, k=k))
    fig.add_trace(
    go.Scatter3d(x=[centroids[50][0]],
                 y=[centroids[50][1]],
                 z=[centroids[50][2]],
                 mode='markers'))
    fig.show()

def plot_sphere_centre(sphere_centre, radius):

    fig = go.Figure()
    
    resolution=101
    x = sphere_centre[0]
    y = sphere_centre[1]
    z = sphere_centre[2]

    u, v = np.mgrid[0:2*np.pi:resolution*2j, 0:np.pi:resolution*1j]
    X = radius * np.cos(u)*np.sin(v) + x
    Y = radius * np.sin(u)*np.sin(v) + y
    Z = radius * np.cos(v) + z

    data_plotly = go.Surface(x=X, y=Y, z=Z, opacity=0.5)

    fig = go.Figure(data=data_plotly)

    fig.add_trace(
        go.Scatter3d(x = [x],
                     y = [y],
                     z = [z],
                    mode='markers'))

    fig.show()

def plot_bbox(bbox, objects, node_number, depth):

    min_x = bbox.mins[0]
    min_Y = bbox.mins[1]
    min_Z = bbox.mins[2]

    max_x = bbox.maxs[0]
    max_Y = bbox.maxs[1]
    max_Z = bbox.maxs[2]

    vertices_x = []
    vertices_y = []
    vertices_z = []

    for triangle in objects:
                #Triangle vertex 1
                vertices_x.append(triangle.vertices[0][0])
                vertices_y.append(triangle.vertices[0][1])
                vertices_z.append(triangle.vertices[0][2])
                #Triangle vertex 2
                vertices_x.append(triangle.vertices[1][0])
                vertices_y.append(triangle.vertices[1][1])
                vertices_z.append(triangle.vertices[1][2])
                #Triangle vertex 3
                vertices_x.append(triangle.vertices[2][0])
                vertices_y.append(triangle.vertices[2][1])
                vertices_z.append(triangle.vertices[2][2])

    fig1 = go.Figure(data=[
    go.Mesh3d(

        x=vertices_x,
        y=vertices_y,
        z=vertices_z,
    
        i = np.arange(0, len(vertices_x), 3),
        j = np.arange(1, len(vertices_x), 3),
        k = np.arange(2, len(vertices_x), 3),
    
        opacity=1,
        color='blue'

        )

    ])

    fig1.add_trace(go.Mesh3d(
        x = [min_x, min_x, max_x, max_x, min_x, min_x, max_x, max_x],
        y = [min_Y, max_Y, max_Y, min_Y, min_Y, max_Y, max_Y, min_Y],
        z = [min_Z, min_Z, min_Z, min_Z, max_Z, max_Z, max_Z, max_Z],

        i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
        j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
        k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
        color='cyan',
        opacity=0.50,
        flatshading = True))

    fig1.show()


def save_obj(node_list, number):

    node_list_depth = defaultdict(list)

    for node in node_list:
        node_list_depth[node.get_depth()].append(node.get_bbox())

    val = 0

    for depth in node_list_depth.values():

        i_list = [8, 1, 1, 1, 5, 5, 7, 7, 5, 1, 4, 3]
        j_list = [4, 5, 2, 3, 6, 7, 6, 3, 1, 2, 7, 4]
        k_list = [1, 8, 3, 4, 7, 8, 2, 2, 6, 6, 8, 7]

        for bbox in depth:

            min_x = bbox.mins[0]
            min_Y = bbox.mins[1]
            min_Z = bbox.mins[2]

            max_x = bbox.maxs[0]
            max_Y = bbox.maxs[1]
            max_Z = bbox.maxs[2]

            x = [min_x, min_x, max_x, max_x, min_x, min_x, max_x, max_x]
            y = [min_Y, max_Y, max_Y, min_Y, min_Y, max_Y, max_Y, min_Y]
            z = [min_Z, min_Z, min_Z, min_Z, max_Z, max_Z, max_Z, max_Z]

            i_list_layer = [i + (8 * val) for i in i_list]
            j_list_layer = [i + (8 * val) for i in j_list]
            k_list_layer = [i + (8 * val) for i in k_list]

            with open(f"all_{number}.obj", "w") as f:
                for i in range(len(x)):
                    f.write("v {} {} {}\n".format(x[i], y[i], z[i]))
                for i in range(12):
                    f.write("f {} {} {}\n".format(i_list_layer[i], j_list_layer[i], k_list_layer[i]))
                val += 1
    
    return f"all_{number}.obj"

def plot_BVH_from_obj(filename):

    verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK = open_obj_file(filename)
    plot_obj_file(verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK)

def plot_layer_sphere(dict_depth):

    val = 0
    for depth in dict_depth.values():

        fig = go.Figure()

        for bbox in depth:
            
            resolution=101
            x = bbox.centre[0]
            y = bbox.centre[1]
            z = bbox.centre[2]

            u, v = np.mgrid[0:2*np.pi:resolution*2j, 0:np.pi:resolution*1j]
            X = bbox.radius * np.cos(u)*np.sin(v) + x
            Y = bbox.radius * np.sin(u)*np.sin(v) + y
            Z = bbox.radius * np.cos(v) + z

            fig.add_trace(go.Surface(x=X, y=Y, z=Z, opacity=1))

        fig.write_image(r"D:\OneDrive\Pulpit\Praca_Magisterska\layers\cow\sphere\layer_{}.png".format(val), format='png')
        val += 1

def calculate_box_OBB(obj_list_copy):
    
    points_3D = []

    for triangle in obj_list_copy:
        points_3D.append(tuple([triangle.vertices[0][0], triangle.vertices[0][1], triangle.vertices[0][2]]))
        points_3D.append(tuple([triangle.vertices[1][0], triangle.vertices[1][1], triangle.vertices[1][2]]))
        points_3D.append(tuple([triangle.vertices[2][0], triangle.vertices[2][1], triangle.vertices[2][2]]))

    ca = np.cov(points_3D, y=None, rowvar=0, bias=1)
    logging.debug(f"ca: {ca}")

    v, vect = np.linalg.eig(ca)
    logging.debug(f"v: {v}")
    logging.debug(f"vect: {vect}")
    tvect = np.transpose(vect)
    logging.debug(f"tvect: {tvect}")

    # use the inverse of the eigenvectors as a rotation matrix and
    # rotate the points so they align with the x and y axes
    ar = np.dot(points_3D, np.linalg.inv(tvect))

    # get the minimum and maximum x and y
    mina = np.min(ar, axis=0)
    maxa = np.max(ar, axis=0)

    logging.debug(f"mina: {mina}")
    logging.debug(f"maxa: {maxa}")
    diff = (maxa - mina) * 0.5
    logging.debug(f"diff: {diff}")

    # the center is just half way between the min and max xy
    center = mina + diff
    logging.debug(f"center: {center}")

    corners = np.array([center + [-diff[0], -diff[1], -diff[2]],
                        center + [diff[0], diff[1], diff[2]],
                        center + [diff[0], -diff[1], diff[2]],
                        center + [diff[0], -diff[1], -diff[2]],
                        center + [diff[0], diff[1], -diff[2]],
                        center + [-diff[0], diff[1], diff[2]],
                        center + [-diff[0], -diff[1], diff[2]],
                        center + [-diff[0], diff[1], -diff[2]]])

    # use the eigenvectors as a rotation matrix and
    # rotate the corners and the center back
    corners = np.dot(corners, tvect)
    logging.debug(f"corner 1 x:{corners[0][0]}, y:{corners[0][1]}, z:{corners[0][2]}")
    logging.debug(f"corner 2 x:{corners[1][0]}, y:{corners[1][1]}, z:{corners[1][2]}")
    logging.debug(f"corners: {corners}")
    center = np.dot(center, tvect)

    world_box = OBB(corners, center, diff, vect)

    return world_box

def plot_BVH_from_obj_with_ray(node_list, ray_origin, ray_dest):

    node_list_depth = defaultdict(list)

    for node in node_list:
        node_list_depth[node.get_depth()].append(node.get_bbox())

    val = 0
    for depth in node_list_depth.values():

        fig = go.Figure()

        for bbox in depth:

            fig.add_trace(go.Mesh3d(
                
                x = bbox.corners[:, 0],
                y = bbox.corners[:, 1],
                z = bbox.corners[:, 2],

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

        val += 1

    fig.add_trace(
            go.Scatter3d(x=[ray_origin[0], ray_dest[0]],
                        y=[ray_origin[1], ray_dest[1]],
                        z=[ray_origin[2], ray_dest[2]],
                        mode='lines'))
    
    fig.show()

def plot_layer_OBB(node_list):

    node_list_depth = defaultdict(list)

    for node in node_list:
        node_list_depth[node.get_depth()].append(node.get_bbox())

    val = 0
    for depth in node_list_depth.values():

        fig = go.Figure()

        for bbox in depth:

            fig.add_trace(go.Mesh3d(
                
                x = bbox.corners[:, 0],
                y = bbox.corners[:, 1],
                z = bbox.corners[:, 2],

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
                go.Scatter3d(x=bbox.corners[:, 0],
                            y=bbox.corners[:, 1],
                            z=bbox.corners[:, 2],
                            mode='markers'))

        val += 1

def plot_collisions(collisions, fig, bbox_type_A, bbox_type_B):
    
    collisions_len = len(collisions)

    for collision in collisions:

        x = [collision[0].vertices[0][0], collision[0].vertices[1][0], collision[0].vertices[2][0]]
        y = [collision[0].vertices[0][1], collision[0].vertices[1][1], collision[0].vertices[2][1]]
        z = [collision[0].vertices[0][2], collision[0].vertices[1][2], collision[0].vertices[2][2]]

        logging.debug(f"---------------------------------")

        logging.debug(f"x: {x}")
        logging.debug(f"y: {y}")
        logging.debug(f"z: {z}")

        i = np.array([0])
        j = np.array([1])
        k = np.array([2])

        x1 = [collision[1].vertices[0][0], collision[1].vertices[1][0], collision[1].vertices[2][0]]
        y1 = [collision[1].vertices[0][1], collision[1].vertices[1][1], collision[1].vertices[2][1]]
        z1 = [collision[1].vertices[0][2], collision[1].vertices[1][2], collision[1].vertices[2][2]]

        logging.debug(f"x: {x1}")
        logging.debug(f"y: {y1}")
        logging.debug(f"z: {z1}")


        i1 = np.array([0])
        j1 = np.array([1])
        k1 = np.array([2])

        fig.add_trace(go.Mesh3d(x=x, y=y, z=z, alphahull=5, opacity=0.4, color='red', i=i, j=j, k=k))
        fig.add_trace(go.Mesh3d(x=x1, y=y1, z=z1, alphahull=5, opacity=0.4, color='red', i=i1, j=j1, k=k1))
        fig.update_layout(title_text=f"Collision {bbox_type_A} against {bbox_type_B}, number of collisions: {collisions_len}")
    
    return fig

def plot_2OBB(obb_a, obb_b, result, aabb):

    fig = go.Figure()

    box_a = obb_a.get_bbox()
    box_b = obb_b.get_bbox()

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
    
    fig.add_trace(go.Mesh3d(
                
        x = box_b.corners[:, 0],
        y = box_b.corners[:, 1],
        z = box_b.corners[:, 2],

        i = [0, 5, 1, 2, 2, 3, 6, 1, 7, 4, 5, 5],
        j = [6, 0, 4, 4, 3, 0, 5, 2, 0, 7, 7, 1],
        k = [5, 7, 2, 3, 6, 6, 2, 5, 3, 3, 4, 4],
        color='red',
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

    fig.update_layout(title_text=f"OBB test result: {result}, OBB_A: {obb_a}, OBB_B: {obb_b}")
    fig.show()

def plot_OBB_triangles(bbox):

    box = bbox.get_bbox()

    fig = go.Figure()

    fig.add_trace(go.Mesh3d(

    x = box.corners[:, 0],
    y = box.corners[:, 1],
    z = box.corners[:, 2],

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

    x = []
    y = []
    z = []    

    i = []
    j = []
    k = []

    num = 0
    for triangle in bbox.get_triangles():

        x.append(triangle.vertices[0][0])
        x.append(triangle.vertices[1][0])
        x.append(triangle.vertices[2][0])

        y.append(triangle.vertices[0][1])
        y.append(triangle.vertices[1][1])
        y.append(triangle.vertices[2][1])

        z.append(triangle.vertices[0][2])
        z.append(triangle.vertices[1][2])
        z.append(triangle.vertices[2][2])

        logging.debug(f"---------------------------------")

        logging.debug(f"x: {x}")
        logging.debug(f"y: {y}")
        logging.debug(f"z: {z}")

    
        i.append([3 * num])
        j.append([(3 * num) + 1])
        k.append([(3 * num) + 2])
        num += 1 

    fig.add_trace(go.Mesh3d(x=x, y=y, z=z, alphahull=5, opacity=0.4, color='red', i=i, j=j, k=k))

    fig.show()
    time.sleep(30)

def plot_all_AABB(filename):

    plot_BVH_from_obj(filename)

def move_obj(x_axis, y_axis, z_axis, verticesX_rotated, verticesY_rotated, verticesZ_rotated):

    verticesX_rotated_moved = []
    verticesY_rotated_moved = []
    verticesZ_rotated_moved = []

    for vert_x in verticesX_rotated:
        vert_x += x_axis
        verticesX_rotated_moved.append(vert_x)
    
    for vert_y in verticesY_rotated:
        vert_y += y_axis
        verticesY_rotated_moved.append(vert_y)
    
    for vert_z in verticesZ_rotated:
        vert_z += z_axis
        verticesZ_rotated_moved.append(vert_z)

    return verticesX_rotated_moved, verticesY_rotated_moved, verticesZ_rotated_moved

def rotate_move_obj_files(filename_A, x_axis_A, y_axis_A, z_axis_A, rot_x_axis_A, rot_y_axis_A, rot_z_axis_A,
                          filename_B, x_axis_B, y_axis_B, z_axis_B, rot_x_axis_B, rot_y_axis_B, rot_z_axis_B):
    
    verticesX_rotated_A, verticesY_rotated_A, verticesZ_rotated_A, verticesI_A, verticesJ_A, verticesK_A = rotate_obj(filename_A, rot_x_axis_A, rot_y_axis_A, rot_z_axis_A)
    verticesX_rotated_moved_A, verticesY_rotated_moved_A, verticesZ_rotated_moved_A = move_obj(x_axis_A, y_axis_A, z_axis_A, verticesX_rotated_A, verticesY_rotated_A, verticesZ_rotated_A)    

    verticesX_rotated_B, verticesY_rotated_B, verticesZ_rotated_B, verticesI_B, verticesJ_B, verticesK_B = rotate_obj(filename_B, rot_x_axis_B, rot_y_axis_B, rot_z_axis_B)
    verticesX_rotated_moved_B, verticesY_rotated_moved_B, verticesZ_rotated_moved_B = move_obj(x_axis_B, y_axis_B, z_axis_B, verticesX_rotated_B, verticesY_rotated_B, verticesZ_rotated_B)

    plot_2obj_file(verticesX_rotated_moved_A, verticesY_rotated_moved_A, verticesZ_rotated_moved_A, verticesI_A, verticesJ_A, verticesK_A, verticesX_rotated_moved_B, verticesY_rotated_moved_B, verticesZ_rotated_moved_B, verticesI_B, verticesJ_B, verticesK_B)
    
def rotate_obj(filename, rot_x_axis, rot_y_axis, rot_z_axis):

    logging.debug("rotate")

    """Open .obj file"""

    script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
    rel_path = f"obj/examples/{filename}.obj"
    abs_file_path = os.path.join(script_dir, rel_path)

    verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK = open_obj_file(abs_file_path)

    centroid = [sum(verticesX)/len(verticesX), sum(verticesY)/len(verticesY), sum(verticesZ)/len(verticesZ)]
    logging.debug(f"centroid: {centroid}")

    new_verticesX = []
    new_verticesY = []
    new_verticesZ = []

    for vert_x in verticesX:
        vert_x += -centroid[0]
        new_verticesX.append(vert_x)
    
    for vert_y in verticesY:
        vert_y += -centroid[1]
        new_verticesY.append(vert_y)
    
    for vert_z in verticesZ:
        vert_z += -centroid[2]
        new_verticesZ.append(vert_z)

    new_centroid = [sum(new_verticesX)/len(new_verticesX), sum(new_verticesY)/len(new_verticesY), sum(new_verticesZ)/len(new_verticesZ)]
    logging.debug(f"centroid: {new_centroid}")

    rot_x_axis_rad = math.radians(rot_x_axis)
    rot_y_axis_rad = math.radians(rot_y_axis)
    rot_z_axis_rad = math.radians(rot_z_axis)

    Rx = np.array([[1, 0, 0],
                    [0, math.cos(rot_x_axis_rad), -math.sin(rot_x_axis_rad)],
                    [0, math.sin(rot_x_axis_rad), math.cos(rot_x_axis_rad)]])

    Ry = np.array([[math.cos(rot_y_axis_rad), 0, math.sin(rot_y_axis_rad)],
                    [0, 1, 0],
                    [-math.sin(rot_y_axis_rad), 0, math.cos(rot_y_axis_rad)]])
    
    Rz = np.array([[math.cos(rot_z_axis_rad), -math.sin(rot_z_axis_rad), 0],
                    [math.sin(rot_z_axis_rad), math.cos(rot_z_axis_rad), 0],
                    [0, 0, 1]])

    R = Rz @ Ry @ Rx

    points = np.vstack([new_verticesX, new_verticesY, new_verticesZ])
    logging.debug(f"points: {points}")

    rotated_points = R @ points
    logging.debug(f"rotated_points: {rotated_points}")

    verticesX_rotated = rotated_points.tolist()[0]
    verticesY_rotated = rotated_points.tolist()[1]
    verticesZ_rotated = rotated_points.tolist()[2]

    return verticesX_rotated, verticesY_rotated, verticesZ_rotated, verticesI, verticesJ, verticesK
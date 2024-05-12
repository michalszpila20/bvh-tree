# importing mplot3d toolkits, numpy and matplotlib
import plotly.graph_objects as go
import numpy as np
from AABB import AABB
from OBB import OBB
import plotly.express as px
import plotly
import math
from sphere import sphere 
import time

def open_obj_file():

    # vertices coordiantes
    verticesX = []
    verticesY = []
    verticesZ = []

    # normal vectors coordiantes 
    verticesNX = []
    verticesNY = []
    verticesNZ = []

    # faces (triangles) 
    verticesI = []
    verticesJ = []
    verticesK = []

    # options: bear / cow / teapot / pumpkin
    file = open(r"D:\OneDrive\Pulpit\Praca_Magisterska\cow.obj.txt")

    Lines = file.readlines()

    for line in Lines:
        if line.__contains__('v'):
            verticesX.append(float(line.split(" ")[1]))
            verticesY.append(float(line.split(" ")[2]))
            verticesZ.append(float(line.strip().split(" ")[3]))
        elif line.__contains__('f'):
            verticesI.append(int(line.split(" ")[1]) -1)
            verticesJ.append(int(line.split(" ")[2]) -1)
            verticesK.append(int(line.strip().split(" ")[3]) -1)   
        elif line.__contains__('vn'):
            verticesNX.append(int(line.split(" ")[1]) -1)
            verticesNY.append(int(line.split(" ")[2]) -1)
            verticesNZ.append(int(line.strip().split(" ")[3]) -1)
 
    return verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK

def plot_obj_file(verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK):
    fig2 = go.Figure(data=[
    go.Mesh3d(
        x=verticesX,
        y=verticesY,
        z=verticesZ,
            
        color='blue',
        opacity=0.4,
            
        i=verticesI[0:-1],
        j=verticesJ[0:-1],
        k=verticesK[0:-1]
        )
    ])

    fig2.add_trace(
    go.Scatter3d(x=[0.21700000000000008],
                 y=[1.575],
                 z=[0.0],
                 mode='markers'))

    fig2.show()

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


def calculate_box_sphere(obj_list_copy):

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

    sphere_centre = [(min(vertices_x) + max(vertices_x)) / 2,
                     (min(vertices_y) + max(vertices_y)) / 2,
                     (min(vertices_z) + max(vertices_z)) / 2]

    print(f"sphere_centre: {sphere_centre}")

    radius = math.dist(sphere_centre, [max(vertices_x), max(vertices_y), max(vertices_z)])

    print(f"radius: {radius}")


    #plot_sphere_centre(sphere_centre, radius)

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

    plot_centroid(triangles, centroids)
    return centroids

def plot_centroid(triangles, centroids):
    x = [triangles[50][0][0], triangles[50][1][0], triangles[50][2][0]]
    y = [triangles[50][0][1], triangles[50][1][1], triangles[50][2][1]]
    z = [triangles[50][0][2], triangles[50][1][2], triangles[50][2][2]]

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

    fig1.write_html(r"D:\OneDrive\Pulpit\Praca_Magisterska\images\node{}_{}".format(node_number, depth))
    ##fig1.write_image(r"D:\OneDrive\Pulpit\Praca_Magisterska\images\node{}_{}".format(node_number, depth), format='png')


def plot_layer(dict_depth):

    val = 0
    for depth in dict_depth.values():

        fig = go.Figure()

        for bbox in depth:

            min_x = bbox.mins[0]
            min_Y = bbox.mins[1]
            min_Z = bbox.mins[2]

            max_x = bbox.maxs[0]
            max_Y = bbox.maxs[1]
            max_Z = bbox.maxs[2]

            fig.add_trace(go.Mesh3d(
                
                x = [min_x, min_x, max_x, max_x, min_x, min_x, max_x, max_x],
                y = [min_Y, max_Y, max_Y, min_Y, min_Y, max_Y, max_Y, min_Y],
                z = [min_Z, min_Z, min_Z, min_Z, max_Z, max_Z, max_Z, max_Z],

                i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
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

        fig.write_html(r"D:\OneDrive\Pulpit\Praca_Magisterska\layers\layer_{}".format(val))
        val += 1

def plot_BVH(node_list):
    
    fig = go.Figure()

    for node in node_list:
        
        opacity_val = 0.05

        if opacity_val < 0.9:
            opacity_val += 0.05

        bbox = node.get_bbox()

        min_x = bbox.mins[0]
        min_Y = bbox.mins[1]
        min_Z = bbox.mins[2]

        max_x = bbox.maxs[0]
        max_Y = bbox.maxs[1]
        max_Z = bbox.maxs[2]
        fig.add_trace(go.Mesh3d(
            
            x = [min_x, min_x, max_x, max_x, min_x, min_x, max_x, max_x],
            y = [min_Y, max_Y, max_Y, min_Y, min_Y, max_Y, max_Y, min_Y],
            z = [min_Z, min_Z, min_Z, min_Z, max_Z, max_Z, max_Z, max_Z],
            i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
            j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
            k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
            color='gray',
            opacity=opacity_val,
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

    fig.write_html(r"D:\OneDrive\Pulpit\Praca_Magisterska\layers\all")


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

        fig.write_html(r"D:\OneDrive\Pulpit\Praca_Magisterska\layers\layer_{}".format(val))
        val += 1

def calculate_box_OBB(obj_list_copy):
    
    points_3D = []

    for triangle in obj_list_copy:
        #Triangle vertex 1
        points_3D.append(tuple([triangle.vertices[0][0], triangle.vertices[0][1], triangle.vertices[0][2]]))
        #Triangle vertex 2
        points_3D.append(tuple([triangle.vertices[0][0], triangle.vertices[0][1], triangle.vertices[0][2]]))
        #Triangle vertex 3
        points_3D.append(tuple([triangle.vertices[0][0], triangle.vertices[0][1], triangle.vertices[0][2]]))

    ca = np.cov(points_3D, y=None, rowvar=0, bias=1)
    print(f"ca: {ca}")

    v, vect = np.linalg.eig(ca)
    print(f"v: {v}")
    print(f"vect: {vect}")
    tvect = np.transpose(vect)
    print(f"tvect: {tvect}")

    # use the inverse of the eigenvectors as a rotation matrix and
    # rotate the points so they align with the x and y axes
    ar = np.dot(points_3D, np.linalg.inv(tvect))

    # get the minimum and maximum x and y
    mina = np.min(ar, axis=0)
    maxa = np.max(ar, axis=0)

    print(f"mina: {mina}")
    print(f"maxa: {maxa}")
    diff = (maxa - mina) * 0.5
    print(f"diff: {diff}")

    # the center is just half way between the min and max xy
    center = mina + diff
    print(f"center: {center}")

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
    print(f"corner 1 x:{corners[0][0]}, y:{corners[0][1]}, z:{corners[0][2]}")
    print(f"corner 2 x:{corners[1][0]}, y:{corners[1][1]}, z:{corners[1][2]}")
    print(f"corners: {corners}")
    center = np.dot(center, tvect)

    world_box = OBB(corners, center)

    return world_box

def plot_layer_OBB(dict_depth):

    val = 0
    for depth in dict_depth.values():

        fig = go.Figure()

        for bbox in depth:

            fig.add_trace(go.Mesh3d(
                
                x = bbox.corners[:, 0],
                y = bbox.corners[:, 1],
                z = bbox.corners[:, 2],

                # i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
                # j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
                # k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],

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

        fig.write_html(r"D:\OneDrive\Pulpit\Praca_Magisterska\layers\layer_{}".format(val))
        val += 1

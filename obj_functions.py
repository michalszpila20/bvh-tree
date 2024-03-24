# importing mplot3d toolkits, numpy and matplotlib
import plotly.graph_objects as go
import numpy as np
import time
from AABB import AABB
import plotly.express as px
import plotly

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
    file = open(r"D:\OneDrive\Pulpit\Praca_Magisterska\pumpkin.obj.txt")

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

def plot_obj_file(verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK, min_s = [0,0,0], max_s = [0, 0, 0]):
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

    fig2.add_trace(go.Mesh3d(
        x=[0.608, 0.608, 0.998, 0.998, 0.608, 0.608, 0.998, 0.998],
        y=[0.091, 0.963, 0.963, 0.091, 0.091, 0.963, 0.963, 0.091],
        z=[0.140, 0.140, 0.140, 0.140, 0.571, 0.571, 0.571, 0.571],

        i = [7, 0, 0, 0, 4, 4, 6, 6, 4, 0, 3, 2],
        j = [3, 4, 1, 2, 5, 6, 5, 2, 0, 1, 6, 3],
        k = [0, 7, 2, 3, 6, 7, 1, 1, 5, 5, 7, 6],
        color='cyan',
        opacity=0.50,
        flatshading = True))

    fig2.show()

def calculate_box(obj_list_copy):

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

    ##fig1.write_html(r"D:\OneDrive\Pulpit\Praca_Magisterska\images\node{}_{}".format(node_number, depth))
    fig1.write_image(r"D:\OneDrive\Pulpit\Praca_Magisterska\images\node{}_{}".format(node_number, depth))


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

            print(f"layer: {val}")

        fig.write_html(r"D:\OneDrive\Pulpit\Praca_Magisterska\layers\layer_{}".format(val))
        ##fig.write_image(r"D:\OneDrive\Pulpit\Praca_Magisterska\layers\layer_{}.png".format(val))
        val += 1

    return 0
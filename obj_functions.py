# importing mplot3d toolkits, numpy and matplotlib
import plotly.graph_objects as go
import numpy as np

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
    file = open(r"")

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
        opacity=1,
            
        i=verticesI[0:-1],
        j=verticesJ[0:-1],
        k=verticesK[0:-1]
        )
    ])

    fig2.add_trace(
    go.Scatter3d(x=[min_s[0], max_s[0]],
                 y=[min_s[1], max_s[1]],
                 z=[min_s[2], max_s[2]],
                 mode='markers')
)

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

    print(f"The mins is: {min_s}")
    print(f"The maxs is: {max_s}")

    return min_s, max_s

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
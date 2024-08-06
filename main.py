from obj_functions import plot_two_obj_file, plot_collisions, plot_obj_file, plot_BVH_from_obj_with_ray, save_obj, plot_layer_OBB, open_obj_file, calculate_box_AABB, build_triangles, find_centroids, calculate_box_sphere, calculate_box_OBB, calculate_box_sphere_ritter
from ray_intersection.ray_intersection_utils import plot_OBB_ray
from bvh import BVHNode
from triangle import Triangle
import statistics
import sys
from ray_intersection.ray_intersection import ray_intersect
from ray_intersection.ray_intersection_AABB import plot_BVH_aabb_from_obj_with_ray
import logging
from collision_detection.collision_detection import BVH_collision_detection
from collision_detection.collision_detection_AABB import plot_triangles_in_aabb
import plotly.graph_objects as go
import numpy as np

#vertices coordiantes
verticesX = []
verticesY = []
verticesZ = []

#normal vectors coordiantes 
verticesNX = []
verticesNY = []
verticesNZ = []

#faces (triangles) 
verticesI = []
verticesJ = []
verticesK = []

triangles = []
centroids = []

def begin(filename):
    """Open .obj file, build triangles and centroids"""

    verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK = open_obj_file(f"{filename}.obj.txt")
    triangles = build_triangles(verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK)
    centroids = find_centroids(verticesI, triangles)

    obj_list = []

    for i in range(len(verticesI)):
        obj_list.append(Triangle([[triangles[i][0][0], triangles[i][0][1], triangles[i][0][2]],
                                [triangles[i][1][0], triangles[i][1][1], triangles[i][1][2]],
                                [triangles[i][2][0], triangles[i][2][1], triangles[i][2][2]]],
                                [centroids[i][0], centroids[i][1], centroids[i][2]]))
        
    return obj_list

def build(node_list, bbox_type, filename):
    """First iteration of the recursion, creation of the root node"""
    
    logging.debug("build function")

    obj_list_copy = begin(filename)
    
    logging.debug(f"obj_list_copy: {obj_list_copy}")

    root = BVHNode(obj_list_copy)

    world_box = None

    if bbox_type == "aabb":
        world_box = calculate_box_AABB(obj_list_copy)
    elif bbox_type == "obb":  
        world_box = calculate_box_OBB(obj_list_copy)
    elif bbox_type == "sphere":
        world_box = calculate_box_sphere(obj_list_copy)

    root.set_bbox(world_box)
    node_list.append(root)

    build_recursive(root, 0, node_list, bbox_type)
    return node_list

def build_recursive(node, depth, node_list, bbox_type):
    """Next iterations of the recursion, creation of the root node children"""

    logging.debug("--------------------------------------------------------------------------------------")
    logging.debug(f"build recursive: {depth}")

    vertices_x = []
    vertices_y = []
    vertices_z = []
    obj_list_node = node.triangles
    median_split = False
    node.set_depth(depth)

    left_index = 0
    right_index = len(obj_list_node)

    logging.debug(f"left_index: {left_index}")
    logging.debug(f"right_index: {right_index}")
    logging.debug(f"right_index - left_index: {right_index - left_index}")

    if right_index - left_index <= 16:
        node.make_leaf()
        logging.debug("Make leaf!")
        return node_list
    else:
        # Split intersectables into left and right by finding a split_index
        
        for triangle in obj_list_node:
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

        #Searching for the largest axis
        axis_x_len = abs(max(vertices_x) - min(vertices_x))
        axis_y_len = abs(max(vertices_y) - min(vertices_y))
        axis_z_len = abs(max(vertices_z) - min(vertices_z))

        logging.debug(f"axis_x_len: {axis_x_len}")
        logging.debug(f"axis_y_len: {axis_y_len}")
        logging.debug(f"axis_z_len: {axis_z_len}")

        axis_len = {
            "x": axis_x_len,
            "y": axis_y_len,
            "z": axis_z_len
        }
        
        max_len_axis = max(axis_len, key=axis_len.get)
        logging.debug(f"The largest axis is: {max_len_axis}")

        split_index_val = axis_len[max_len_axis] / 2
        logging.debug(f"split_index: {split_index_val}")

        # Make sure that neither left nor right is completely empty
        obj_centroid_x = []
        obj_centroid_y = []
        obj_centroid_z = []

        for triangle in obj_list_node:
            obj_centroid_x.append(triangle.centroid[0])
            obj_centroid_y.append(triangle.centroid[1])
            obj_centroid_z.append(triangle.centroid[2])

        centres = []
        if max_len_axis == "x":
            centres = obj_centroid_x
            logging.debug(f"Max axis: {max_len_axis}")
        elif max_len_axis == "y":
            centres = obj_centroid_y
            logging.debug(f"Max axis: {max_len_axis}")
        elif max_len_axis == "z":
            centres = obj_centroid_z
            logging.debug(f"Max axis: {max_len_axis}")

        left_index_pop = 0
        right_index_pop = 0
        split_index_min = min(centres) + split_index_val

        logging.debug(f"split_index_min: {split_index_min}")

        for centre in centres:
            if centre < split_index_min:
                left_index_pop += 1
            else:
                right_index_pop += 1

        logging.debug(f"left_index_pop: {left_index_pop}")
        logging.debug(f"right_index_pop: {right_index_pop}")

        axis_median = 0
        if left_index_pop <= 1 or right_index_pop <= 1:
            median_split = True
            axis_median = statistics.median(centres)
            logging.debug(f"axis_median: {axis_median}")

        logging.debug(f"Median split? {median_split}")
        
        # Calculate bounding boxes of left and right sides
        obj_list_node_sorted = []   
        for triangle in obj_list_node:
            if max_len_axis == "x":
                obj_list_node_sorted = sorted(obj_list_node, key=lambda triangle: triangle.centroid[0])
            elif max_len_axis == "y":
                obj_list_node_sorted = sorted(obj_list_node, key=lambda triangle: triangle.centroid[1])
            elif max_len_axis == "z":
                obj_list_node_sorted = sorted(obj_list_node, key=lambda triangle: triangle.centroid[2])

        if median_split == True:
            split_index_min = axis_median
            logging.debug(f"split index val: {split_index_min}")

        split_index = left_index
        logging.debug(f"split index before: {split_index}")
        
        for triangle in obj_list_node_sorted:
            if max_len_axis == "x":
                if triangle.centroid[0] < split_index_min:
                    split_index += 1 
            elif max_len_axis == "y":
                if triangle.centroid[1] < split_index_min:
                    split_index += 1 
            elif max_len_axis == "z":
                if triangle.centroid[2] < split_index_min:
                    split_index += 1 

        #one more index
        split_index += 1
        logging.debug(f"Split index after: {split_index}")

        logging.debug(f"Len obj_list_node_sorted: {len(obj_list_node_sorted)}")
        if right_index - left_index < 20:
            logging.debug(f"obj_list_copy[left_index:split_index]: {obj_list_node_sorted[left_index:split_index]}")
            logging.debug(f"obj_list_copy[split_index:right_index]: {obj_list_node_sorted[split_index:right_index]}")
        
        logging.debug(f"len(obj_list_copy[left_index:split_index]) : {len(obj_list_node_sorted[left_index:split_index])}")
        logging.debug(f"len(obj_list_copy[split_index:right_index]) : {len(obj_list_node_sorted[split_index:right_index])}")
        if bbox_type == "aabb":
            left_index_box = calculate_box_AABB(obj_list_node_sorted[left_index:split_index])
            right_index_box = calculate_box_AABB(obj_list_node_sorted[split_index:right_index])
        elif bbox_type == "sphere":
            left_index_box = calculate_box_sphere(obj_list_node_sorted[left_index:split_index])
            right_index_box = calculate_box_sphere(obj_list_node_sorted[split_index:right_index])
        elif bbox_type == "obb":
            left_index_box = calculate_box_OBB(obj_list_node_sorted[left_index:split_index])
            right_index_box = calculate_box_OBB(obj_list_node_sorted[split_index:right_index])
        
        # Initiate current node as an interior node with leftNode and rightNode as children
        node.left = BVHNode(obj_list_node_sorted[left_index:split_index])
        node.right = BVHNode(obj_list_node_sorted[split_index:right_index])

        node.left.bbox = left_index_box
        node.right.bbox = right_index_box

        node_list.append(node.left)
        node_list.append(node.right)
        build_recursive(node.left, depth + 1, node_list, bbox_type)
        build_recursive(node.right, depth + 1, node_list, bbox_type)
        

def main():

    bbox_type = input("Box type: aabb / sphere / obb: ")
    filename_A = input("Choose obj file: bear / boat / cow / pumpkin / rabbit / teapot: ")
    test_type = input("Ray intersect [a] or collision detection [b]?: ")    
    filename_B = None

    node_list_A = []
    node_list_B = []

    node_list_A = build(node_list_A, bbox_type, filename_A)

    logging.debug(f"node_list_A : {node_list_A}")

    ray_origin = [1.223, -2.78, 10]
    ray_dest = [1.222, -5, 4]
    
    # plot_BVH_aabb_from_obj_with_ray(f"{filename_A}.obj.txt", ray_origin, ray_dest)

    if test_type == 'b':
        filename_B = input("Choose the second obj file: bear / boat / cow / pumpkin / rabbit / teapot: ")
        node_list_B = build(node_list_B, bbox_type, filename_B)
    if test_type == 'a':
        if ray_intersect(ray_origin, ray_dest, node_list_A, bbox_type):
            logging.debug("The end with intersection")
        else:
            logging.debug("The end without intersection")
    elif test_type == 'b':
        collisions = BVH_collision_detection(node_list_A, node_list_B, bbox_type)
        fig = plot_two_obj_file(f"{filename_A}.obj.txt", f"{filename_B}.obj.txt")
        fig = plot_collisions(collisions, fig)
        fig.show()
        
if __name__ == "__main__":

    logging.basicConfig(filename='app.log', filemode='w', format='%(asctime)s - %(levelname)s - %(message)s', level=logging.DEBUG)

    main()
    



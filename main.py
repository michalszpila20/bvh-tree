from obj_functions import save_obj_sphere, scale_obj, save_obj_obb, plot_BVH_from_obj, rotate_move_obj_files, rotate_move_obj_file_ray, rotate_move_obj_file_ray_no_collision, rotate_move_obj_file, move_obj, rotate_obj, plot_two_obj_file, plot_OBB_triangles, plot_collisions, plot_obj_file, plot_BVH_from_obj_with_ray, save_obj_aabb, plot_layer_OBB, open_obj_file, calculate_box_AABB, build_triangles, find_centroids, calculate_box_sphere, calculate_box_OBB, calculate_box_sphere_ritter
from ray_intersection.ray_intersection_utils import plot_OBB_ray
from node import Node
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
import tracemalloc
import time
from dataclasses import dataclass
from typing import List
from dataclasses import asdict

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

@dataclass
class Position:
    x_axis: float
    y_axis: float
    z_axis: float
    rot_x_axis: float
    rot_y_axis: float
    rot_z_axis: float
    scale: float

@dataclass
class Config:
    filename: str
    bbox_type: str  
    termination_step: int

def begin(filename, position):
    """Open .obj file, build triangles and centroids"""

    x_axis, y_axis, z_axis, rot_x_axis, rot_y_axis, rot_z_axis, scale = asdict(position).values()

    scaled_vert_x, scaled_vert_y, scaled_vert_z, verticesI, verticesJ, verticesK = scale_obj(filename, scale)
    verticesX_rotated, verticesY_rotated, verticesZ_rotated = rotate_obj(scaled_vert_x, scaled_vert_y, scaled_vert_z, rot_x_axis, rot_y_axis, rot_z_axis)
    verticesX_rotated_moved_A, verticesY_rotated_moved_A, verticesZ_rotated_moved_A = move_obj(x_axis, y_axis, z_axis, verticesX_rotated,
                                                                                                verticesY_rotated, verticesZ_rotated)

    triangles = build_triangles(verticesX_rotated_moved_A, verticesY_rotated_moved_A, verticesZ_rotated_moved_A, verticesI, verticesJ, verticesK)
    centroids = find_centroids(verticesI, triangles)

    obj_list = []

    for i in range(len(verticesI)):
        obj_list.append(Triangle([[triangles[i][0][0], triangles[i][0][1], triangles[i][0][2]],
                                [triangles[i][1][0], triangles[i][1][1], triangles[i][1][2]],
                                [triangles[i][2][0], triangles[i][2][1], triangles[i][2][2]]],
                                [centroids[i][0], centroids[i][1], centroids[i][2]]))
        
    return obj_list

def build(node_list, config, position):
    """First iteration of the recursion, creation of the root node"""
    
    logging.debug("build function")

    filename, bbox_type, _ = asdict(config).values()

    obj_list_copy = begin(filename, position)
    
    logging.debug(f"obj_list_copy: {obj_list_copy}")

    root = Node(obj_list_copy)

    world_box = None

    if bbox_type == "aabb":
        world_box = calculate_box_AABB(obj_list_copy)
    elif bbox_type == "obb":  
        world_box = calculate_box_OBB(obj_list_copy)
    elif bbox_type == "sphere":
        world_box = calculate_box_sphere(obj_list_copy)

    root.set_bbox(world_box)
    node_list.append(root)

    build_recursive(root, node_list, config)
    return node_list

def build_recursive(node, node_list, config):
    """Next iterations of the recursion, creation of the root node children"""

    vertices_x = []
    vertices_y = []
    vertices_z = []
    obj_list_node = node.triangles
    median_split = False

    _, bbox_type, termination_step = asdict(config).values()

    left_index = 0
    right_index = len(obj_list_node)

    logging.debug(f"left_index: {left_index}")
    logging.debug(f"right_index: {right_index}")
    logging.debug(f"right_index - left_index: {right_index - left_index}")

    if right_index - left_index <= termination_step:
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
            # left_index_box = calculate_box_sphere_ritter(obj_list_node_sorted[left_index:split_index])
            # right_index_box = calculate_box_sphere_ritter(obj_list_node_sorted[split_index:right_index])
        elif bbox_type == "obb":
            left_index_box = calculate_box_OBB(obj_list_node_sorted[left_index:split_index])
            right_index_box = calculate_box_OBB(obj_list_node_sorted[split_index:right_index])
        
        # Initiate current node as an interior node with leftNode and rightNode as children
        node.left = Node(obj_list_node_sorted[left_index:split_index])
        node.right = Node(obj_list_node_sorted[split_index:right_index])

        node.left.bbox = left_index_box
        node.right.bbox = right_index_box

        node_list.append(node.left)
        node_list.append(node.right)
        build_recursive(node.left, node_list, config)
        build_recursive(node.right, node_list, config)
        
def main():

    node_list_A = []

    position = Position(0, 0, 0, 0, 0, 0, 1)
    config = Config("cow", "aabb", 4)

    node_list_A = build(node_list_A, config, position)

    if config.bbox_type == "aabb":
        all_aabb_file = save_obj_aabb(node_list_A, config.filename)
        plot_BVH_from_obj(all_aabb_file)
    elif config.bbox_type == "sphere":
        all_sphere_file = save_obj_sphere(node_list_A, 6, config.filename)
        plot_BVH_from_obj(all_sphere_file)
    elif config.bbox_type == "obb":
        all_obb_file = save_obj_obb(node_list_A, config.filename)
        plot_BVH_from_obj(all_obb_file)
  
if __name__ == "__main__":

    logging.basicConfig(filename='app.log', filemode='a', format='%(asctime)s - %(levelname)s - %(message)s', level=logging.DEBUG)

    main()
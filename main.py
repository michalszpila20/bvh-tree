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

def begin(filename, x_axis, y_axis, z_axis, rot_x_axis, rot_y_axis, rot_z_axis, scale):
    """Open .obj file, build triangles and centroids"""

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

def build(node_list, bbox_type, filename, x_axis, y_axis, z_axis, rot_x_axis, rot_y_axis, rot_z_axis, scale, test_type, termination_step):
    """First iteration of the recursion, creation of the root node"""
    
    logging.debug("build function")

    obj_list_copy = begin(filename, x_axis, y_axis, z_axis, rot_x_axis, rot_y_axis, rot_z_axis, scale)
    
    logging.debug(f"obj_list_copy: {obj_list_copy}")

    root = Node(obj_list_copy)

    world_box = None

    if bbox_type == "aabb":
        world_box = calculate_box_AABB(obj_list_copy, test_type)
    elif bbox_type == "obb":  
        world_box = calculate_box_OBB(obj_list_copy)
    elif bbox_type == "sphere":
        world_box = calculate_box_sphere(obj_list_copy)

    root.set_bbox(world_box)
    node_list.append(root)

    build_recursive(root, 0, node_list, bbox_type, test_type, termination_step)
    return node_list

def build_recursive(node, depth, node_list, bbox_type, test_type, termination_step):
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
            left_index_box = calculate_box_AABB(obj_list_node_sorted[left_index:split_index], test_type)
            right_index_box = calculate_box_AABB(obj_list_node_sorted[split_index:right_index], test_type)
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
        build_recursive(node.left, depth + 1, node_list, bbox_type, test_type, termination_step)
        build_recursive(node.right, depth + 1, node_list, bbox_type, test_type, termination_step)
        
def main():

    filename_B = None
    bbox_type_B = None
    node_list_A = []
    node_list_B = []

    # hard coding for testing
    bbox_type_A = input("Box type: aabb / sphere / obb: ")
    filename_A = input("Choose obj file: bear / boat / cow / pumpkin / rabbit / teapot: ")
    test_type = input("Ray intersect [a] or collision detection [b] or BVH creation [c]?: ")
    # bbox_type_A = sys.argv[1]
    # filename_A = sys.argv[2]
    # termination_step = int(sys.argv[3])
    # test_type = sys.argv[4]
    

    if test_type == 'b':
        # hard coding for testing
        bbox_type_B = input("Box type: aabb / sphere / obb: ")
        filename_B = input("Choose the second obj file: bear / boat / cow / pumpkin / rabbit / teapot: ")
        # bbox_type_B = sys.argv[5]
        # filename_B = sys.argv[6]
        pass

    right_position = False
    # x_axis_A = float(sys.argv[7])
    # y_axis_A = float(sys.argv[8])
    # z_axis_A = float(sys.argv[9])
    # rot_x_axis_A = float(sys.argv[10])
    # rot_y_axis_A = float(sys.argv[11])
    # rot_z_axis_A = float(sys.argv[12])
    # scale_A = float(sys.argv[13])
    
    # x_axis_B = float(sys.argv[14])
    # y_axis_B = float(sys.argv[15])
    # z_axis_B = float(sys.argv[16])
    # rot_x_axis_B = float(sys.argv[17])
    # rot_y_axis_B = float(sys.argv[18])
    # rot_z_axis_B = float(sys.argv[19])
    # scale_B = float(sys.argv[20])

    x_axis_A = 0
    y_axis_A = 0
    z_axis_A = 0
    rot_x_axis_A = 0
    rot_y_axis_A = 0
    rot_z_axis_A = 0
    scale_A = 1
    
    x_axis_B = 0
    y_axis_B = 0
    z_axis_B = 0
    rot_x_axis_B = 0
    rot_y_axis_B = 0
    rot_z_axis_B = 0
    scale_B = 1 

    termination_step = 4
    
    if filename_A == "bear":
        ray_origin = [-2, 10, 29]
        ray_dest = [1, -17, -15]
    elif filename_A == "ducky":
        ray_origin = [1, -2, 0.5]
        ray_dest = [-1.5, 3, -2]
    elif filename_A == "bunny":
        ray_origin = [0.15, 0, 0.1]
        ray_dest = [-0.15, 0, -0.05]
    elif filename_A == "dragon":
        ray_origin = [0.6, -0.5, 0.62]
        ray_dest = [0.5, 0.3, -1]
        # ray_origin = [0.5, -0.5, 0.6]
        # ray_dest = [0.5, 0.3, -1]
    #bear rotX = 90 and rotZ = 90
    # ray_origin = [-2, 10, 29]
    # ray_dest = [1, -17, -15]

    # ducky rotX = 90
    # ray_origin = [1, -2, 0.5]
    # ray_dest = [-1.5, 3, -2]

    #bunny rotX = 90
    # ray_origin = [0.15, 0, 0.1]
    # ray_dest = [-0.15, 0, -0.05]

    #dragon
    # ray_origin = [0.5, -0.5, 0.6]
    # ray_dest = [0.5, 0.3, -1]

    # fig1 = rotate_move_obj_files(filename_A, x_axis_A, y_axis_A, z_axis_A, rot_x_axis_A, rot_y_axis_A, rot_z_axis_A,
    #                         filename_B, x_axis_B, y_axis_B, z_axis_B, rot_x_axis_B, rot_y_axis_B, rot_z_axis_B)
    # fig1.show()

    if test_type == "b":
        while not right_position:
            fig = rotate_move_obj_files(filename_A, 0, 0, 0, 0, 0, 0, 1,
                            filename_B, 0, 0, 0, 0, 0, 0, 1)
            fig.show()
            position = input("Do you want to move object? [y/n]")
            if position == 'y':
                print("Object A:")
                x_axis_A = float(input("Move in x axis:"))
                y_axis_A = float(input("Move in y axis:"))
                z_axis_A = float(input("Move in z axis:"))
                rot_x_axis_A = float(input("Rotate in x axis:"))
                rot_y_axis_A = float(input("Rotate in y axis:"))
                rot_z_axis_A = float(input("Rotate in z axis:"))
                scale_A = float(input("Set scale:"))

                print("=========================================")
                print("Object B:")
                x_axis_B = float(input("Move in x axis:"))
                y_axis_B = float(input("Move in y axis:"))
                z_axis_B = float(input("Move in z axis:"))
                rot_x_axis_B = float(input("Rotate in x axis:"))
                rot_y_axis_B = float(input("Rotate in y axis:"))
                rot_z_axis_B = float(input("Rotate in z axis:"))
                scale_B = float(input("Set scale:"))
                fig1 = rotate_move_obj_files(filename_A, x_axis_A, y_axis_A, z_axis_A, rot_x_axis_A, rot_y_axis_A, rot_z_axis_A, scale_A,
                            filename_B, x_axis_B, y_axis_B, z_axis_B, rot_x_axis_B, rot_y_axis_B, rot_z_axis_B, scale_B)
                fig1.show()
            elif position == 'n':
                right_position = True
    elif test_type == "c":
        while not right_position:
            rotate_move_obj_file(filename_A, 0, 0, 0, 0, 0, 0, 1)
            position = input("Do you want to move object? [y/n]")
            if position == 'y':
                print("Object A:")
                x_axis_A = float(input("Move in x axis:"))
                y_axis_A = float(input("Move in y axis:"))
                z_axis_A = float(input("Move in z axis:"))
                rot_x_axis_A = float(input("Rotate in x axis:"))
                rot_y_axis_A = float(input("Rotate in y axis:"))
                rot_z_axis_A = float(input("Rotate in z axis:"))
                rotate_move_obj_file(filename_A, x_axis_A, y_axis_A, z_axis_A, rot_x_axis_A, rot_y_axis_A, rot_z_axis_A, scale_A)
            elif position == 'n':
                right_position = True
    else:
        while not right_position:
            rotate_move_obj_file_ray_no_collision(filename_A, 0, 0, 0, 0, 0, 0, ray_origin, ray_dest, scale_A)
            position = input("Do you want to move object? [y/n]")
            if position == 'y':
                print("Object A:")
                x_axis_A = float(input("Move in x axis:"))
                y_axis_A = float(input("Move in y axis:"))
                z_axis_A = float(input("Move in z axis:"))
                rot_x_axis_A = float(input("Rotate in x axis:"))
                rot_y_axis_A = float(input("Rotate in y axis:"))
                rot_z_axis_A = float(input("Rotate in z axis:"))
                rotate_move_obj_file_ray_no_collision(filename_A, x_axis_A, y_axis_A, z_axis_A, rot_x_axis_A, rot_y_axis_A, rot_z_axis_A, ray_origin, ray_dest, scale_A)
            elif position == 'n':
                right_position = True

    logging.debug(f"x_axis_A, y_axis_A, z_axis_A: {x_axis_A}, {y_axis_A}, {z_axis_A}")
    logging.debug(f"rot_x_axis_A, rot_y_axis_A, rot_z_axis_A: {rot_x_axis_A}, {rot_y_axis_A}, {rot_z_axis_A}")

    logging.debug(f"x_axis_B, y_axis_B, z_axis_B: {x_axis_B}, {y_axis_B}, {z_axis_B}")
    logging.debug(f"rot_x_axis_B, rot_y_axis_B, rot_z_axis_B: {rot_x_axis_B}, {rot_y_axis_B}, {rot_z_axis_B}")

    if test_type == "c":
        start_time_BVH = time.time()
        tracemalloc.start()
        logging.info(f"time of start_time_BVH: {start_time_BVH}")

    node_list_A = build(node_list_A, bbox_type_A, filename_A, x_axis_A, y_axis_A, z_axis_A, rot_x_axis_A, rot_y_axis_A, rot_z_axis_A, scale_A, test_type, termination_step)

    if test_type == "c":    
        logging.info(f"tracemalloc.get_traced_memory(): {tracemalloc.get_traced_memory()}")
        tracemalloc.stop()
        stop_time_BVH = time.time()
        logging.info(f"time of stop_time_BVH: {stop_time_BVH}")
        logging.info(f"time of execution: {stop_time_BVH - start_time_BVH}")

    #aabb
    # all_aabb_file = save_obj_aabb(node_list_A, filename_A)
    # plot_BVH_from_obj(all_aabb_file)

    #obb
    # all_obb_file = save_obj_obb(node_list_A, filename_A)
    # plot_BVH_from_obj(all_obb_file)

    #sphere
    # all_sphere_file = save_obj_sphere(node_list_A, 6, filename_A)
    # plot_BVH_from_obj(all_sphere_file)

    if test_type == 'b':
        node_list_B = build(node_list_B, bbox_type_B, filename_B, x_axis_B, y_axis_B, z_axis_B, rot_x_axis_B, rot_y_axis_B, rot_z_axis_B, scale_B, test_type, termination_step) 
    
    if test_type == 'a':
        ray_start = time.time() 
        tracemalloc.start()
        is_intersection, ray_dest_new, closest_hit = ray_intersect(ray_origin, ray_dest, node_list_A, bbox_type_A)
        logging.info(f"tracemalloc.get_traced_memory(): {tracemalloc.get_traced_memory()}")
        tracemalloc.stop()
        stop_time = time.time()
        logging.info(f"time of stop_time: {stop_time}")
        logging.info(f"time of execution: {stop_time - ray_start}")
        rotate_move_obj_file_ray(filename_A, x_axis_A, y_axis_A, z_axis_A, rot_x_axis_A, rot_y_axis_A, rot_z_axis_A, ray_origin, ray_dest, closest_hit, scale_A)
    elif test_type == 'b':
        start_time = time.time()
        tracemalloc.start()
        collisions = BVH_collision_detection(node_list_A, node_list_B, bbox_type_A, bbox_type_B)
        logging.info(f"tracemalloc.get_traced_memory(): {tracemalloc.get_traced_memory()}")
        tracemalloc.stop()
        stop_time = time.time()
        logging.info(f"time of stop_time: {stop_time}")
        logging.info(f"time of execution: {stop_time - start_time}")
        logging.info(f"number of collisions: {len(collisions)}")

        # fig = rotate_move_obj_files(filename_A, x_axis_A, y_axis_A, z_axis_A, rot_x_axis_A, rot_y_axis_A, rot_z_axis_A, scale_A,
        #                       filename_B, x_axis_B, y_axis_B, z_axis_B, rot_x_axis_B, rot_y_axis_B, rot_z_axis_B, scale_B)

        # plot_collisions(collisions, fig, bbox_type_A, bbox_type_B)
        
if __name__ == "__main__":

    logging.basicConfig(filename='app.log', filemode='a', format='%(asctime)s - %(levelname)s - %(message)s', level=logging.DEBUG)

    main()
import math
import numpy as np
from ray_intersection.ray_intersection_utils import intersect_line_triangle_boolean, intersect_line_triangle_closest
import logging

def ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation):

    logging.debug(f"obb_center: {obb_center}")
    logging.debug(f"obb_half_extents: {obb_half_extents}")
    logging.debug(f"obb_rotation: {obb_rotation}")

    magnitude = math.sqrt(pow(ray_dest[0] - ray_origin[0], 2) + pow(ray_dest[1] - ray_origin[1], 2) + pow(ray_dest[2] - ray_origin[2], 2))
    vector_x_len = ray_dest[0] - ray_origin[0]
    vector_y_len = ray_dest[1] - ray_origin[1]
    vector_z_len = ray_dest[2] - ray_origin[2]

    logging.debug(f"magnitude: {magnitude}")
    logging.debug(f"vector_x_len: {vector_x_len}")
    logging.debug(f"vector_y_len: {vector_y_len}")
    logging.debug(f"vector_z_len: {vector_z_len}")

    dir_ray_X = vector_x_len / magnitude
    dir_ray_Y = vector_y_len / magnitude
    dir_ray_Z = vector_z_len / magnitude

    logging.debug(f"dir_ray_X: {dir_ray_X}")
    logging.debug(f"dir_ray_Y: {dir_ray_Y}")
    logging.debug(f"dir_ray_Z: {dir_ray_Z}")

    ray_dir = np.array([dir_ray_X, dir_ray_Y, dir_ray_Z])

    local_ray_origin = np.dot(obb_rotation.T, ray_origin - obb_center)
    local_ray_dir = np.dot(obb_rotation.T, ray_dir)

    logging.debug(f"local_ray_origin: {local_ray_origin}")
    logging.debug(f"local_ray_dir: {local_ray_dir}")

    t_X_min = (-obb_half_extents[0] - local_ray_origin[0]) / local_ray_dir[0]
    t_Y_min = (-obb_half_extents[1] - local_ray_origin[1]) / local_ray_dir[1]
    t_Z_min = (-obb_half_extents[2] - local_ray_origin[2]) / local_ray_dir[2]

    t_X_max = (obb_half_extents[0] - local_ray_origin[0]) / local_ray_dir[0]
    t_Y_max = (obb_half_extents[1] - local_ray_origin[1]) / local_ray_dir[1]
    t_Z_max = (obb_half_extents[2] - local_ray_origin[2]) / local_ray_dir[2]

    t_X_near = min(t_X_min, t_X_max)
    t_X_far = max(t_X_min, t_X_max)

    t_Y_near = min(t_Y_min, t_Y_max)
    t_Y_far = max(t_Y_min, t_Y_max)

    t_Z_near = min(t_Z_min, t_Z_max)
    t_Z_far = max(t_Z_min, t_Z_max)

    t_near = max(t_X_near, t_Y_near, t_Z_near)
    t_far = min(t_X_far, t_Y_far, t_Z_far)

    logging.debug(f"t_near: {t_near}")
    logging.debug(f"t_far: {t_far}")

    if t_near < t_far:
        logging.debug("There is intersection!")
        return True, t_near
    else:
        logging.debug("There is no intersection!")
        return False, t_near

def intersection_OBB_boolean(ray_origin, ray_dest, node_list):

    current_node = node_list[0]
    node_stack = []
    
    #To do -> add these to OBB
    obb_center = current_node.get_bbox().centre
    obb_half_extents = current_node.get_bbox().half_extents
    obb_rotation = current_node.get_bbox().rotation

    print("--------------------------------------------")

    if not ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation):
        logging.debug("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node.is_leaf():

            logging.debug("Current node is not leaf, checking children of this node.")
            left_child_intersect = ray_intersection_OBB(ray_origin, ray_dest, current_node.left.get_bbox().centre, current_node.left.get_bbox().half_extents, current_node.left.get_bbox().rotation)
            right_child_intersect = ray_intersection_OBB(ray_origin, ray_dest, current_node.right.get_bbox().centre, current_node.right.get_bbox().half_extents, current_node.right.get_bbox().rotation)
            logging.debug(f"Left child intersect? {left_child_intersect}")
            logging.debug(f"Right child intersect? {right_child_intersect}")
            if left_child_intersect and right_child_intersect:
                logging.debug("Both intersect.")
                node_stack.append(current_node.right)
                logging.debug(f"Node stack is: {node_stack}")
                current_node = current_node.left
                logging.debug(f"Curent node is: {current_node}, when both node were intersected.")
                continue
            elif left_child_intersect and not right_child_intersect:
                current_node = current_node.left
                logging.debug(f"Curent node is: {current_node}, when only one node was intersected (left).")
                continue
            elif not left_child_intersect and right_child_intersect:
                current_node = current_node.right
                logging.debug(f"Curent node is: {current_node}, when only one node was intersected (right).")
                continue
            else:
                logging.debug("Both nodes were not hit, do nothing!")
        else:
            logging.debug("Final intersection!!!")
            is_hit = intersect_line_triangle_boolean(current_node, ray_origin, ray_dest)
            
            logging.debug(f"is_hit after final intersection: {is_hit}")

            if is_hit == True:
                return True

        current_node = node_stack.pop(0)
        logging.debug(f"current_node after no hit: {current_node}")

        if len(node_stack) == 0:
            return False

def intersection_OBB_closest(ray_origin, ray_dest, node_list):

    current_node = [node_list[0], 0]
    node_stack = []
    
    obb_center = current_node[0].get_bbox().centre
    obb_half_extents = current_node[0].get_bbox().half_extents
    obb_rotation = current_node[0].get_bbox().rotation

    logging.debug("--------------------------------------------")

    ray_dest_new = ray_dest 
    closest_hit = None

    if not ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation):
        logging.debug("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node[0].is_leaf():
            logging.debug("Current node is not leaf, checking children of this node.")
            left_child_intersect, t_near_left = ray_intersection_OBB(ray_origin, ray_dest, current_node[0].left.get_bbox().centre, current_node[0].left.get_bbox().half_extents, current_node[0].left.get_bbox().rotation)
            right_child_intersect, t_near_right = ray_intersection_OBB(ray_origin, ray_dest, current_node[0].right.get_bbox().centre, current_node[0].right.get_bbox().half_extents, current_node[0].right.get_bbox().rotation)
            logging.debug(f"Left child intersect? {left_child_intersect}")
            logging.debug(f"Right child intersect? {right_child_intersect}")
            if left_child_intersect and right_child_intersect:
                logging.debug("Both intersect.")

                if t_near_left < t_near_right:
                    node_stack.append([current_node[0].right, t_near_right])
                    current_node[0] = current_node[0].left
                    logging.debug("Left node is closer, traverse left node, right on stack.")
                else:
                    node_stack.append([current_node[0].left, t_near_left])
                    current_node[0] = current_node[0].right
                    logging.debug("Right node is closer, traverse right node, left on stack.")
                
                logging.debug(f"Node stack is: {node_stack}")
                logging.debug(f"Curent node is: {current_node}, when both node were intersected.")
                continue
            elif left_child_intersect and not right_child_intersect:
                current_node[0] = current_node[0].left
                current_node[1] = t_near_left
                logging.debug(f"Curent node is: {current_node}, when only one node was intersected (left).")
                continue
            elif not left_child_intersect and right_child_intersect:
                current_node[0] = current_node[0].right
                current_node[1] = t_near_right
                logging.debug(f"Curent node is: {current_node}, when only one node was intersected (right).")
                continue
            else:
                logging.debug("Both nodes were not hit, do nothing!")
        else:
            logging.debug("Final intersection!!!")
            ray_dest_new, closest_hit = intersect_line_triangle_closest(current_node, ray_origin, ray_dest, closest_hit)
        
        while node_stack:

            logging.debug(f"Ultimate node stack is: {node_stack}")

            node = node_stack.pop(0)

            logging.debug(f"node is in popping: {node}")
            logging.debug(f"node is in popping [0]: {node[0]}")
            logging.debug(f"node is in popping [1]: {node[1]}")
            logging.debug(f"math.dist(ray_origin, ray_dest_new): {math.dist(ray_origin, ray_dest_new)}")

            if node[1] < math.dist(ray_origin, ray_dest_new):
                current_node = node
                logging.debug(f"Current node: {current_node}, node[1] < ray_dest_new!")
                break
            elif node[1] > math.dist(ray_origin, ray_dest_new):
                logging.debug(f"Current node: {current_node}, node[1] > ray_dest_new!!")

            logging.debug(f"node stack after loop is: {node_stack}")

            if len(node_stack) == 0:
                if closest_hit != None:
                    return True, ray_dest_new, closest_hit
                else:
                    return False, ray_dest_new, closest_hit
        

import logging
from ray_intersection.ray_intersection_utils import intersect_line_triangle_boolean, intersect_line_triangle_closest
import math

def ray_intersection_sphere(centre, radius, ray_origin, ray_dest):

    x1 = ray_origin[0]
    y1 = ray_origin[1] 
    z1 = ray_origin[2]

    x2 = ray_dest[0]
    y2 = ray_dest[1]
    z2 = ray_dest[2]
    
    x3 = centre[0]
    y3 = centre[1]
    z3 = centre[2]

    logging.debug(f"x3: {x3}")
    logging.debug(f"y3: {y3}")
    logging.debug(f"z3: {z3}")

    a = pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2)
    b = 2 * ((x2 - x1)*(x1 - x3) + (y2 - y1)*(y1 - y3) + (z2 - z1)*(z1 - z3))
    c = pow(x3, 2) + pow(y3, 2) + pow(z3, 2) + pow(x1, 2) + pow(y1, 2) + pow(z1, 2) - 2*((x3 * x1) + (y3*y1) + (z3*z1)) - pow(radius, 2)
    end_val = b * b - 4 * a * c

    logging.debug(f"end_val: {end_val}")
    
    if end_val < 0:
        logging.debug("There is no intersection!")
        return False, end_val
    elif end_val >= 0:
        logging.debug("There is an intersection!")
        return True, end_val

def intersection_sphere_boolean(ray_origin, ray_dest, node_list):

    current_node = node_list[0]
    node_stack = []

    centre = current_node.get_bbox().centre
    radius = current_node.get_bbox().radius

    logging.debug("--------------------------------------------")

    if not ray_intersection_sphere(centre, radius, ray_origin, ray_dest):
        logging.debug("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node.is_leaf():

            logging.debug("Current node is not leaf, checking children of this node.")
            left_child_intersect = ray_intersection_sphere(current_node.left.get_bbox().centre, current_node.left.get_bbox().radius, ray_origin, ray_dest)
            right_child_intersect = ray_intersection_sphere(current_node.right.get_bbox().centre, current_node.right.get_bbox().radius, ray_origin, ray_dest)
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

def intersection_sphere_closest(ray_origin, ray_dest, node_list):

    current_node = [node_list[0], 0]
    node_stack = []
    
    centre = current_node[0].get_bbox().centre
    radius = current_node[0].get_bbox().radius
    
    logging.debug("--------------------------------------------")

    ray_dest_new = ray_dest 
    closest_hit = None

    if not ray_intersection_sphere(centre, radius, ray_origin, ray_dest):
        logging.debug("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node[0].is_leaf():
            logging.debug("Current node is not leaf, checking children of this node.")
            left_child_intersect, t_near_left = ray_intersection_sphere(current_node[0].left.get_bbox().centre, current_node[0].left.get_bbox().radius, ray_origin, ray_dest_new)
            right_child_intersect, t_near_right = ray_intersection_sphere(current_node[0].right.get_bbox().centre, current_node[0].right.get_bbox().radius, ray_origin, ray_dest_new)
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
            ray_dest_new, closest_hit = intersect_line_triangle_closest(current_node, ray_origin, ray_dest_new, closest_hit)
            logging.debug(f"Function exit: ray_dest_new: {ray_dest_new}, closest_hit: {closest_hit}")
        logging.debug(f"node_stack before while loop: {node_stack}")

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

        logging.debug(f"length of node stack {len(node_stack)}")
        if len(node_stack) == 0:
                logging.debug(f"{closest_hit}, {ray_dest_new}")
                if closest_hit != None:
                    return True, ray_dest_new, closest_hit
                else:
                    return False, ray_dest_new, closest_hit
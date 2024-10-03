import math
import numpy as np
from ray_intersection.ray_intersection_utils import intersect_line_triangle_closest, plot_triangle_ray
import logging

def ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation):

    logging.debug(f"ray_intersection_OBB")

    EPSILON = 1e-6

    # Convert inputs to numpy arrays
    ray_origin = np.array(ray_origin)
    ray_dest = np.array(ray_dest)
    obb_center = np.array(obb_center)
    obb_half_extents = np.array(obb_half_extents)

    # Calculate direction vector and normalize it
    ray_dir = ray_dest - ray_origin
    magnitude = np.linalg.norm(ray_dir)
    ray_dir = ray_dir / (magnitude + EPSILON)

    # Transform ray into the OBB's local space
    local_ray_origin = np.dot(obb_rotation.T, ray_origin - obb_center)
    local_ray_dir = np.dot(obb_rotation.T, ray_dir)

    # Calculate intersection t-values for each axis, using np.where for robustness
    t_min_vals = np.where(local_ray_dir != 0, (-obb_half_extents - local_ray_origin) / local_ray_dir, -np.inf)
    t_max_vals = np.where(local_ray_dir != 0, (obb_half_extents - local_ray_origin) / local_ray_dir, np.inf)

    # Calculate near and far values for each axis
    t_near_vals = np.minimum(t_min_vals, t_max_vals)
    t_far_vals = np.maximum(t_min_vals, t_max_vals)

    # Determine the overall t_near and t_far for the intersection
    t_near = np.max(t_near_vals)
    t_far = np.min(t_far_vals)

    # Check if there's a valid intersection within the ray's bounds
    if t_near < t_far - EPSILON and t_near >= 0 and t_near <= magnitude:
        return True, t_near
    else:
        return False, t_near

def intersection_OBB_closest(ray_origin, ray_dest, node_list):
    current_node = [node_list[0], 0]
    node_stack = []

    obb_center = current_node[0].get_bbox().centre
    obb_half_extents = current_node[0].get_bbox().half_extents
    obb_rotation = current_node[0].get_bbox().rotation

    logging.debug("--------------------------------------------")

    ray_dest_new = ray_dest
    closest_hit = None

    # Initial intersection check with the root OBB
    intersection_root, t_near_root = ray_intersection_OBB(ray_origin, ray_dest, obb_center, obb_half_extents, obb_rotation)

    logging.debug(f"intersection_root: {intersection_root}")
    logging.debug(f"t_near_root: {t_near_root}")

    if not intersection_root:
        logging.debug("No intersection in root node, returning False.")
        return False, None, None

    while True:
        if not node_stack and not current_node:
            # Exit if the stack is empty and there’s no valid hit
            return False, ray_dest_new, closest_hit if closest_hit else None

        if not current_node[0].is_leaf():
            logging.debug("Current node is not leaf, checking children of this node.")
            left_child_bbox = current_node[0].left.get_bbox()
            right_child_bbox = current_node[0].right.get_bbox()

            left_child_intersect, t_near_left = ray_intersection_OBB(
                ray_origin, ray_dest_new,
                left_child_bbox.centre,
                left_child_bbox.half_extents,
                left_child_bbox.rotation
            )

            right_child_intersect, t_near_right = ray_intersection_OBB(
                ray_origin, ray_dest_new,
                right_child_bbox.centre,
                right_child_bbox.half_extents,
                right_child_bbox.rotation
            )

            if left_child_intersect and right_child_intersect:
                # Put the furthest on the stack
                if t_near_left < t_near_right:
                    node_stack.append([current_node[0].right, t_near_right])
                    current_node = [current_node[0].left, t_near_left]
                else:
                    node_stack.append([current_node[0].left, t_near_left])
                    current_node = [current_node[0].right, t_near_right]
            elif left_child_intersect:
                current_node = [current_node[0].left, t_near_left]
            elif right_child_intersect:
                current_node = [current_node[0].right, t_near_right]
            else:
                # No intersections, pop the next node if available
                if node_stack:
                    current_node = node_stack.pop()
                else:
                    break
        else:
            # Leaf node - perform intersection with each primitive
            ray_dest_new, closest_hit = intersect_line_triangle_closest(current_node, ray_origin, ray_dest_new, closest_hit)

            if closest_hit:
                # plot_triangle_ray(closest_hit, ray_origin, ray_dest_new)
                return True, ray_dest_new, closest_hit

            # Pop the next node if available
            if node_stack:
                current_node = node_stack.pop()
            else:
                return False, ray_dest_new, closest_hit if closest_hit else None
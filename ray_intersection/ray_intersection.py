import plotly.graph_objects as go
import numpy as np
from ray_intersection.ray_intersection_AABB import intersection_AABB_closest
from ray_intersection.ray_intersection_sphere import intersection_sphere_closest
from ray_intersection.ray_intersection_OBB import intersection_OBB_closest
import logging

def ray_intersect(ray_origin, ray_dest, node_list, bbox_type):

    is_intersection = False
    ray_dest_new = None
    closest_hit = None

    if bbox_type == "aabb":
        is_intersection, ray_dest_new, closest_hit = intersection_AABB_closest(ray_origin, ray_dest, node_list)
        logging.debug(f"is_intersection, ray_dest_new, closest_hit: {is_intersection}, {ray_dest_new}, {closest_hit}")
    elif bbox_type == "sphere":
        is_intersection, ray_dest_new, closest_hit = intersection_sphere_closest(ray_origin, ray_dest, node_list)
        logging.debug(f"is_intersection, ray_dest_new, closest_hit: {is_intersection}, {ray_dest_new}, {closest_hit}")
    elif bbox_type == "obb":
        is_intersection, ray_dest_new, closest_hit = intersection_OBB_closest(ray_origin, ray_dest, node_list)
        logging.debug(f"is_intersection, ray_dest_new, closest_hit: {is_intersection}, {ray_dest_new}, {closest_hit}")
    return is_intersection, ray_dest_new, closest_hit
import plotly.graph_objects as go
import numpy as np
from ray_intersection.ray_intersection_AABB import intersection_AABB_boolean, intersection_AABB_closest
from ray_intersection.ray_intersection_sphere import intersection_sphere
from ray_intersection.ray_intersection_OBB import intersection_OBB_boolean, intersection_OBB_closest

def ray_intersect(ray_origin, ray_dest, node_list):

    bvh_type = "sphere"
    is_intersection = False

    if bvh_type == "AABB":
        is_intersection, ray_dest_new, closest_hit = intersection_AABB_closest(ray_origin, ray_dest, node_list)
    elif bvh_type == "sphere":
        is_intersection = intersection_sphere(ray_origin, ray_dest, node_list)
    elif bvh_type == "OBB":
        is_intersection = intersection_OBB_boolean(ray_origin, ray_dest, node_list)
    
    return is_intersection
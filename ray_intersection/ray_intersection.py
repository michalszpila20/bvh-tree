import plotly.graph_objects as go
import numpy as np
from ray_intersection.ray_intersection_AABB import intersection_AABB_boolean, intersection_AABB_closest
from ray_intersection.ray_intersection_sphere import intersection_sphere_boolean, intersection_sphere_closest
from ray_intersection.ray_intersection_OBB import intersection_OBB_boolean, intersection_OBB_closest

def ray_intersect(ray_origin, ray_dest, node_list, bbox_type):

    is_intersection = False

    ray_test = input("Boolean [a] or Closest [b] Ray intersect?: ") 

    if bbox_type == "aabb":
        if ray_test == 'a':
            is_intersection = intersection_AABB_boolean(ray_origin, ray_dest, node_list)
        elif ray_test == 'b':
            is_intersection = intersection_AABB_closest(ray_origin, ray_dest, node_list)
    elif bbox_type == "sphere":
        if ray_test == 'a':
            is_intersection = intersection_sphere_boolean(ray_origin, ray_dest, node_list)
        elif ray_test == 'b':
            is_intersection = intersection_sphere_closest(ray_origin, ray_dest, node_list)
        
    elif bbox_type == "obb":
        if ray_test == 'a':
            is_intersection = intersection_OBB_boolean(ray_origin, ray_dest, node_list)
        elif ray_test == 'b':
            is_intersection = intersection_OBB_closest(ray_origin, ray_dest, node_list)
    return is_intersection
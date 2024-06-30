import plotly.graph_objects as go
import numpy as np
from ray_intersection_AABB import intersection_AABB
from ray_intersection_sphere import intersection_sphere
from ray_intersection_OBB import intersection_OBB

def ray_intersect(ray_origin, ray_dest, node_list):

    bvh_type = "OBB"

    if bvh_type == "AABB":
        intersection_AABB(ray_origin, ray_dest, node_list)
    elif bvh_type == "sphere":
        intersection_sphere(ray_origin, ray_dest, node_list)
    elif bvh_type == "OBB":
        intersection_OBB(ray_origin, ray_dest, node_list)

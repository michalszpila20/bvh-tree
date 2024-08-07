import logging
from collision_detection.collision_detection_AABB import collision_detection_AABB
from collision_detection.collision_detection_sphere import collision_detection_sphere
from collision_detection.collision_detection_obb import collision_detection_obb


def BVH_collision_detection(node_list_A, node_list_B, bbox_type_A, bbox_type_B):

    collisions = []

    if bbox_type_A == "aabb":
        collisions = collision_detection_AABB(node_list_A, node_list_B, bbox_type_B)
    elif bbox_type_A == "sphere":
        collisions = collision_detection_sphere(node_list_A, node_list_B, bbox_type_B)
    elif bbox_type_A == "obb":
        collisions = collision_detection_obb(node_list_A, node_list_B, bbox_type_B)
    return collisions
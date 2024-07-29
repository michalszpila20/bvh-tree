import logging
from obj_functions import save_obj, plot_two_obj_file

def test_AABB_AABB(aabb_a, aabb_b):

    logging.debug("test_AABB_AABB")

    if (aabb_a.get_bbox().maxs[0] < aabb_b.get_bbox().mins[0] or aabb_a.get_bbox().mins[0] > aabb_b.get_bbox().maxs[0]): return False
    if (aabb_a.get_bbox().maxs[1] < aabb_b.get_bbox().mins[1] or aabb_a.get_bbox().mins[1] > aabb_b.get_bbox().maxs[1]): return False
    if (aabb_a.get_bbox().maxs[2] < aabb_b.get_bbox().mins[2] or aabb_a.get_bbox().mins[2] > aabb_b.get_bbox().maxs[2]): return False

    return True

def descend_larger_method(tree_a, tree_b, index_a, index_b):

    descend = tree_b[index_b].is_leaf() or (not tree_a[index_a].is_leaf() and len(tree_a) >= len(tree_b))
    
    return descend

def descend_A(tree_a, index_a):

    return not tree_a[index_a].is_leaf()

def BVH_collision(tree_a, tree_b, index_a, index_b):

    logging.debug(f"index_a: {index_a}")
    logging.debug(f"index_b: {index_b}")

    logging.debug(f"tree_a: {tree_a}")
    logging.debug(f"tree_b: {tree_b}")

    aabb_a_temp = tree_a[index_a]
    aabb_b_temp = tree_b[index_b]

    if not test_AABB_AABB(aabb_a_temp, aabb_b_temp): return False

    if aabb_a_temp.is_leaf() and aabb_b_temp.is_leaf():
        logging.debug("Checking collisions on objects level.")
        return True
    else:
        if descend_A(tree_a, index_a):
            index_a_one = tree_a.index(aabb_a_temp.left)
            index_a_two = tree_a.index(aabb_a_temp.right)
            BVH_collision(tree_a, tree_b, index_a_one, index_b)
            BVH_collision(tree_a, tree_b, index_a_two, index_b)
        else:
            index_b_one = tree_b.index(aabb_b_temp.left)
            index_b_two = tree_b.index(aabb_b_temp.right)
            BVH_collision(tree_a, tree_b, index_a, index_b_one)
            BVH_collision(tree_a, tree_b, index_a, index_b_two)

def collision_detection_AABB(node_list_A, node_list_B):

    logging.debug("collision_detection for AABB")

    filename_A = save_obj(node_list_A, 'A')
    filename_B = save_obj(node_list_B, 'B')

    plot_two_obj_file(filename_A, filename_B)

    aabb_a = node_list_A[0]
    aabb_b = node_list_B[0]

    logging.debug(f"aabb_a: {aabb_a}")
    logging.debug(f"aabb_b: {aabb_b}")

    logging.debug(f"aabb_a mins and maxs: {aabb_a.get_bbox().mins}, {aabb_a.get_bbox().maxs}")
    logging.debug(f"aabb_b mins and maxs: {aabb_b.get_bbox().mins}, {aabb_b.get_bbox().maxs}")

    # is_collision = test_AABB_AABB(aabb_a, aabb_b)
    # logging.debug(f"is_collision: {is_collision}")

    is_collision = BVH_collision(node_list_A, node_list_B, 0, 0)
    logging.debug(f"is_collision: {is_collision}")

    
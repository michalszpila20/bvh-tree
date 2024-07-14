from obj_functions import open_obj_file, calculate_box_AABB, build_triangles, find_centroids, calculate_box_sphere, calculate_box_OBB, calculate_box_sphere_ritter
from bvh import BVHNode
from triangle import Triangle
import statistics
import sys
from ray_intersection.ray_intersection import ray_intersect

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

def begin():
    """Open .obj file, build triangles and centroids"""

    verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK = open_obj_file(r"D:\OneDrive\Pulpit\Praca_Magisterska\cow.obj.txt")
    #plot_obj_file(verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK)
    triangles = build_triangles(verticesX, verticesY, verticesZ, verticesI, verticesJ, verticesK)
    centroids = find_centroids(verticesI, triangles)

    obj_list = []

    for i in range(len(verticesI)):
        obj_list.append(Triangle([[triangles[i][0][0], triangles[i][0][1], triangles[i][0][2]],
                                [triangles[i][1][0], triangles[i][1][1], triangles[i][1][2]],
                                [triangles[i][2][0], triangles[i][2][1], triangles[i][2][2]]],
                                [centroids[i][0], centroids[i][1], centroids[i][2]]))
        
    return obj_list

def build(node_list):
    """First iteration of the recursion, creation of the root node"""
    
    print("build function")

    bbox_type = "sphere"

    obj_list_copy = begin()
    
    print(f"obj_list_copy: {obj_list_copy}")

    root = BVHNode(obj_list_copy)

    world_box = None

    if bbox_type == "AABB":
        world_box = calculate_box_AABB(obj_list_copy)
    elif bbox_type == "OBB":  
        world_box = calculate_box_OBB(obj_list_copy)
    elif bbox_type == "sphere":
        world_box = calculate_box_sphere(obj_list_copy)

    root.set_bbox(world_box)
    node_list.append(root)

    build_recursive(root, 0, node_list)
    return node_list

def build_recursive(node, depth, node_list):
    """Next iterations of the recursion, creation of the root node children"""

    print("--------------------------------------------------------------------------------------")
    print(f"build recursive: {depth}")

    bbox_type = "sphere"
    vertices_x = []
    vertices_y = []
    vertices_z = []
    obj_list_node = node.triangles
    median_split = False
    node.set_depth(depth)

    left_index = 0
    right_index = len(obj_list_node)

    print(f"left_index: {left_index}")
    print(f"right_index: {right_index}")
    print(f"right_index - left_index: {right_index - left_index}")

    if right_index - left_index <= 256:
        node.make_leaf()
        print("Make leaf!")
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

        print(f"axis_x_len: {axis_x_len}")
        print(f"axis_y_len: {axis_y_len}")
        print(f"axis_z_len: {axis_z_len}")

        axis_len = {
            "x": axis_x_len,
            "y": axis_y_len,
            "z": axis_z_len
        }
        
        max_len_axis = max(axis_len, key=axis_len.get)
        print(f"The largest axis is: {max_len_axis}")

        split_index_val = axis_len[max_len_axis] / 2
        print(f"split_index: {split_index_val}")

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
            print(f"Max axis: {max_len_axis}")
        elif max_len_axis == "y":
            centres = obj_centroid_y
            print(f"Max axis: {max_len_axis}")
        elif max_len_axis == "z":
            centres = obj_centroid_z
            print(f"Max axis: {max_len_axis}")

        left_index_pop = 0
        right_index_pop = 0
        split_index_min = min(centres) + split_index_val

        print(f"split_index_min: {split_index_min}")

        for centre in centres:
            if centre < split_index_min:
                left_index_pop += 1
            else:
                right_index_pop += 1

        print(f"left_index_pop: {left_index_pop}")
        print(f"right_index_pop: {right_index_pop}")

        axis_median = 0
        if left_index_pop <= 1 or right_index_pop <= 1:
            median_split = True
            axis_median = statistics.median(centres)
            print(f"axis_median: {axis_median}")

        
        print(f"Median split? {median_split}")
        # Calculate bounding boxes of left and right sides

        obj_list_node_sorted = []   
        for triangle in obj_list_node:
            if max_len_axis == "x":
                obj_list_node_sorted = sorted(obj_list_node, key=lambda triangle: triangle.centroid[0])
            elif max_len_axis == "y":
                obj_list_node_sorted = sorted(obj_list_node, key=lambda triangle: triangle.centroid[1])
            elif max_len_axis == "z":
                obj_list_node_sorted = sorted(obj_list_node, key=lambda triangle: triangle.centroid[2])

        #print(f"obj_list_node_sorted: {obj_list_node_sorted}")

        if median_split == True:
            split_index_min = axis_median
            print(f"split index val: {split_index_min}")

        split_index = left_index
        print(f"split index before: {split_index}")
        
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
        print(f"Split index after: {split_index}")

        print(f"Len obj_list_node_sorted: {len(obj_list_node_sorted)}")
        if right_index - left_index < 20:
            print(f"obj_list_copy[left_index:split_index]: {obj_list_node_sorted[left_index:split_index]}")
            print(f"obj_list_copy[split_index:right_index]: {obj_list_node_sorted[split_index:right_index]}")
        
        print(f"len(obj_list_copy[left_index:split_index]) : {len(obj_list_node_sorted[left_index:split_index])}")
        print(f"len(obj_list_copy[split_index:right_index]) : {len(obj_list_node_sorted[split_index:right_index])}")
        if bbox_type == "AABB":
            left_index_box = calculate_box_AABB(obj_list_node_sorted[left_index:split_index])
            right_index_box = calculate_box_AABB(obj_list_node_sorted[split_index:right_index])
        elif bbox_type == "sphere":
            left_index_box = calculate_box_sphere(obj_list_node_sorted[left_index:split_index])
            right_index_box = calculate_box_sphere(obj_list_node_sorted[split_index:right_index])
        elif bbox_type == "OBB":
            left_index_box = calculate_box_OBB(obj_list_node_sorted[left_index:split_index])
            right_index_box = calculate_box_OBB(obj_list_node_sorted[split_index:right_index])
        
        # Initiate current node as an interior node with leftNode and rightNode as children
        node.left = BVHNode(obj_list_node_sorted[left_index:split_index])
        node.right = BVHNode(obj_list_node_sorted[split_index:right_index])

        node.left.bbox = left_index_box
        node.right.bbox = right_index_box

        node_list.append(node.left)
        node_list.append(node.right)
        build_recursive(node.left, depth + 1, node_list)
        build_recursive(node.right, depth + 1, node_list)
        

def main():
    node_list = []
    node_list = build(node_list)
    
    ray_origin = [1.223, -2.78, 10]
    ray_dest = [-3, 5, -8]

    if ray_intersect(ray_origin, ray_dest, node_list):
        print("The end with intersection")
    else:
        print("The end without intersection")


if __name__ == "__main__":
    sys.stdout = open(r"D:\OneDrive\Pulpit\Praca_Magisterska\log.txt", 'w')
    main()
    



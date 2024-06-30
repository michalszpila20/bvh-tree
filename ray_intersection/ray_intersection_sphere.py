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

    print(f"x3: {x3}")
    print(f"y3: {y3}")
    print(f"z3: {z3}")

    a = pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2)
    b = 2 * ((x2 - x1)*(x1 - x3) + (y2 - y1)*(y1 - y3) + (z2 - z1)*(z1 - z3))
    c = pow(x3, 2) + pow(y3, 2) + pow(z3, 2) + pow(x1, 2) + pow(y1, 2) + pow(z1, 2) - 2*((x3 * x1) + (y3*y1) + (z3*z1)) - pow(radius, 2)
    end_val = b * b - 4 * a * c

    print(f"end_val: {end_val}")
    
    if end_val < 0:
        print("There is no intersection!")
        return False
    elif end_val >= 0:
        print("There is an intersection!")
        return True
    
def intersection_sphere(ray_origin, ray_dest, node_list):

    current_node = node_list[0]
    node_stack = []

    if not ray_intersection_sphere(current_node.get_bbox().centre, current_node.get_bbox().radius, ray_origin, ray_dest):
        print("No intersection in root node, returing False.")
        return False
    while 1:
        if not current_node.is_leaf():
            print("Current node is not leaf, checking children of this node.")
            left_child_intersect = ray_intersection_sphere(current_node.left.get_bbox().centre, current_node.left.get_bbox().radius, ray_origin, ray_dest)
            right_child_intersect = ray_intersection_sphere(current_node.right.get_bbox().centre, current_node.right.get_bbox().radius, ray_origin, ray_dest)
            print(f"Left child intersect? {left_child_intersect}")
            print(f"Right child intersect? {right_child_intersect}")
            if left_child_intersect and right_child_intersect:
                print("Both intersect.")
                node_stack.append(current_node.right)
                print(f"Node stack is: {node_stack}")
                current_node = current_node.left
                print(f"Curent node is: {current_node}, when both node were intersected.")
            elif left_child_intersect and not right_child_intersect:
                current_node = current_node.left
                print(f"Curent node is: {current_node}, when only one node was intersected (left).")
            elif not left_child_intersect and right_child_intersect:
                current_node = current_node.right
                print(f"Curent node is: {current_node}, when only one node was intersected (right).")
            else:
                print("Something went wrong!")
                return False
        else:
            print("Final intersection!!!")
            return True
        if len(node_stack) == 0:
            print("Stack is empty -> False.")
            False
        else:
            print("Pop stack_node element.")
            current_node = node_stack.pop()
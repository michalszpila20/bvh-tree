class BVHNode:
    
    def __init__(self, data):
        self.leaf = None # bool, is that node a leaf?
        self.triangles = data  # objects of class Triangle
        self.left = None # child of the root node on the left
        self.right = None # child of the root node on the right
        self.bbox = None # bounding box, object of class AABB
        self.depth = 0
        
    def set_bbox(self, object): 
        self.bbox = object

    def make_leaf(self):
        self.leaf = True

    def set_depth(self, object):
        self.depth = object

    def get_bbox(self):
        return self.bbox
    
    def get_triangles(self):
        return self.triangles
    
    def get_depth(self):
        return self.depth
    
    def is_leaf(self):
        return self.leaf
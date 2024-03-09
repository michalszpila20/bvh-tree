class Triangle:

    def __init__(self, vertices, centroid):
        self.vertices = vertices # list of triangle verticies: [[vertex_AX, vertex_AY, vertex_AZ],
                                 #[vertex_BX, vertex_BY, vertex_BZ], [vertex_CX, vertex_CY, vertex_CZ]]
        self.centroid = centroid # centre of the triangle [point_X, point_Y, point_Z]
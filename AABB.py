class AABB:
    
    def __init__(self, mins, maxs, corners = None, centre = None, half_extents = None, rotation = None):
        self.mins = mins # [min_X, min_Y, min_Z]
        self.maxs = maxs # [max_X, max_Y, max_Z]

        self.corners = corners
        self.centre = centre
        self.half_extents = half_extents
        self.rotation = rotation

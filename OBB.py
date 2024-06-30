class OBB:
    
    def __init__(self, corners, centre, half_extents, rotation):
        self.corners = corners 
        self.centre = centre
        self.half_extents = half_extents
        self.rotation = rotation
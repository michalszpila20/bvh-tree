class AABB:
    
    def __init__(self, mins, maxs):
        self.mins = mins # [min_X, min_Y, min_Z]
        self.maxs = maxs # [max_X, max_Y, max_Z]
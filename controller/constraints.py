class Constraints:
    def __init__(self, s = [1e-10]*2, r = [1.]*2, b = [1.]*2):
        # s-> sharpness of the corners of the constraint function
        # r-> range of the constraint
        # b-> offset to the range
        self.s = s
        self.r = r
        self.b = b

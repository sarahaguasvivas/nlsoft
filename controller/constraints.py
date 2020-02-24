class Constraints:
    def __init__(self, s = 1e-10, r = 1e-5, b = 1e-5):
        # s-> sharpness of the corners of the constraint function
        # r-> range of the constraint
        # b-> offset to the range
        self.s = s
        self.r = r
        self.b = b

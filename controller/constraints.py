class Constraints:
    """
        by Sarah Aguasvivas
        Container of Constraints class
        s, r and b specify roundness of corners
    """
    def __init__(self, s : float = 1e-10, r : float = 1., b : float = 1.):
        # s-> sharpness of the corners of the constraint function
        # r-> range of the constraint
        # b-> offset to the range
        self.s = s
        self.r = r
        self.b = b

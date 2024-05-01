class ControlPose:

    def __init__(self, x=0, y=0, z=0, o=0) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.o = o

        self.tramo = 0
        self.u = 0

    def __str__(self):
        return f'Pose: ({self.x}, {self.y}, {self.z}, {self.o})'
    


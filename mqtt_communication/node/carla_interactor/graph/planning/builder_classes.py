class Location:
    def __init__(self, x="", y="", z=""):
        self.x = x  # (float - meters)
        self.y = y # (float - meters)
        self.z = z # (float - meters)

class Rotation:
    def __init__(self, pitch="", yaw="", roll=""):
        self.pitch = pitch # (float - degrees)
        self.yaw = yaw # (float - degrees)
        self.roll = roll # (float - degrees)

class Transform:
    def __init__(self, location=Location("", "", ""), 
                       rotation=Rotation("", "", "")):
        self.location = Location(location.x, location.y, location.z)
        self.rotation = Rotation(rotation.pitch, rotation.yaw, rotation.roll)
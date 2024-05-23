from .builder_classes import Transform, Location, Rotation

class LandmarkPose:
    """
    Landmark's pose with respect to the of with respect to affecting road.
    """
    def __init__(self, s, t, z_offset, h_offset):
        self.s = s # (float - meters)
        self.t = t # (float - meters)
        self.z_offset = z_offset  # (float - meters)
        self.h_offset = h_offset  # (float - meters)

class LandmarkRoad:
    """
    LandmarkRoad is used to define each of the roads that are affected by a Landmark.
    """
    def __init__(self):
        self.road_id = None # (int)
        self.pose = None # LandmarkPose
        self.orientation = None # can be positive (¿??), negative (¿??) or both (affects in both directions of the road)
        self.lanes = [] # (int) if none afecta a todos

class Landmark:
    """
    For initializing a landmark in a specific position, the location
    parameter must be passed in Location format. If not, other option
    is not passing any parameter an set the parameters after initializing the
    Landmark object.
    """
    def __init__(self, location=Location("", "", ""), 
                       rotation=Rotation("", "", "")):
        self.id = "" # (int)
        self.name = "" # Name of the landmark in OpenDRIVE file (str) 
        self.is_dynamic = "" # (bool)
        self.height = ""  # (float - meters)
        self.width = ""  # (float - meters)
        self.length = ""  # (float - meters)
        self.roll = ""  # (float - rad)
        self.pitch = "" # (float - rad)
        #self.country = None # (str) Country code where the landmark is defined (default to OpenDRIVE is Germany 2017)
            ## At the moment, Roadrunner and Opendrive do not have these types well defined. 
            ## Therefore, we create our own types for the landmarks.
        
        self.type = "" # (str) Type identificator of the landmark according to the country code
        self.transform = Transform(location, rotation)
        self.affecting_roads = [] # LandmarkRoad()

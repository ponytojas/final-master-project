"""
Authors: Alejandro D. and J. Felipe Arango.
Last mod: Navil Abdeselam Abdel-lah. 13/02/2023

Some classes describing structures to parse a file in xodr format
    / header
        // geoReference
    / roads []
        // road
            /// link (roadLink)
                //// predecessor (roadPredecessor)
                //// successor (roadSuccessor)
            /// type
                //// speed
            /// planView
                //// geometry
            /// lanes
                //// laneOffset
                //// laneSections
                    ///// lane
                        ////// link (laneLink)
                            /////// predecessor (lanePredecessor)
                            /////// successor (laneSuccessor)
                        ////// width
                        ////// roadMark
                        ////// userData
            /// objects
                //// object
                    ///// cornerLocal
            /// signals
                //// signal
                    ///// validity
                    ///// userData
                //// signalReference
                    ///// validity
                    ///// userData
    / controllers []
        // controller
            /// control
    / junctions []
        // junction
            /// [] connection
                //// link (junctionLaneLink)
            /// [] controller
            /// userData
"""
##############
### HEADER ###

class Header:
    def __init__(self, revMajor = '', revMinor='', name='', version='', date='', north='', south='', east='', west='', vendor=''):
        self.revMajor = revMajor
        self.revMinor = revMinor
        self.name = name
        self.version = version
        self.date = date
        self.north = north
        self.south = south
        self.east = east
        self.west = west
        self.vendor = vendor
        self.geoReference = GeoReference()

class GeoReference:
    def __init__(self):
        self.lat_0 = ""
        self.lon_0 = ""
        self.k = ""
        self.x_0 = ""
        self.y_0 = ""
        self.datum = ""
        self.units = ""
        self.geoidgrids = ""
        self.vunits = ""

### HEADER ###
##############


### SIGNALS ###
###############

class Signals:
    def __init__(self):
        self.signal = [] #Signal()
        self.signalReference = [] #SignalReference()

class Signal:
    def __init__(self):
        self.name = ""  # (str)
        self.id = "" # (int)
        self.s = ""  # (float - meters)
        self.t = ""  # (float - meters)
        self.zOffset = ""
        self.hOffset = ""
        self.roll = ""
        self.pitch = ""
        self.orientation = ""
        self.dynamic = ""
        self.country = ""
        self.type = ""
        self.subtype = ""
        self.value = ""
        self.text = ""
        self.height = ""
        self.width = ""
        self.validities = [] # Validity()
        self.userData = [] # VectorSignal()

class SignalReference:
    def __init__(self):
        self.id = ""
        self.s = ""
        self.t = ""
        self.orientation = ""
        self.validities = [] # Validity()
        self.userData = [] # VectorSignal()

class VectorSignal:
    def __init__(self):
        self.signalId = ""
        self.gateId = ""
        self.turnRelation = ""

class Validity:
    def __init__(self):
        self.fromLane = ""
        self.toLane = ""

### SIGNALS ###
###############


### ROADS ###
#############

class RoadLink:
    def __init__(self):
        self.predecessor = RoadPredecessor()
        self.successor = RoadSuccessor()

class RoadPredecessor:
    def __init__(self):
        self.elementType = "" # (string)
        self.elementId = ""  # (int)
        self.contactPoint = ""

class RoadSuccessor:
    def __init__(self):
        self.elementType = ""
        self.elementId = ""
        self.contactPoint = ""

class RoadType:
    def __init__(self):
        self.s = ""
        self.type = ""
        self.country = ""
        self.speed = Speed()

class Speed:
    def __init__(self):
        self.max = 0
        self.unit = ""

class Geometry:
    def __init__(self, s='', x='', y='', hdg='', length='', type='', curvature=''):
        self.s = s
        self.x = x
        self.y = y
        self.hdg = hdg
        self.length = length
        self.type = type
        self.curvature = curvature

class Elevation:
    def __init__(self, s='', a='', b='', c='', d=''):
        self.s = s
        self.a = a
        self.b = b
        self.c = c
        self.d = d

### ROADS ###
#############


### LANES ###
#############

class Lanes:
    def __init__(self):
        self.laneOffset = [] #LaneOffset
        self.laneSections = [] #LaneSection()

class LaneOffset:
    def __init__(self):
        self.s = ""
        self.a = ""
        self.b = ""
        self.c = ""
        self.d = ""

class LaneSection:
    def __init__(self, s='', left=[], center=[], right=[]):
        self.s = s
        self.left = left
        self.center = center
        self.right = right

class Lane:
    def __init__(self):
        self.id = ""
        self.type = ""
        self.level = ""
        self.link = LaneLink()
        self.width = [] #LaneWidth()
        self.roadMark = [] #RoadMark
        self.userData = [] #VectorLane

class LaneLink:
    def __init__(self):
        self.predecessor = LanePredecessor()
        self.successor = LaneSuccessor()

class LanePredecessor:
    def __init__(self):
        self.id = ""

class LaneSuccessor:
    def __init__(self):
        self.id = ""

class LaneWidth:
    def __init__(self):
        self.sOffset = ""
        self.a = ""
        self.b = ""
        self.c = ""
        self.d = ""

class RoadMark:
    def __init__(self):
        self.sOffset = ""
        self.type = ""
        self.material = ""
        self.color = ""
        self.width = ""
        self.laneChange = ""

class VectorLane:
    def __init__(self):
        self.sOffset = ""
        self.laneId = ""
        self.traverDir = ""

class Road:
    def __init__(self, name='', length='', id='', junction='', link=RoadLink(), type=RoadType(), planView=[], elevationProfile=[], lanes=Lanes(), objects=[], signals=Signals()):
        self.name = name
        self.length = length
        self.id = id
        self.junction = junction
        self.link = link
        self.type = type
        self.planView = planView
        self.elevationProfile = elevationProfile
        self.lanes = lanes
        self.objects = objects
        self.signals = signals


### LANES ###
#############



### OBJECTS ###
###############

class Object:
    def __init__(self):
        self.id = ""
        self.name = ""
        self.s = ""
        self.t = ""
        self.zOffset = ""
        self.hdg = ""
        self.roll = ""
        self.pitch = ""
        self.orientation = ""
        self.type = ""
        self.height = ""
        self.width = ""
        self.length = ""
        self.outline = [] #CornerLocal()

class CornerLocal:
    def __init__(self):
        self.u = ""
        self.v = ""
        self.z = ""

### OBJECTS ###
###############






### CONTROLLERS ###
###################

class Controller:
    def __init__(self):
        self.name = ""
        self.id = ""
        self.sequence = ""
        self.control = [] #ControlSignal()

class ControlSignal:
    def __init__(self):
        self.signalId = ""
        self.type = ""

### CONTROLLERS ###
###################


### JUNCTIONS ###
#################

class Junction:
    def __init__(self):
        self.id = ""
        self.name = ""
        self.connection = [] #Connection()
        self.controller = [] #ControllerJunction()
        self.userData = VectorJunction()

class Connection:
    def __init__(self):
        self.id = ""
        self.incomingRoad = ""
        self.connectingRoad = ""
        self.contactPoint = ""
        self.laneLink = [] #LaneLinkJunction()

class LaneLinkJunction:
    def __init__(self):
        self.fromLane = ""
        self.toLane = ""

class ControllerJunction:
    def __init__(self):
        self.id = ""
        self.type = ""
        self.sequence = ""

class VectorJunction:
    def __init__(self):
        self.junctionId = ""

### JUNCTIONS ###
#################

### CARLA EQUIVALENT CLASSES ###
################################

class Transform:
    def __init__(self):
        self.location = Location()
        self.rotation = Rotation()

class Location:
    def __init__(self):
        self.x = ""
        self.y = ""
        self.z = ""

class Rotation:
    def __init__(self):
        self.pitch = ""
        self.yaw = ""
        self.roll = ""

################################
### CARLA EQUIVALENT CLASSES ###

import math
import numpy as np
from scipy.spatial import KDTree

from .builder_classes import Transform, Location, Rotation

class Waypoint:
    """
    For initializing a waypoint in a specific position, the location
    parameter must be passed in Location format. If not, other option
    is not passing any parameter an set the parameters after initializing the
    Waypoint object.
    """
    def __init__(self, location=Location("", "", ""), 
                       rotation=Rotation("", "", "")):
        self.id = "" # (int)
        self.transform = Transform(location, rotation)
        self.road_id = "" # (int)
        self.section_id = "" # (int)
        self.lane_id = "" # (int)
        self.s = "" # (float - meters)
        self.junction = "" # junction's ID, if -1 it is not junction (int)
        self.lane_width = "" # (float)
        self.lane_change = "" #can be none, right, left or both (string)
        self.lane_type = "" # (string)
        self.right_lane_marking = ""
        self.left_lane_marking = ""
        self.vmax = ""
        self.vunit = ""
        self.n_lanes = ""         # Number of lanes in same direction
        self.lane_position = ""   # Position of the current lane,
                                # starting from 1 to the right

    def get_closer_wp(self, waypoint_list):
        """
        Return closer wp given a wp list
        It can be usefull to get road and lane info of the self waypoint
        """
        closer_distance = 10000 # High arbitrary value
        for wp in waypoint_list:
            distance = math.sqrt((wp.transform.location.x-self.transform.location.x)**2 + 
                                 (wp.transform.location.y-self.transform.location.y)**2 +
                                 (wp.transform.location.z-self.transform.location.z)**2)
            if distance < closer_distance:
                closer_distance = distance
                closer_wp = wp
        return closer_wp

    def get_closer_right_wp(self, waypoint_list, kdtree):
        """
        Calculates closer right waypoint only considering it it is in a 
        different road or lane

        Args:
            self
            waypoint_list: (list)
            kdtree: (scipy.spatial.kdtree.KDTree)

        Returns:
            closer_right_wp
        """
        k = -1
        if self.lane_id < 0: k = 1

        alpha_radians = math.radians(self.transform.rotation.yaw)
        x = self.transform.location.x + math.cos(alpha_radians)*self.lane_width*(-k)
        y = self.transform.location.y - math.sin(alpha_radians)*self.lane_width*k
        z = self.transform.location.z

        current_location_array = np.array((x, y, z))
        closer_dist, closer_point = kdtree.query(
            current_location_array, 1)
        right_waypoint = waypoint_list[closer_point.numerator]
        return right_waypoint

    def get_closer_left_wp(self, waypoint_list, kdtree):
        """
        Calculates closer left waypoint only considering it it is in a 
        different road or lane

        Args:
            self
            waypoint_list: (list)
            kdtree: (scipy.spatial.kdtree.KDTree)

        Returns:
            closer_left_wp
        """
        k = -1
        if self.lane_id < 0: k = 1

        alpha_radians = math.radians(self.transform.rotation.yaw)
        x = self.transform.location.x - math.cos(alpha_radians)*self.lane_width*(-k)
        y = self.transform.location.y + math.sin(alpha_radians)*self.lane_width*k
        z = self.transform.location.z

        current_location_array = np.array((x, y, z))
        closer_dist, closer_point = kdtree.query(
            current_location_array, 1)
        left_waypoint = waypoint_list[closer_point.numerator]
        return left_waypoint

    def distance(self, waypoint):
        """
        Calculate distance from current waypoint to other waypoint

        Args:
            waypoint: Goal waypoint to compute distance from current

        Returns:
            distance: (float) euclidean distance
        """
        distance = math.sqrt(
            (waypoint.transform.location.x-self.transform.location.x)**2 + 
            (waypoint.transform.location.y-self.transform.location.y)**2)

        return distance

    def get_lane_position(self, lane_id, road):
        """
        Returns the number of lanes in the current road with same direction
        and the position of the current lane, starting to count from 1 from 
        the right to the left

        Args:
            lane_id: (int) Id of the current lane
            road: (Road) Road containing the current lane

        Returns:
            n_lanes: (int) Number of lanes
            lane_position: (int) Position of the current lane
        """

        n_lanes = 0
        lane_position = 0

        if lane_id < 0:
            for lane in road.lanes.laneSections[0].right:
                if lane.type == "driving":
                    n_lanes += 1
                if lane.id == lane_id:
                    lane_position = n_lanes
        elif lane_id > 0:
            for lane in road.lanes.laneSections[0].left:
                if lane.type == "driving":
                    n_lanes += 1
                if lane.id == lane_id:
                    lane_position = n_lanes

        return n_lanes, lane_position
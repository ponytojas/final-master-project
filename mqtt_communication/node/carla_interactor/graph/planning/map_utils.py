import math

from .waypoint import Waypoint
from .builder_classes import Transform, Location

def ref_line_pose_2_global_pose(road, s, t, zOffset, hOffset):
    """
    """
    pose = Transform()
    index = 0

    # Locate in which geometry of the road is the s position
    for i in range(0, len(road.planView)):
        if s > road.planView[i].s:
            index = i
        else:
            break

    pose.location.x = road.planView[index].x
    pose.location.y = road.planView[index].y
    heading = road.planView[index].hdg

    # Apply 'remaining s' traslation
    remaining_s = s - road.planView[index].s
    if (road.planView[index].type == "line"):
        pose.location = get_point_in_line (pose.location.x, pose.location.y, 0, remaining_s, heading)

    elif (road.planView[index].type == "arc"):
        pose.location = get_point_in_arc (pose.location.x, pose.location.y, 0, remaining_s, road.planView[index].curvature, heading)
        ### The most of the landmarks locate in a arc are rotated in CARLA, so we are going to recalculate the heading
        ### If remaining_s is close to start/end of the geometry, we use the first/last heading of the geometry
        if (remaining_s < road.planView[index].length*0.3):
            pass ## Not change heading
        elif (remaining_s > road.planView[index].length*0.7):
            heading += road.planView[index].length*road.planView[index].curvature
        else:
            heading += remaining_s*road.planView[index].curvature

    # Apply 't' traslation
    pose.location = get_point_in_line (pose.location.x, pose.location.y, 0, t, (heading + (math.pi/2)))

    # Apply 'zOffset' traslation
    pose.location.z = zOffset

    # Apply 'hOffset' rotation
    pose.rotation.yaw = math.degrees(heading + hOffset)
    pose.rotation.roll = 0.0
    pose.rotation.pitch = 0.0

    return pose

def get_point_in_line(x,y,z,s,heading):
    """
    Get a Location into a line geometry given an 's' distance and initial pose (x, y , z, heading)
    """
    location = Location(x, y, z)

    location.x += (s * math.cos(heading))
    location.y += (s * math.sin(heading))

    return location

def get_point_in_arc(x,y,z,s,curvature,heading):
    """
    Get a Location into an arc geometry given an 's' distance  and initial pose (x, y , z, heading)
    """
    location = Location(x, y, z)
    radius = 1.0 / curvature

    location.x -= radius * math.sin(heading)
    location.y += radius * math.cos(heading)
    heading += s*curvature
    location.x += radius * math.sin(heading)
    location.y -= radius * math.cos(heading)

    return location

def get_road(roads, road_id):
    """
    Get a road from a map.roads by its id

    Args:
        roads: (list) List of Road()
        road_id: (int) 

    Returns:
        road: (Road()) Road of the given id
    """
    for road in roads:
        if road.id == road_id:
            return road
    print("get_road Error: Road not found!")

def get_lane(road, lane_id, laneSectionValue):
    """
    Get a lane from a road by its id

    Args:
        road: (Road) road containing the lane
        lane_id: (int)
        laneSectionValue: (int) or (str) 'unknown'

    Returns:
        lane: (Lane)
    """
    if laneSectionValue == 'unknown':
        if lane_id < 0:
            laneSection = 0
        elif lane_id > 0:
            laneSection = -1
    else:
        laneSection = laneSectionValue

    for lane in (road.lanes.laneSections[laneSection].right +
                 road.lanes.laneSections[laneSection].left):
        if lane.id == lane_id:
            return lane

def get_junction(junctions, junction_id):
    """
    Get a junction from a map.junctions by its id

    Args:
        junctions: (list) List of Junction()
        junction_id: (int)

    Returns:
        junction: (Junction()) Junction of the given id
    """
    for junction in junctions:
        if junction.id == junction_id:
            return junction
    print("get_junction Error: Junction not found!")

def get_direction(road):
    """
    Get driving direction of the road, positive or negative,
    according to the predecessor --> successor sequence

    Args:
        road: (Road())

    Returns:
        (str) '+' or '-' defining driving direction
    """
    if (len(road.lanes.laneSections[0].right) > 0):
        for lane in road.lanes.laneSections[0].right:
            if (lane.type == "driving" and lane.id < 0):
                return "+"
    if (len(road.lanes.laneSections[0].left) > 0):
        for lane in road.lanes.laneSections[0].left:
            if (lane.type == "driving" and lane.id > 0):
                return "-"

def xyz_to_roadlane(x, y, z, waypoint_list):
    """
    Receives a location an get its corresponding road and lane, because global
    path planning need road/lane tuple as input. If the location is not inside
    a lane/road, returns the closer one inside a lane/road.

    Args:
        x: (int)
        y: (int)
        z: (int)
        waypoint_list: (list) list of every waypoint generated by the map bulder
            centered at every lane

    Returns:
        closer_roadlane: (tuple) Closer road/lane to the given location
    """

    waypoint = Waypoint()
    waypoint.transform.location.x = x
    waypoint.transform.location.y = y
    waypoint.transform.location.z = z


    closer_waypoint = waypoint.get_closer_wp(waypoint_list)
    closer_roadlane = (int(closer_waypoint.road_id), int(closer_waypoint.lane_id))

    return closer_roadlane

def get_initial_position(road, lane, side):
    """
    Returns initial position of the road centered at the lane

    Args:
        road: (Road())
        lane: (Lane())
        side: (str) left or right

    Returns:
        location: (Location())
    """
    # First get offset from the center of the lane to the center of the road
    lane_width_offset = 0
    if (side == 'left'):
        for i in range(0, abs(lane.id)):
            # -1-i because the list must be checked inside out
            lane_width_offset += road.lanes.laneSections[0].left[-1-i].width[0].a
        # total lane offset for lane origin
        lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
    elif (side == 'right'):
        for i in range(0, abs(lane.id)):
            lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
        # total lane offset for lane origin
        lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

    location = get_point_in_line(road.planView[0].x,
                                 road.planView[0].y,
                                 0,
                                 lane_offset,
                                 road.planView[0].hdg + math.pi/2)
    location.x = round(location.x, 3)
    location.y = round(location.y, 3)
    location.z = round(location.z, 3)

    return location

def get_inverted_initial_position(road, lane, side):
    """
    Return last position of the road. It can be used to get the origin
    position for inverted roads

    Args:
        road: (Road())
        lane: (Lane())
        side: (str) left or right

    Returns:
        location: (Location())
    """
    # First get offset from the center of the lane to the center of the road
    lane_width_offset = 0
    if (side == 'left'):
        for i in range(0, abs(lane.id)):
            # -1-i because the list must be checked inside out
            lane_width_offset += road.lanes.laneSections[0].left[-1-i].width[0].a
        # total lane offset for lane origin
        lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
    elif (side == 'right'):
        for i in range(0, abs(lane.id)):
            lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
        # total lane offset for lane origin
        lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

    # The get location centered on the road
    if (road.planView[-1].type == "line"):
        location_aux = get_point_in_line(road.planView[-1].x,
                                     road.planView[-1].y,
                                     0,
                                     road.planView[-1].length,
                                     road.planView[-1].hdg,
                                     )
    elif (road.planView[-1].type == "arc"):
        location_aux = get_point_in_arc(road.planView[-1].x,
                                    road.planView[-1].y,
                                    0,
                                    road.planView[-1].length,
                                    road.planView[-1].curvature,
                                    road.planView[-1].hdg,
                                    )
    # Apply offset
    location = get_point_in_line(location_aux.x,
                                 location_aux.y,
                                 0,
                                 lane_offset,
                                 road.planView[-1].hdg + math.pi/2)

    # Round value 
    location.x = round(location.x, 3)
    location.y = round(location.y, 3)
    location.z = round(location.z, 3)

    return location

def get_lane_action(current, next):
    """
    Gets type of action to do between current (road, lane) and next 
    (road, lane). Type of action can be:
        - lanefollow
        - changeright
        - changeleft

    Args:
        current: (tuple) Current (road, lane)
        next: (tuple) Next (road, lane)

    Returns:
        action: (str) Type of action to do 
    """
    # If current road == next road, but lanes are different, means lanechange

    # Case of lanechange and '-' road direction
    if (current[0] == next[0] and current[1] > 0):
        if (current[1] > next[1]):
            action = "changeleft"
        else:
            action = "changeright"

    # Case of lanechange and '+' road direction
    elif (current[0] == next[0] and current[1] < 0):
        if (current[1] > next[1]):
            action = "changeright"
        else:
            action = "changeleft"

    # Case of lanefollow
    else:
        action = "lanefollow"

    return action

def get_lane_change(road, lane_id):
    """
    Returns if lane change is possible for this lane, depending on
    the roadmark and if the left/right lane exists. 

    Values for lane change are:
        1) None
        2) Left
        3) Right
        4) Both

    Args:
        road    : (Road)
        lane_id : (int)

    Returns:
        lane_change: (str)

    Summary of How LaneChange works in XODR standard:
    Whatever the travel direction of the current lane, the roadMark that must
    be checked is:
        - right lane change : current lane id      (current_lane)
        - left lane change  : current lane id - 1  (decrement_lane)
    """

    current_lane = get_lane(road, lane_id, 0)
    decrement_lane = get_lane(road, lane_id - 1, 0)
    increment_lane =  get_lane(road, lane_id + 1, 0)
    # Must check all the roadmark segments, for possible bug 
    # in xodr definition
    incrementLane_laneChange = "none"
    decrementLane_laneChange = "none"
    currentLane_laneChange = "none"
    if current_lane is not None and current_lane.type == "driving":
        for roadMark in current_lane.roadMark:          
            if roadMark.laneChange == "both":
                currentLane_laneChange = "both"
                break

            elif roadMark.laneChange == "increase":
                currentLane_laneChange = "increase"
                break

            elif roadMark.laneChange == "decrease":
                currentLane_laneChange = "decrease"
                break
    
    if decrement_lane is not None and decrement_lane.type == "driving":
        for roadMark in decrement_lane.roadMark:
            if roadMark.laneChange == "both":
                decrementLane_laneChange = "both"
                break

            elif roadMark.laneChange == "increase":
                decrementLane_laneChange = "increase"
                break

            elif roadMark.laneChange == "decrease":
                decrementLane_laneChange = "decrease"
                break

    if increment_lane is not None and increment_lane.type == "driving":
        for roadMark in increment_lane.roadMark:
            if roadMark.laneChange == "both":
                incrementLane_laneChange = "both"
                break

            elif roadMark.laneChange == "increase":
                incrementLane_laneChange = "increase"
                break

            elif roadMark.laneChange == "decrease":
                incrementLane_laneChange = "decrease"
                break

    # Get the lane change
    # For positive travel direction
    if lane_id < 0:
        if ((incrementLane_laneChange == "increase" or incrementLane_laneChange == "both") and
            (currentLane_laneChange == "increase" or currentLane_laneChange == "none")):
            return "left"

        elif ((incrementLane_laneChange == "decrease" or incrementLane_laneChange == "none") and
              (currentLane_laneChange == "decrease" or currentLane_laneChange == "both")):
            return "right"

        elif ((incrementLane_laneChange == "increase" or incrementLane_laneChange == "both") and
              (currentLane_laneChange == "decrease" or currentLane_laneChange == "both")):
            return "both"

        else:
            return "none"

    # For negative traver direction
    elif lane_id > 0:
        if ((currentLane_laneChange == "increase" or currentLane_laneChange == "both") and
            (decrementLane_laneChange == "increase" or decrementLane_laneChange == "none")):
            return "right"

        elif ((currentLane_laneChange == "decrease" or currentLane_laneChange == "none") and
              (decrementLane_laneChange == "decrease" or decrementLane_laneChange == "both")):
            return "left"

        elif ((currentLane_laneChange == "increase" or currentLane_laneChange == "both") and
              (decrementLane_laneChange == "decrease" or decrementLane_laneChange == "both")):
            return "both"

        else:
            return "none"

def get_topology_waypoints(roads):
    """
    Return a list of pairs of wp [[wp0,wp1],[wp2,wp3],...,[wpn, wpn+1]] with in and out wp for every lane centered on the lane
    """
    topology_tuples = []
    for road in roads:
        # Get left lanes info
        if (road.lanes.laneSections[0].left):
            for lane in road.lanes.laneSections[0].left:
                if (lane.type == "driving"):
                    waypoint_tuple = []
                    # lane width offset for lane origin
                    lane_width_offset = 0
                    for i in range(0, abs(lane.id)):
                        lane_width_offset += road.lanes.laneSections[0].left[-1-i].width[0].a # -1-i because the list must be checked inside out
                    # total lane offset for lane origin
                    lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
                    # lane origin center
                    waypoint_lane_origin_center = Waypoint()
                    waypoint_lane_origin_center.transform.location = get_point_in_line(road.planView[0].x, road.planView[0].y, 0, lane_offset, road.planView[0].hdg+math.pi/2)
                    waypoint_lane_origin_center.transform.location.z = calculate_elevation(road.elevationProfile[0].a, road.elevationProfile[0].b, road.elevationProfile[0].c, road.elevationProfile[0].d, road.elevationProfile[0].s, 0)
                    waypoint_lane_origin_center.road_id = road.id
                    waypoint_lane_origin_center.lane_id = lane.id
                    waypoint_tuple.append(waypoint_lane_origin_center)
                    # lane end center   
                                                    
                    if (road.planView[-1].type == "line"):                             
                        waypoint_lane_end_center = Waypoint()
                        aux_location = get_point_in_line(road.planView[-1].x, road.planView[-1].y, 0, lane_offset, road.planView[-1].hdg+math.pi/2)
                        waypoint_lane_end_center.transform.location = get_point_in_line(aux_location.x, aux_location.y, 0, road.planView[-1].length, road.planView[-1].hdg)
                        waypoint_lane_end_center.transform.location.z = calculate_elevation(road.elevationProfile[-1].a, road.elevationProfile[-1].b, road.elevationProfile[-1].c, road.elevationProfile[-1].d, road.elevationProfile[-1].s, road.planView[-1].length)
                        waypoint_lane_end_center.road_id = road.id
                        waypoint_lane_end_center.lane_id = lane.id
                        waypoint_tuple.insert(0, waypoint_lane_end_center)
                    
                    elif (road.planView[-1].type == "arc"):
                        radius = 1/road.planView[-1].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[-1].length/radius
                        new_length = k_arc*new_radius
                        
                        waypoint_lane_end_center = Waypoint()
                        aux_location = get_point_in_line(road.planView[-1].x, road.planView[-1].y, 0, lane_offset, road.planView[-1].hdg+math.pi/2)
                        waypoint_lane_end_center.transform.location = get_point_in_arc(aux_location.x, aux_location.y, 0, new_length, new_curvature, road.planView[-1].hdg)
                        waypoint_lane_end_center.transform.location.z = calculate_elevation(road.elevationProfile[-1].a, road.elevationProfile[-1].b, road.elevationProfile[-1].c, road.elevationProfile[-1].d, road.elevationProfile[-1].s, road.planView[-1].length)
                        waypoint_lane_end_center.road_id = road.id
                        waypoint_lane_end_center.lane_id = lane.id
                        waypoint_tuple.insert(0, waypoint_lane_end_center)
                    topology_tuples.append(waypoint_tuple)
                    
        # Get right lanes info
        if (road.lanes.laneSections[0].right):
            for lane in road.lanes.laneSections[0].right:
                if (lane.type == "driving"):
                    waypoint_tuple = []
                    # lane width offset for lane origin
                    lane_width_offset = 0
                    for i in range(0, abs(lane.id)):
                        lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
                    # total lane offset for lane origin
                    lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a
                    # lane origin center
                    waypoint_lane_origin_center = Waypoint()
                    waypoint_lane_origin_center.transform.location = get_point_in_line(road.planView[0].x, road.planView[0].y, 0, lane_offset, road.planView[0].hdg+math.pi/2)
                    waypoint_lane_origin_center.transform.location.z = calculate_elevation(road.elevationProfile[0].a, road.elevationProfile[0].b, road.elevationProfile[0].c, road.elevationProfile[0].d, road.elevationProfile[0].s, 0)
                    waypoint_lane_origin_center.road_id = road.id
                    waypoint_lane_origin_center.lane_id = lane.id
                    waypoint_tuple.append(waypoint_lane_origin_center)
                    # lane end center  
                                                    
                    if (road.planView[-1].type == "line"):                             
                        waypoint_lane_end_center = Waypoint()
                        aux_location = get_point_in_line(road.planView[-1].x, road.planView[-1].y, 0, lane_offset, road.planView[-1].hdg+math.pi/2)
                        waypoint_lane_end_center.transform.location = get_point_in_line(aux_location.x, aux_location.y, 0, road.planView[-1].length, road.planView[-1].hdg)
                        waypoint_lane_end_center.transform.location.z = calculate_elevation(road.elevationProfile[-1].a, road.elevationProfile[-1].b, road.elevationProfile[-1].c, road.elevationProfile[-1].d, road.elevationProfile[-1].s, road.planView[-1].length)
                        waypoint_lane_end_center.road_id = road.id
                        waypoint_lane_end_center.lane_id = lane.id
                        waypoint_tuple.append(waypoint_lane_end_center)
                    
                    elif (road.planView[-1].type == "arc"):
                        radius = 1/road.planView[-1].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[-1].length/radius
                        new_length = k_arc*new_radius
                        
                        waypoint_lane_end_center = Waypoint()
                        aux_location = get_point_in_line(road.planView[-1].x, road.planView[-1].y, 0, lane_offset, road.planView[-1].hdg+math.pi/2)
                        waypoint_lane_end_center.transform.location = get_point_in_arc(aux_location.x, aux_location.y, 0, new_length, new_curvature, road.planView[-1].hdg)
                        waypoint_lane_end_center.transform.location.z = calculate_elevation(road.elevationProfile[-1].a, road.elevationProfile[-1].b, road.elevationProfile[-1].c, road.elevationProfile[-1].d, road.elevationProfile[-1].s, road.planView[-1].length)
                        waypoint_lane_end_center.road_id = road.id
                        waypoint_lane_end_center.lane_id = lane.id
                        waypoint_tuple.append(waypoint_lane_end_center)
                    topology_tuples.append(waypoint_tuple)
                    
    return topology_tuples

def generate_waypoints(roads, distance):
    """
    Generate waypoints centered in every driving lane given a distance
    """
    # @ADD 07/11/20 : Fix elevation (z inertial)
    waypoints = []
    for road in roads:
        # Get left lanes info
        if (road.lanes.laneSections[-1].left):
            for lane in road.lanes.laneSections[-1].left:
                if (lane.type == "driving"):
                    # lane width offset for lane origin
                    lane_width_offset = 0
                    for i in range(0, abs(lane.id)):
                        lane_width_offset += road.lanes.laneSections[-1].left[-1-i].width[-1].a # -1-i because the list must be checked inside out
                    # total lane offset for lane origin
                    lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[-1].a
                    # For each lane go through every geometry generating waypoints
                    for geometry in road.planView:
                        # lane origin center
                        waypoint_geometry_origin_lane_center = Waypoint()
                        waypoint_geometry_origin_lane_center.s = geometry.s
                        waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(geometry.x, geometry.y, 0, lane_offset, geometry.hdg+math.pi/2)
                        waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint_geometry_origin_lane_center.s)
                        waypoint_geometry_origin_lane_center.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                        waypoint_geometry_origin_lane_center.lane_width = lane.width[0].a
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        waypoint_geometry_origin_lane_center.vmax = road.type.speed.max
                        waypoint_geometry_origin_lane_center.vunit = road.type.speed.unit
                        (waypoint_geometry_origin_lane_center.n_lanes, 
                        waypoint_geometry_origin_lane_center.lane_position) = (
                            waypoint_geometry_origin_lane_center.get_lane_position(lane.id, road))
                        waypoint_geometry_origin_lane_center.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint_geometry_origin_lane_center)
                        
                        if (geometry.type == "line"):   
                            n = int(geometry.length/distance)                           
                            for n_dist in range(0,n):
                                waypoint = Waypoint()
                                waypoint.s = geometry.s + n_dist*distance
                                waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, geometry.hdg)
                                waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                                waypoint.lane_change = get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            waypoint = Waypoint()
                            waypoint.s = geometry.s + geometry.length
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, geometry.length, geometry.hdg)
                            waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                           
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        elif (geometry.type == "arc"):
                            radius = 1/geometry.curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = geometry.length/radius
                            new_length = k_arc*new_radius
                            n = int(new_length/distance)  
                            for n_dist in range(0,n):
                                waypoint = Waypoint()
                                waypoint.s = geometry.s + n_dist*distance
                                waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, geometry.hdg)
                                waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (geometry.hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                                waypoint.lane_change = get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            waypoint = Waypoint()
                            waypoint.s = geometry.s + new_length
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, geometry.hdg)
                            waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (geometry.hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                          
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
        # Get right lanes info
        if (road.lanes.laneSections[0].right):
            for lane in road.lanes.laneSections[0].right:
                if (lane.type == "driving"):
                    # lane width offset for lane origin
                    lane_width_offset = 0
                    for i in range(0, abs(lane.id)):
                        lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
                    # total lane offset for lane origin
                    lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a
                    # For each lane go through every geometry generating waypoints
                    for geometry in road.planView:
                        n = int(geometry.length/distance)
                        # lane origin center
                        waypoint_geometry_origin_lane_center = Waypoint()
                        waypoint_geometry_origin_lane_center.s = geometry.s
                        waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(geometry.x, geometry.y, 0, lane_offset, geometry.hdg+math.pi/2)
                        waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint_geometry_origin_lane_center.s)
                        waypoint_geometry_origin_lane_center.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                        waypoint_geometry_origin_lane_center.lane_width = lane.width[0].a
                        waypoint_geometry_origin_lane_center.road_id = road.id
                        waypoint_geometry_origin_lane_center.lane_id = lane.id                       
                        waypoint_geometry_origin_lane_center.junction = road.junction
                        waypoint_geometry_origin_lane_center.vmax = road.type.speed.max
                        waypoint_geometry_origin_lane_center.vunit = road.type.speed.unit
                        (waypoint_geometry_origin_lane_center.n_lanes, 
                        waypoint_geometry_origin_lane_center.lane_position) = (
                            waypoint_geometry_origin_lane_center.get_lane_position(lane.id, road))
                        waypoint_geometry_origin_lane_center.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint_geometry_origin_lane_center)
                        
                        if (geometry.type == "line"): 
                            n = int(geometry.length/distance)                             
                            for n_dist in range(0,n):
                                waypoint = Waypoint()
                                waypoint.s = geometry.s + n_dist*distance
                                waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, geometry.hdg)
                                waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                                
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                                waypoint.lane_change = get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            waypoint = Waypoint()
                            waypoint.s = geometry.s + geometry.length
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, geometry.length, geometry.hdg)
                            waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id                           
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        
                        elif (geometry.type == "arc"):
                            radius = 1/geometry.curvature
                            new_radius = radius - lane_offset
                            new_curvature = 1/new_radius
                            k_arc = geometry.length/radius
                            new_length = k_arc*new_radius
                            n = int(new_length/distance)
                            for n_dist in range(0,n):
                                waypoint = Waypoint()
                                waypoint.s = geometry.s + n_dist*distance
                                waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, geometry.hdg)
                                waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint.s)
                                waypoint.transform.rotation.yaw = (geometry.hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                                waypoint.lane_width = lane.width[0].a
                                waypoint.road_id = road.id
                                waypoint.lane_id = lane.id                               
                                waypoint.junction = road.junction
                                waypoint.vmax = road.type.speed.max
                                waypoint.vunit = road.type.speed.unit
                                waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                                waypoint.lane_change = get_lane_change(road, lane.id)
                                waypoints.append(waypoint)
                            
                            waypoint = Waypoint()
                            waypoint.s = geometry.s + new_length
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, geometry.hdg)
                            waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, waypoint.s)
                            waypoint.transform.rotation.yaw = (geometry.hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
    return waypoints

def generate_waypoints_in_lane(road, lane, distance):
    """
    Generate waypoints centered in a driving lane given a distance
    """
    waypoints = []
    if lane.id > 0: #left side --> road s reference line is inverted
        if (lane.type == "driving"):
            # lane width offset for lane origin
            lane_width_offset = 0
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[-1].left[-1-i].width[0].a # -1-i because the list must be checked inside out
            # total lane offset for lane origin
            if len(road.lanes.laneSections) > 1:
                lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[-1].a
            elif len(road.lanes.laneSections) == 1:
                lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
            # For each lane go through every geometry generating waypoints
            for geometry in reversed(road.planView):
                # lane origin center
                waypoint_geometry_origin_lane_center = Waypoint()
                waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(geometry.x, geometry.y, 0, lane_offset, geometry.hdg+math.pi/2)
                #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                waypoint_geometry_origin_lane_center.road_id = road.id
                waypoint_geometry_origin_lane_center.lane_id = lane.id
                waypoint_geometry_origin_lane_center.s = geometry.s
                waypoint_geometry_origin_lane_center.junction = road.junction
                #waypoints.append(waypoint_geometry_origin_lane_center)
                
                if (geometry.type == "line"):  
                    waypoint = Waypoint()
                    waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, geometry.length, geometry.hdg)
                    #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, geometry.length)
                    waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                    waypoint.lane_width = lane.width[0].a
                    waypoint.road_id = road.id
                    waypoint.lane_id = lane.id
                    waypoint.s = geometry.s + geometry.length
                    waypoint.junction = road.junction
                    waypoint.vmax = road.type.speed.max
                    waypoint.vunit = road.type.speed.unit
                    waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                    waypoint.lane_change = get_lane_change(road, lane.id)
                    waypoints.append(waypoint) 
                    n = int(geometry.length/distance)                           
                    for n_dist in range(n,-1,-1):
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, geometry.hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                        waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = geometry.s + n_dist*distance
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                    
                
                elif (geometry.type == "arc"):
                    radius = 1/geometry.curvature
                    new_radius = radius - lane_offset
                    new_curvature = 1/new_radius
                    k_arc = geometry.length/radius
                    new_length = k_arc*new_radius
                    waypoint = Waypoint()
                    waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, geometry.hdg)
                    #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                    waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                    waypoint.lane_width = lane.width[0].a
                    waypoint.road_id = road.id
                    waypoint.lane_id = lane.id
                    waypoint.s = geometry.s + new_length
                    waypoint.junction = road.junction
                    waypoint.vmax = road.type.speed.max
                    waypoint.vunit = road.type.speed.unit
                    waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                    waypoint.lane_change = get_lane_change(road, lane.id)
                    waypoints.append(waypoint)
                    n = int(new_length/distance)  
                    for n_dist in range(n,-1,-1):
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, geometry.hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                        waypoint.transform.rotation.yaw = (geometry.hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = geometry.s + n_dist*distance
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                    
            return waypoints

    elif lane.id < 0: #right side
        if (lane.type == "driving"):
            # lane width offset for lane origin
            lane_width_offset = 0
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
            # total lane offset for lane origin
            lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a
            # For each lane go through every geometry generating waypoints
            for geometry in road.planView:
                n = int(geometry.length/distance)
                # lane origin center
                waypoint_geometry_origin_lane_center = Waypoint()
                waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(geometry.x, geometry.y, 0, lane_offset, geometry.hdg+math.pi/2)
                #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                waypoint_geometry_origin_lane_center.road_id = road.id
                waypoint_geometry_origin_lane_center.lane_id = lane.id
                waypoint_geometry_origin_lane_center.s = geometry.s
                waypoint_geometry_origin_lane_center.junction = road.junction
                #waypoints.append(waypoint_geometry_origin_lane_center)
                
                if (geometry.type == "line"): 
                    n = int(geometry.length/distance)                             
                    for n_dist in range(0,n):
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, geometry.hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                        waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = geometry.s + n_dist*distance
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                    waypoint = Waypoint()
                    waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, geometry.length, geometry.hdg)
                    #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, geometry.length)
                    waypoint.transform.rotation.yaw = (geometry.hdg) * (180 / math.pi) + 90
                    waypoint.lane_width = lane.width[0].a
                    waypoint.road_id = road.id
                    waypoint.lane_id = lane.id
                    waypoint.s = geometry.s + geometry.length
                    waypoint.junction = road.junction
                    waypoint.vmax = road.type.speed.max
                    waypoint.vunit = road.type.speed.unit
                    waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                    waypoint.lane_change = get_lane_change(road, lane.id)
                    waypoints.append(waypoint)
                
                
                elif (geometry.type == "arc"):
                    radius = 1/geometry.curvature
                    new_radius = radius - lane_offset
                    new_curvature = 1/new_radius
                    k_arc = geometry.length/radius
                    new_length = k_arc*new_radius
                    n = int(new_length/distance)
                    for n_dist in range(0,n):
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, geometry.hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                        waypoint.transform.rotation.yaw = (geometry.hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = geometry.s + n_dist*distance
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                    
                    waypoint = Waypoint()
                    waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, geometry.hdg)
                    #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                    waypoint.transform.rotation.yaw = (geometry.hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                    waypoint.lane_width = lane.width[0].a
                    waypoint.road_id = road.id
                    waypoint.lane_id = lane.id
                    waypoint.s = geometry.s + new_length
                    waypoint.junction = road.junction
                    waypoint.vmax = road.type.speed.max
                    waypoint.vunit = road.type.speed.unit
                    waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                    waypoint.lane_change = get_lane_change(road, lane.id)
                    waypoints.append(waypoint)
            return waypoints

def generate_next_waypoints_until_lane_end(road, lane, distance, s_0):
    """
    Generate waypoints centered in a driving lane given a distance, from s_0 to end
    """
    waypoints = []
    # If left side, next wps have lower s (s is inverted)
    if lane.id > 0: #left side
        if (lane.type == "driving"):
            # lane width offset for lane origin
            lane_width_offset = 0
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[-1].left[-1-i].width[0].a # -1-i because the list must be checked inside out
            # total lane offset for lane origin
            if len(road.lanes.laneSections) > 1:
                lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[-1].a
            elif len(road.lanes.laneSections) == 1:
                lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a
            
            # First locate in which geometry is s_0
            geometry_index = 0
            for i in range(0, len(road.planView)):
                if s_0 > road.planView[i].s:
                    geometry_index = i
                else:
                    break
            # Then generate every wp from this s_0 in this geometry_index to origin (because in left side s is inverted)
            for i in range(geometry_index, -1,-1):
                # For first geometry get only wps from s_0
                if i == geometry_index:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"):   
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0-road.planView[i].s, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = s_0 - road.planView[i].s
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                        n = int((s_0-road.planView[i].s)/distance)                           
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0-road.planView[i].s, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = s_0 - road.planView[i].s
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                        n = int((s_0-road.planView[i].s)/distance)
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        

                # For rest of geometries get every wp
                else:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"):   
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                        n = int(road.planView[i].length/distance)                           
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + new_length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                        n = int(new_length/distance)  
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
            return waypoints

    # If right side, next wps have higher s 
    elif lane.id < 0: #right side
        if (lane.type == "driving"):
            # lane width offset for lane origin
            lane_width_offset = 0
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
            # total lane offset for lane origin
            lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

            # First locate in which geometry is s_0
            geometry_index = 0
            for i in range(0, len(road.planView)):
                if s_0 > road.planView[i].s:
                    geometry_index = i
                else:
                    break

            # Then generate every wp from this s_0 in this geometry_index to end 
            for i in range(geometry_index, len(road.planView)):

                if i == geometry_index:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"): 

                        n = int((road.planView[i].length+road.planView[i].s-s_0)/distance)                             
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0-waypoint_geometry_origin_lane_center.s+(n_dist*distance), road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s_0 + (n_dist*distance)
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                    
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        n = int((new_length+road.planView[i].s-s_0)/distance)
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0-waypoint_geometry_origin_lane_center.s+(n_dist*distance), new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s_0 + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + new_length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)

                
                else:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"): 
                        n = int(road.planView[i].length/distance)                             
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                    
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        n = int(new_length/distance)
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + new_length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
            return waypoints

def generate_previous_waypoints_until_lane_start(road, lane, distance, s):
    """
    Generate waypoints centered in a driving lane given a distance, from start to s
    """
    waypoints = []
    # If left side, next wps have lower s (s is inverted)
    if lane.id > 0: #left side
        if (lane.type == "driving"):
            # lane width offset for lane origin
            lane_width_offset = 0
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[-1].left[-1-i].width[0].a # -1-i because the list must be checked inside out
            # total lane offset for lane origin
            if len(road.lanes.laneSections) > 1:
                lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[-1].a
            elif len(road.lanes.laneSections) == 1:
                lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a

            # First locate in which geometry is s
            geometry_index = 0
            for i in range(0, len(road.planView)):
                if s > road.planView[i].s:
                    geometry_index = i
                else:
                    break
            # Then generate every wp from this s in this geometry_index to end (because in left side s is inverted)
            for i in range(len(road.planView)-1, geometry_index-1, -1):
                # For first geometry get only wps from s
                if i == geometry_index:
                        # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"):   
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                        n = int((road.planView[i].s+road.planView[i].length-s)/distance)                           
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, (s-waypoint_geometry_origin_lane_center.s)+(n_dist*distance), road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                        n = int((road.planView[i].s+road.planView[i].length-s)/distance)  
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, (s-waypoint_geometry_origin_lane_center.s)+(n_dist*distance), new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        

                # For rest of geometries get every wp
                else:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"):   
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                        n = int(road.planView[i].length/distance)                           
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + new_length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                        n = int(new_length/distance)  
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
            return waypoints

    # If right side, next wps have higher s 
    elif lane.id < 0: #right side
        if (lane.type == "driving"):
            # lane width offset for lane origin
            lane_width_offset = 0
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
            # total lane offset for lane origin
            lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

            # First locate in which geometry is s
            geometry_index = 0
            for i in range(0, len(road.planView)):
                if s > road.planView[i].s:
                    geometry_index = i
                else:
                    break

            # Then generate every wp from origin to this s in geometry_index 
            for i in range(0, geometry_index+1):

                if i == geometry_index:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"): 

                        n = int((s-road.planView[i].s)/distance)                             
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + (n_dist*distance)
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n*distance, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + (n*distance)
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                    
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        n = int(s/distance)
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + (n_dist*distance)
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + s
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)

                
                else:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"): 
                        n = int(road.planView[i].length/distance)                             
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
                    
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        n = int(new_length/distance)
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.transform.rotation.yaw = (road.planView[i].hdg + (n_dist*distance) * new_curvature) * (180 / math.pi) + 90
                            waypoint.lane_width = lane.width[0].a
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoint.vmax = road.type.speed.max
                            waypoint.vunit = road.type.speed.unit
                            waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                            waypoint.lane_change = get_lane_change(road, lane.id)
                            waypoints.append(waypoint)
                        
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.transform.rotation.yaw = (road.planView[i].hdg + (new_length) * new_curvature) * (180 / math.pi) + 90
                        waypoint.lane_width = lane.width[0].a
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + new_length
                        waypoint.junction = road.junction
                        waypoint.vmax = road.type.speed.max
                        waypoint.vunit = road.type.speed.unit
                        waypoint.n_lanes, waypoint.lane_position = waypoint.get_lane_position(lane.id, road)
                        waypoint.lane_change = get_lane_change(road, lane.id)
                        waypoints.append(waypoint)
            return waypoints

def generate_waypoints_from_to(road, lane, distance, s_0, s):
    # @ADD 18/11/20 : Need to be done and tested
    """
    Generate waypoints centered in a driving lane given a distance, from s_0 to s
    *** This method is in developing, so is not properly working ***
    """
    waypoints = []
    # If left side, next wps have lower s (s is inverted)
    if lane.id > 0: #left side
        if (lane.type == "driving"):
            # lane width offset for lane origin
            lane_width_offset = 0
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[0].left[-1-i].width[0].a # -1-i because the list must be checked inside out
            # total lane offset for lane origin
            lane_offset = lane_width_offset - lane.width[0].a/2 + road.lanes.laneOffset[0].a

            # Locate in which geometry is s_0
            geometry_index_s_0 = 0
            for i in range(0, len(road.planView)):
                if s_0 > road.planView[i].s:
                    geometry_index_s_0 = i
                else:
                    break

            # Locate in which geometry is s
            geometry_index_s = 0
            for i in range(0, len(road.planView)):
                if s_0 > road.planView[i].s:
                    geometry_index_s = i
                else:
                    break

            # Then generate every wp from this s_0 in this geometry_index to s (because in left side s is inverted)
            for i in range(geometry_index_s_0, geometry_index_s,-1):
                # Case of first geometry get only wps from s_0
                if i == geometry_index_s_0:
                        # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"):   
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = s_0
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                        n = int(s_0/distance)                           
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = s_0
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                        n = int(s_0/distance)  
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                        
                # Case of last geometry get until s
                elif i == geometry_index_s:    
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"):   
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = s_0
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                        n = int(s_0/distance)                           
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = s_0
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                        n = int(s_0/distance)  
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)

                # case of rest of geometries get every wp
                else:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"):   
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                        n = int(road.planView[i].length/distance)                           
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + new_length
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                        n = int(new_length/distance)  
                        for n_dist in range(n,-1,-1):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
            return waypoints

    # If right side, next wps have higher s 
    elif lane.id < 0: #right side
        if (lane.type == "driving"):
            # lane width offset for lane origin
            lane_width_offset = 0
            for i in range(0, abs(lane.id)):
                lane_width_offset += road.lanes.laneSections[0].right[i].width[0].a
            # total lane offset for lane origin
            lane_offset = -lane_width_offset + lane.width[0].a/2 + road.lanes.laneOffset[0].a

            # Locate in which geometry is s_0
            for i in range(0, len(road.planView)):
                if s_0 > road.planView[i].s:
                    geometry_index_s_0 = i
                else:
                    break

            # Locate in which geometry is s
            for i in range(0, len(road.planView)):
                if s_0 > road.planView[i].s:
                    geometry_index_s = i
                else:
                    break

            # Then generate every wp from this s_0 in this geometry_index to s 
            for i in range(geometry_index_s_0, geometry_index_s):

                # Case of first geometry 
                if i == geometry_index_s_0:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"): 

                        n = int((road.planView[i].length-s_0)/distance)                             
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0+(n_dist*distance), road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s_0 + (n_dist*distance)
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                    
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        n = int((new_length-s_0)/distance)
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s_0+(n_dist*distance), new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = s_0 + n_dist*distance
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + new_length
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)

                # Case of last geometry 
                elif i == geometry_index_s:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"): 

                        n = int(s/distance)                             
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + (n_dist*distance)
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + s
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                    
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        n = int(s/distance)
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + (n_dist*distance)
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, s, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + s
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                
                # Case of any other geometry 
                else:
                    # lane origin center
                    waypoint_geometry_origin_lane_center = Waypoint()
                    waypoint_geometry_origin_lane_center.transform.location = get_point_in_line(road.planView[i].x, road.planView[i].y, 0, lane_offset, road.planView[i].hdg+math.pi/2)
                    #waypoint_geometry_origin_lane_center.transform.location.z = calculate_elevationProfile(road.elevationProfile, 0)
                    waypoint_geometry_origin_lane_center.road_id = road.id
                    waypoint_geometry_origin_lane_center.lane_id = lane.id
                    waypoint_geometry_origin_lane_center.s = road.planView[i].s
                    waypoint_geometry_origin_lane_center.junction = road.junction
                    
                    if (road.planView[i].type == "line"): 
                        n = int(road.planView[i].length/distance)                             
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_line(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, road.planView[i].length, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, road.planView[i].length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + road.planView[i].length
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
                    
                    
                    elif (road.planView[i].type == "arc"):
                        radius = 1/road.planView[i].curvature
                        new_radius = radius - lane_offset
                        new_curvature = 1/new_radius
                        k_arc = road.planView[i].length/radius
                        new_length = k_arc*new_radius
                        n = int(new_length/distance)
                        for n_dist in range(0,n):
                            waypoint = Waypoint()
                            waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, n_dist*distance, new_curvature, road.planView[i].hdg)
                            #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, n_dist*distance)
                            waypoint.road_id = road.id
                            waypoint.lane_id = lane.id
                            waypoint.s = road.planView[i].s + n_dist*distance
                            waypoint.junction = road.junction
                            waypoints.append(waypoint)
                        
                        waypoint = Waypoint()
                        waypoint.transform.location = get_point_in_arc(waypoint_geometry_origin_lane_center.transform.location.x, waypoint_geometry_origin_lane_center.transform.location.y, 0, new_length, new_curvature, road.planView[i].hdg)
                        #waypoint.transform.location.z = calculate_elevationProfile(road.elevationProfile, new_length)
                        waypoint.road_id = road.id
                        waypoint.lane_id = lane.id
                        waypoint.s = road.planView[i].s + new_length
                        waypoint.junction = road.junction
                        waypoints.append(waypoint)
            return waypoints

def calculate_elevation(a, b, c, d, s_0, s):
    """
    Return elevation (inertial z) at a given position (knowing the elevationProfile)
    """
    ds = s - s_0
    elevation = a + b*ds + c*ds**2 + d*ds**3
    return elevation

def calculate_elevationProfile(elevationProfile, s):
    """
    Return elevation (inertial z) at a given position (not knowing which elevationProfile is)
    """
    if elevationProfile:
        for i in range(0,len(elevationProfile)):
            if elevationProfile[i].s > s: break
        if (i == len(elevationProfile)):
            profile = elevationProfile[i]
        else:
            profile = elevationProfile[i-1]

        elevation = calculate_elevation(profile.a, profile.b, profile.c, profile.d, profile.s, s)
        return elevation

def get_affecting_lanes (validity, affecting_lanes):
    from_lane = int(validity.fromLane)
    to_lane = int(validity.fromLane)

    if from_lane == to_lane:
        affecting_lanes.append(from_lane)
    else:
        for lane in range (from_lane, to_lane):
            affecting_lanes.append(lane)
    return affecting_lanes

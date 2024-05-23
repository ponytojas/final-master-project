from carla_interactor.graph.planning import LaneGraphPlanner


def create_planner(carla_map):
    name = carla_map.name.split('/')[-1]
    # return LaneGraphPlanner(name, '/opt/carla-simulator/CarlaUE4/Content/Carla/Maps/OpenDrive', False)
    return LaneGraphPlanner(name, '/home/ponytojas/Downloads/planning_&_control/maps_0913/', False)


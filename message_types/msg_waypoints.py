"""
msg_waypoints
    - messages type for input to path manager
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        3/26/2019 - RWB
"""
import numpy as np

class msg_waypoints:
    def __init__(self):
        self.flag_waypoints_changed = True
        self.flag_manager_requests_waypoints = True

        # type of waypoint following:
        #   - straight line following
        #   - fillets between straight lines
        #   - follow dubins paths
        self.type = 'straight_line'
        # self.type = 'fillet'
        # self.type = 'dubins'
        self.max_waypoints = 100
        # current number of valid waypoints in memory
        self.num_waypoints = 0
        # [n, e, d] - coordinates of waypoints
        self.ned = np.inf * np.ones((3, self.max_waypoints))
        # the airspeed that is commanded along the waypoints
        self.airspeed = np.inf * np.ones((1, self.max_waypoints))
        # the desired course at each waypoint (used only for Dubins paths)
        self.course = np.inf * np.ones((1, self.max_waypoints))

        # these last three variables are used by the path planner running cost at each node
        self.cost = np.inf * np.ones((1, self.max_waypoints))
        # index of the parent to the node
        self.parent_idx = np.inf * np.ones((1, self.max_waypoints))
        # can this node connect to the goal?
        self.flag_connect_to_goal = 0 * np.ones((1, self.max_waypoints))

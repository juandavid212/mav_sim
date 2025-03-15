import numpy as np
import sys
sys.path.append('..')
from parameters.dubins_parameters import dubins_parameters
from message_types.msg_path import msg_path

class path_manager:
    def __init__(self):
        # message sent to path follower
        self.path = msg_path()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        # flag that request new waypoints from path planner
        self.flag_need_new_waypoints = True
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        # dubins path parameters
        self.dubins_path = dubins_parameters()

    def update(self, waypoints, radius, state):
        if self.path.flag_path_changed == True:
            self.path.flag_path_changed = False
        self.dubins_manager(waypoints, radius, state)
        if waypoints.flag_waypoints_changed == True:
            waypoints.flag_waypoints_changed = False
        self.path.airspeed = waypoints.airspeed.item(self.ptr_current)
        return self.path

    def dubins_manager(self, waypoints, radius, state):
        if waypoints.num_waypoints < 2:
            print("Must have at least 2 waypoints")
        else:
            if waypoints.flag_waypoints_changed:
                self.initialize_pointers(waypoints)
                self.dubins_path.update(waypoints.ned[:, self.ptr_previous], waypoints.course.item(self.ptr_previous),
                                        waypoints.ned[:, self.ptr_current], waypoints.course.item(self.ptr_current), radius)
                self.path.type = 'orbit'
                self.path.flag_path_changed = True
                self.path.orbit_center = self.dubins_path.center_s
                self.path.orbit_radius = self.dubins_path.radius
                self.orbitDirection(self.dubins_path.dir_s)
                self.halfspace_r = self.dubins_path.r1
                self.halfspace_n = -self.dubins_path.n1
            if self.manager_state == 1:
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 2
                    self.halfspace_r = self.dubins_path.r1
                    self.halfspace_n = self.dubins_path.n1
            elif self.manager_state == 2:
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 3
                    self.path.type = 'line'
                    self.path.flag_path_changed = True
                    self.path.line_origin = self.dubins_path.r1
                    self.path.line_direction = self.dubins_path.n1
                    self.halfspace_r = self.dubins_path.r2
                    self.halfspace_n = self.dubins_path.n1
            elif self.manager_state == 3:
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 4
                    self.path.type = 'orbit'
                    self.path.flag_path_changed = True
                    self.path.orbit_center = self.dubins_path.center_e
                    self.path.orbit_radius = self.dubins_path.radius
                    self.orbitDirection(self.dubins_path.dir_e)
                    self.halfspace_r = self.dubins_path.r3
                    self.halfspace_n = -self.dubins_path.n3
            elif self.manager_state == 4:
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 5
                    self.halfspace_r = self.dubins_path.r3
                    self.halfspace_n = self.dubins_path.n3
            elif self.manager_state == 5:
                if self.inHalfSpace(np.array([state.pn, state.pe, -state.h]).T):
                    self.manager_state = 1
                    self.increment_pointers()
                    self.dubins_path.update(waypoints.ned[:, self.ptr_previous],
                                            waypoints.course.item(self.ptr_previous),
                                            waypoints.ned[:, self.ptr_current], waypoints.course.item(self.ptr_current),
                                            radius)
                    self.path.type = 'orbit'
                    self.path.flag_path_changed = True
                    self.path.orbit_center = self.dubins_path.center_s
                    self.path.orbit_radius = self.dubins_path.radius
                    self.orbitDirection(self.dubins_path.dir_s)
                    self.halfspace_r = self.dubins_path.r1
                    self.halfspace_n = -self.dubins_path.n1
            else:
                print("Error in manager state")

    def initialize_pointers(self, waypoints):
        self.num_waypoints = waypoints.num_waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2

    def increment_pointers(self):
        self.ptr_previous = self.ptr_current
        self.ptr_current = self.ptr_next
        self.ptr_next += 1
        if self.ptr_next == self.num_waypoints:
            self.ptr_next = 0

    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r).T @ self.halfspace_n >= 0:
            return True
        else:
            return False

    def updateLineHalfSpace(self, waypoints):
        r = waypoints.ned[:, self.ptr_previous]
        q_prev = (waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous]) / np.linalg.norm(
            waypoints.ned[:, self.ptr_current] - waypoints.ned[:, self.ptr_previous])
        q = (waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current]) / np.linalg.norm(
            waypoints.ned[:, self.ptr_next] - waypoints.ned[:, self.ptr_current])
        n = (q_prev + q) / np.linalg.norm(q_prev + q)
        self.halfspace_r = waypoints.ned[:, self.ptr_current]
        self.halfspace_n = n
        self.path.line_origin = r
        self.path.line_direction = q_prev

    def orbitDirection(self, orbitD):
        if orbitD == 1:
            self.path.orbit_direction = 'CW'
        elif orbitD == -1:
            self.path.orbit_direction = 'CCW'
        else:
            print("error in orbitDirection")
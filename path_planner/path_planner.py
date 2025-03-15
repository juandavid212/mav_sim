# path planner for mavsim_python
#
# mavsim_python
#     - Beard & McLain, PUP, 2012
#     - Last updated:
#         4/3/2019 - BGM
import numpy as np
import sys
sys.path.append('..')
from message_types.msg_waypoints import msg_waypoints
from path_planner.planRRT import planRRT
from path_planner.planRRTDubins import planRRTDubins
from path_planner.planRRTDubinsProj import planRRTDubinsProj

class path_planner:
    def __init__(self):
        # waypoints definition
        self.waypoints = msg_waypoints()
        self.rrt = planRRT(map)
        self.rrtDubins = planRRTDubins(map)
        self.rrtDubinsProj = planRRTDubinsProj(map)

    def update(self, map, state, PLAN):
        self.waypoints.type = 'dubins'
        self.waypoints.num_waypoints = 0
        Va = 25
        numberWaypoints = 4
        primaryWaypoints = np.array([[0., 0., -100.],
                                        [1400., 0., -100.],
                                        [1400., 1400., -100.],
                                        [0., 1400., -100.]]).T
        primaryWaypointsAirspeed = np.array([[Va, Va, Va, Va]])
        primaryCourseAngles = np.array([[np.radians(0),
                                            np.radians(0),
                                            np.radians(90),
                                            np.radians(180)]])
        for i in range(0, numberWaypoints):
            # current configuration vector format: N, E, D, Va
            if i == 0 and np.sqrt((state.pn - primaryWaypoints[0,0])**2 + (state.pe - primaryWaypoints[1,0])**2) > PLAN.R_min:
                wpp_start = np.array([state.pn,
                                        state.pe,
                                        -state.h,
                                        state.chi,
                                        state.Va])
                self.waypoints.ned[:, self.waypoints.num_waypoints] = wpp_start[0:3]
                self.waypoints.course[:, self.waypoints.num_waypoints] = wpp_start.item(3)
                self.waypoints.airspeed[:, self.waypoints.num_waypoints] = wpp_start.item(4)
                self.waypoints.num_waypoints += 1

            elif i == 0:
                self.waypoints.ned[:, self.waypoints.num_waypoints] = np.array([primaryWaypoints[0, 0],
                                                                                    primaryWaypoints[1, 0],
                                                                                    primaryWaypoints[2, 0]])
                self.waypoints.course[:, self.waypoints.num_waypoints] = primaryCourseAngles.item(0)
                self.waypoints.airspeed[:, self.waypoints.num_waypoints] = primaryWaypointsAirspeed.item(0)
                self.waypoints.num_waypoints += 1
                continue
            else:
                wpp_start = np.array([primaryWaypoints[0, i - 1],
                                        primaryWaypoints[1, i - 1],
                                        primaryWaypoints[2, i - 1],
                                        primaryCourseAngles.item(i-1),
                                        primaryWaypointsAirspeed.item(i-1)])
            wpp_end = np.array([primaryWaypoints[0, i],
                                primaryWaypoints[1, i],
                                primaryWaypoints[2, i],
                                primaryCourseAngles.item(i),
                                primaryWaypointsAirspeed.item(i)])
            waypoints = self.rrtDubins.planPath(wpp_start, wpp_end, PLAN.R_min, map)
            numNew = waypoints.num_waypoints-1
            numOld = self.waypoints.num_waypoints
            if numNew > 1:
                self.waypoints.ned[:, numOld:numOld + numNew] = waypoints.ned[:, 1:numNew+1]
                self.waypoints.course[:,numOld:numOld + numNew] = waypoints.course[:,1:numNew+1]
                self.waypoints.airspeed[:, numOld:(numOld + numNew)] = wpp_end.item(4) * np.ones((1, numNew))
            else:
                self.waypoints.ned[:, numOld] = waypoints.ned[:, 1]
                self.waypoints.course[:, numOld] = waypoints.course[:, 1]
                self.waypoints.airspeed[:, numOld] = wpp_end.item(4) * np.ones((1, numNew))
            self.waypoints.num_waypoints = numNew + numOld

        return self.waypoints

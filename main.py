"""
mavsim_python
    - Chapter 12 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        4/3/2019 - BGM
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN

from viewers.data_viewer import data_viewer
from wind_simulation import wind_simulation
from control.autopilot import autopilot
from mav_dynamics import mav_dynamics
from observer import observer
from path_manager.path_follower import path_follower
from path_manager.path_manager import path_manager
from viewers.world_viewer import world_viewer
from path_planner.path_planner import path_planner


# initialize the visualization
world_view = world_viewer()
data_view = data_viewer()

# initialize elements of the architecture
wind = wind_simulation(SIM.ts_simulation)
mav = mav_dynamics(SIM.ts_simulation)
ctrl = autopilot(SIM.ts_simulation)
obsv = observer(SIM.ts_simulation)
path_follow = path_follower()
path_manage = path_manager()
path_plan = path_planner()

from message_types.msg_map import msg_map
map = msg_map(PLAN)

sim_time = SIM.start_time

print("Press Command ctrl-c to exit...")
while sim_time < SIM.end_time:
    measurements = mav.update_sensors()
    estimated_state = obsv.update(measurements)

    if path_manage.flag_need_new_waypoints == 1:
        path_plan.update(map, estimated_state, PLAN)
        path_manage.flag_need_new_waypoints = 0

    path = path_manage.update(path_plan.waypoints, PLAN.R_min, estimated_state)

    autopilot_commands = path_follow.update(path, estimated_state)

    delta, commanded_state = ctrl.update(autopilot_commands, estimated_state)

    current_wind = wind.update()
    mav.update_state(delta, current_wind)

    world_view.update(map, path_plan.waypoints, path, mav.msg_true_state)
    data_view.update(mav.msg_true_state,
                     estimated_state,
                     commanded_state,
                     SIM.ts_simulation)

    sim_time += SIM.ts_simulation






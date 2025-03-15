"""
mavsim_python
    - Chapter 10 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        3/11/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from viewers.data_viewer import data_viewer
from wind_simulation import wind_simulation
from control.autopilot import autopilot
from mav_dynamics import mav_dynamics
from observer import observer
from simulation.path_manager.path_follower import path_follower
from viewers.path_viewer import path_viewer
from message_types.msg_autopilot import msg_autopilot

# initialize the visualization
path_view = path_viewer()  # initialize the viewer
data_view = data_viewer()  # initialize view of data plots

# initialize elements of the architecture
wind = wind_simulation(SIM.ts_simulation)
mav = mav_dynamics(SIM.ts_simulation)
ctrl = autopilot(SIM.ts_simulation)
path_follow = path_follower()
obsv = observer(SIM.ts_simulation)

# path definition
from message_types.msg_path import msg_path
path = msg_path()
path.type = 'line'
path.type = 'orbit'
if path.type == 'line':
    path.line_origin = np.array([[0.0, 0.0, -10.0]]).T    
    path.line_direction = np.array([[0.5, 1.0, -.1]]).T
    path.line_direction = path.line_direction / np.linalg.norm(path.line_direction)
else:  # path.flag == 'orbit'
    path.orbit_center = np.array([[0.0, 0.0, -100.0]]).T  # center of the orbit
    path.orbit_radius = 300.0  # radius of the orbit
    path.orbit_direction = 'CCW'  # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise

# initialize the simulation time
sim_time = SIM.start_time

# _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
mav._state[2] = 0  # start flying up
# mav._state[3] = 0  # start flying north
# set the initial euler angles for the observer
# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = mav.update_sensors()  # get sensor measurements
    estimated_state = obsv.update(measurements)  # estimate states from measurements

    #-------path follower-------------
    # autopilot_commands = path_follow.update(path, estimated_state)
    # autopilot_commands = msg_autopilot()
    # autopilot_commands.course_command = 0.0
    autopilot_commands = path_follow.update(path, mav.msg_true_state)  # for debugging

    #-------controller-------------
    # delta, commanded_state = ctrl.update(autopilot_commands, estimated_state)
    delta, commanded_state = ctrl.update(autopilot_commands, mav.msg_true_state)

    #-------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update_state(delta, current_wind)  # propagate the MAV dynamics

    #-------update viewer-------------
    path_view.update(path, mav.msg_true_state)  # plot path and MAV
    data_view.update(mav.msg_true_state, # true states
                     estimated_state, # estimated states
                     commanded_state, # commanded states
                     SIM.ts_simulation)

    #-------increment time-------------
    sim_time += SIM.ts_simulation





"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/16/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import msg_state
from message_types.msg_sensors import msg_sensors

import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from tools.angleConversions import Quaternion2Rotation, Quaternion2Euler

class mav_dynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[MAV.pn0],  # (0)
                                [MAV.pe0],  # (1)
                                [MAV.pd0],  # (2)
                                [MAV.u0],  # (3)
                                [MAV.v0],  # (4)
                                [MAV.w0],  # (5)
                                [MAV.e0],  # (6)
                                [MAV.e1],  # (7)
                                [MAV.e2],  # (8)
                                [MAV.e3],  # (9)
                                [MAV.p0],  # (10)
                                [MAV.q0],  # (11)
                                [MAV.r0]])  # (12)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        self._update_velocity_data(np.zeros((6, 1)))
        self._Va = MAV.Va0
        self._Vg = MAV.Va0
        self._alpha = 0
        self._beta = 0
        self._chi = 0  # could initialize better
        self._gamma = 0  # could initialize better
        # initialize true_state message
        self.msg_true_state = msg_state()

        self._forces = np.array([[0.], [0.], [0.]])
        # initialize the sensors message
        self.sensors = msg_sensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.


    ###################################
    # public functions
    def update_state(self, delta, wind):
        '''
                    Integrate the differential equations defining dynamics, update sensors
                    delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
                    wind is the wind vector in inertial coordinates
                    Ts is the time step between function calls.
                '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step / 2. * k1, forces_moments)
        k3 = self._derivatives(self._state + time_step / 2. * k2, forces_moments)
        k4 = self._derivatives(self._state + time_step * k3, forces_moments)
        self._state += time_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0 ** 2 + e1 ** 2 + e2 ** 2 + e3 ** 2)
        self._state[6][0] = self._state.item(6) / normE
        self._state[7][0] = self._state.item(7) / normE
        self._state[8][0] = self._state.item(8) / normE
        self._state[9][0] = self._state.item(9) / normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)

        # update the message class for the true state
        self._update_msg_true_state()

    def update_sensors(self):
        "Return value of sensors on MAV: gyros, accels, static_pressure, dynamic_pressure, GPS"
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        self.sensors.gyro_x = self._state.item(10) + SENSOR.gyro_x_bias + SENSOR.gyro_sigma*np.random.randn()
        self.sensors.gyro_y = self._state.item(11) + SENSOR.gyro_y_bias + SENSOR.gyro_sigma*np.random.randn()
        self.sensors.gyro_z = self._state.item(12) + SENSOR.gyro_z_bias + SENSOR.gyro_sigma*np.random.randn()
        self.sensors.accel_x = self._forces[0]/MAV.mass + MAV.gravity*np.sin(theta) + SENSOR.accel_sigma*np.random.randn()
        self.sensors.accel_y = self._forces[1]/MAV.mass - MAV.gravity*np.cos(theta)*np.sin(phi) + SENSOR.accel_sigma*np.random.randn()
        self.sensors.accel_z = self._forces[2]/MAV.mass - MAV.gravity*np.cos(theta)*np.cos(phi) + SENSOR.accel_sigma*np.random.randn()
        self.sensors.static_pressure = MAV.rho*MAV.gravity*-self._state.item(2) + SENSOR.static_pres_sigma*np.random.randn()
        self.sensors.diff_pressure = MAV.rho*self._Va**2/2. + SENSOR.diff_pres_sigma*np.random.randn()
        # self.sensors.mag = Quaternion2Rotation(self._state[6:10]).T@Euler2Rotation(0,-66*np.pi/180.,12.5*np.pi/180.)@np.array([[1.],[0.],[0.]]) \
        #                    + SENSOR.mag_beta + np.random.normal(0, SENSOR.mag_sigma, 1)
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n = np.exp(-SENSOR.gps_beta*SENSOR.ts_gps)*self._gps_eta_n + SENSOR.gps_n_sigma*np.random.randn()
            self._gps_eta_e = np.exp(-SENSOR.gps_beta*SENSOR.ts_gps)*self._gps_eta_e + SENSOR.gps_e_sigma*np.random.randn()
            self._gps_eta_h = np.exp(-SENSOR.gps_beta*SENSOR.ts_gps)*self._gps_eta_h + SENSOR.gps_h_sigma*np.random.randn()
            self.sensors.gps_n = self._state.item(0) + self._gps_eta_n
            self.sensors.gps_e = self._state.item(1) + self._gps_eta_e
            self.sensors.gps_h = self._state.item(2) + self._gps_eta_h
            self.sensors.gps_Vg = np.sqrt((self._Va*np.cos(psi) + self._wind.item(0))**2 + (self._Va*np.sin(psi) + self._wind.item(1))**2) + SENSOR.gps_Vg_sigma*np.random.randn()
            self.sensors.gps_course = np.arctan2(self._Va*np.sin(psi) + self._wind.item(1), self._Va*np.cos(psi) + self._wind.item(0)) + SENSOR.gps_course_sigma*np.random.randn()
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation
        return self.sensors

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
                for the dynamics xdot = f(x, u), returns f(x, u)
                """
        # extract the states
        pn = state.item(0)
        pe = state.item(1)
        pd = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        pn_dot = (e1 ** 2 + e0 ** 2 - e2 ** 2 - e3 ** 2) * u + \
                 (2 * (e1 * e2 - e3 * e0)) * v + \
                 (2 * (e1 * e3 + e2 * e0)) * w
        pe_dot = (2 * (e1 * e2 + e3 * e0)) * u + \
                 (e2 ** 2 + e0 ** 2 - e1 ** 2 - e3 ** 2) * v + \
                 (2 * (e2 * e3 - e1 * e0)) * w
        pd_dot = (2 * (e1 * e3 - e2 * e0)) * u + \
                 (2 * (e2 * e3 + e1 * e0)) * v + \
                 (e3 ** 2 + e0 ** 2 - e1 ** 2 - e2 ** 2) * w

        # position dynamics
        u_dot = r * v - q * w + fx / MAV.mass
        v_dot = p * w - r * u + fy / MAV.mass
        w_dot = q * u - p * v + fz / MAV.mass

        # rotational kinematics
        e0_dot = (-p * e1 - q * e2 - r * e3) / 2
        e1_dot = (p * e0 + r * e2 - q * e3) / 2
        e2_dot = (q * e0 - r * e1 + p * e3) / 2
        e3_dot = (r * e0 + q * e1 - p * e2) / 2

        # rotatonal dynamics
        p_dot = MAV.gamma1 * p * q - MAV.gamma2 * q * r + MAV.gamma3 * l + MAV.gamma4 * n
        q_dot = MAV.gamma5 * p * r - MAV.gamma6 * (p ** 2 - r ** 2) + m / MAV.Jy
        r_dot = MAV.gamma7 * p * q - MAV.gamma1 * q * r + MAV.gamma4 * l + MAV.gamma8 * n

        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot

    def _update_velocity_data(self, wind=np.zeros((6,1))):
        R = Quaternion2Rotation(self._state[6:10])
        # Should _wind be in inertial or body
        wind2 = R.T @ wind[0:3] + wind[3:6]
        self._wind = R @ wind2

        wind2 = np.zeros((6, 1))

        ur = self._state.item(3) - wind2.item(0)
        vr = self._state.item(4) - wind2.item(1)
        wr = self._state.item(5) - wind2.item(2)

        # compute groud speed
        Vg = R @ self._state[3:6]
        self._Vg = np.sqrt(Vg.item(0) ** 2 + Vg.item(1) ** 2 + Vg.item(2) ** 2)
        self._chi = np.arctan2(Vg.item(1), Vg.item(0))
        if self._Vg == 0:
            self._gamma = MAV.psi0
        else:
            self._gamma = np.arcsin(Vg.item(2) / self._Vg)
        # compute airspeed
        self._Va = np.sqrt(ur ** 2 + vr ** 2 + wr ** 2)
        # compute angle of attack
        self._alpha = np.arctan2(wr, ur)
        # compute sideslip angle
        if self._Va == 0:
            self._beta = 0
        else:
            self._beta = np.arcsin(vr / self._Va)

    def _forces_moments(self, delta):
        """
                return the forces on the UAV based on the state, wind, and control surfaces
                :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
                :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
                """
        # Forces due to gravity
        fx = MAV.mass * MAV.gravity * 2 * (self._state[7] * self._state[9] - self._state[8] * self._state[6])
        fy = MAV.mass * MAV.gravity * 2 * (self._state[8] * self._state[9] + self._state[7] * self._state[6])
        fz = MAV.mass * MAV.gravity * (
                    self._state[9] ** 2 + self._state[6] ** 2 - self._state[7] ** 2 - self._state[8] ** 2)

        # Forces due to aerodynamics
        cA = np.cos(self._alpha)
        sA = np.sin(self._alpha)
        Cd = MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha * self._alpha) ** 2 / (np.pi * MAV.e * MAV.AR)
        sigmaA = (1 + np.exp(-MAV.M * (self._alpha - MAV.alpha0)) + np.exp(MAV.M * (self._alpha + MAV.alpha0))) \
                 / ((1 + np.exp(-MAV.M * (self._alpha - MAV.alpha0))) * (
                    1 + np.exp(MAV.M * (self._alpha + MAV.alpha0))))
        Cl = (1 - sigmaA) * (MAV.C_L_0 + MAV.C_L_alpha * self._alpha) + sigmaA * (
                    2 * np.sign(self._alpha) * sA ** 2 * cA)
        Cx = -Cd * cA + Cl * sA
        Cxq = -MAV.C_D_q * cA + MAV.C_L_q * sA
        Cxdeltae = -MAV.C_D_delta_e * cA + MAV.C_L_delta_e * sA
        Cz = -Cd * sA - Cl * cA
        Czq = -MAV.C_D_q * sA - MAV.C_L_q * cA
        Czdeltae = -MAV.C_D_delta_e * sA - MAV.C_L_delta_e * cA

        fx += .5 * MAV.rho * self._Va ** 2 * MAV.S_wing * (
                    Cx + Cxq * MAV.c * self._state[11] / (2 * self._Va) + Cxdeltae * delta[1])
        fy += .5 * MAV.rho * self._Va ** 2 * MAV.S_wing * (
                    MAV.C_Y_0 + MAV.C_Y_beta * self._beta + MAV.C_Y_p * MAV.b * self._state[10] / (2 * self._Va) \
                    + MAV.C_Y_r * MAV.b * self._state[12] / (2 * self._Va) + MAV.C_Y_delta_a * delta[
                        0] + MAV.C_Y_delta_r * delta[2])
        fz += .5 * MAV.rho * self._Va ** 2 * MAV.S_wing * (
                    Cz + Czq * MAV.c * self._state[11] / (2 * self._Va) + Czdeltae * delta[1])
        Mx = .5 * MAV.rho * self._Va ** 2 * MAV.S_wing * (MAV.b * (
                    MAV.C_ell_0 + MAV.C_ell_beta * self._beta + MAV.C_ell_p * MAV.b * self._state[10] / (2 * self._Va) \
                    + MAV.C_ell_r * MAV.b * self._state[12] / (2 * self._Va) + MAV.C_ell_delta_a * delta[
                        0] + MAV.C_ell_delta_r * delta[2]))
        My = .5 * MAV.rho * self._Va ** 2 * MAV.S_wing * (MAV.c * (
                    MAV.C_m_0 + MAV.C_m_alpha * self._alpha + MAV.C_m_q * MAV.c * self._state[11] / (
                        2 * self._Va) + MAV.C_m_delta_e * delta[1]))
        Mz = .5 * MAV.rho * self._Va ** 2 * MAV.S_wing * (MAV.b * (
                    MAV.C_n_0 + MAV.C_n_beta * self._beta + MAV.C_n_p * MAV.b * self._state[10] / (2 * self._Va) \
                    + MAV.C_n_r * MAV.b * self._state[12] / (2 * self._Va) + MAV.C_n_delta_a * delta[
                        0] + MAV.C_n_delta_r * delta[2]))

        # Compute thrust and torque due to propeller
        # map delta_t throttle command (0 to 1) into motor input voltage
        V_in = MAV.V_max * delta[3]
        # Quadratic formula to solve for motor speed
        a = MAV.C_Q0 * MAV.rho * np.power(MAV.D_prop, 5) \
            / ((2. * np.pi) ** 2)
        b = (MAV.C_Q1 * MAV.rho * np.power(MAV.D_prop, 4) \
             / (2. * np.pi)) * self._Va + MAV.KQ ** 2 / MAV.R_motor
        c = MAV.C_Q2 * MAV.rho * np.power(MAV.D_prop, 3) \
            * self._Va ** 2 - (MAV.KQ / MAV.R_motor) * V_in + MAV.KQ * MAV.i0
        # Consider only positive _rotate_points
        Omega_op = (-b + np.sqrt(b ** 2. - 4. * a * c)) / (2. * a)
        # compute advance ratio
        J_op = 2. * np.pi * self._Va / (Omega_op * MAV.D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = MAV.C_T2 * J_op ** 2 + MAV.C_T1 * J_op + MAV.C_T0
        C_Q = MAV.C_Q2 * J_op ** 2 + MAV.C_Q1 * J_op + MAV.C_Q0
        # add thrust and torque due to propeller
        n = Omega_op / (2 * np.pi)
        fx += MAV.rho * n ** 2 * np.power(MAV.D_prop, 4) * C_T
        Mx += -MAV.rho * n ** 2 * np.power(MAV.D_prop, 5) * C_Q

        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, Va, delta_t):
        # Compute thrust and torque due to propeller
        # map delta_t throttle command (0 to 1) into motor input voltage
        V_in = MAV.V_max * delta_t
        # Quadratic formula to solve for motor speed
        a = MAV.C_Q0 * MAV.rho * np.power(MAV.D_prop, 5) \
            / ((2. * np.pi) ** 2)
        b = (MAV.C_Q1 * MAV.rho * np.power(MAV.D_prop, 4) \
             / (2. * np.pi)) * Va + MAV.KQ ** 2 / MAV.R_motor
        c = MAV.C_Q2 * MAV.rho * np.power(MAV.D_prop, 3) \
            * Va ** 2 - (MAV.KQ / MAV.R_motor) * V_in + MAV.KQ * MAV.i0
        # Consider only positive _rotate_points
        Omega_op = (-b + np.sqrt(b ** 2. - 4. * a * c)) / (2. * a)
        # compute advance ratio
        J_op = 2. * np.pi * Va / (Omega_op * MAV.D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = MAV.C_T2 * J_op ** 2 + MAV.C_T1 * J_op + MAV.C_T0
        C_Q = MAV.C_Q2 * J_op ** 2 + MAV.C_Q1 * J_op + MAV.C_Q0
        # add thrust and torque due to propeller
        n = Omega_op / (2 * np.pi)
        T_p = MAV.rho * n ** 2 * np.power(MAV.D_prop, 4) * C_T
        Q_p += -MAV.rho * n ** 2 * np.power(MAV.D_prop, 5) * C_Q
        return T_p, Q_p


    def _update_msg_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        self.msg_true_state.pn = self._state.item(0)
        self.msg_true_state.pe = self._state.item(1)
        self.msg_true_state.h = -self._state.item(2)
        self.msg_true_state.Va = self._Va
        self.msg_true_state.alpha = self._alpha
        self.msg_true_state.beta = self._beta
        self.msg_true_state.phi = phi
        self.msg_true_state.theta = theta
        self.msg_true_state.psi = psi
        self.msg_true_state.Vg = self._Vg
        self.msg_true_state.gamma = self._gamma
        self.msg_true_state.chi = self._chi
        self.msg_true_state.p = self._state.item(10)
        self.msg_true_state.q = self._state.item(11)
        self.msg_true_state.r = self._state.item(12)
        self.msg_true_state.wn = self._wind.item(0)
        self.msg_true_state.we = self._wind.item(1)

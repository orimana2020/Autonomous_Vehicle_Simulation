import numpy as np
import math
import matplotlib.pyplot as plt
import cubic_spline_planner
from py_simulator import State, States, Simulator

class Trajectory(object):
    def __init__(self,path ,dl=1.0, TARGET_SPEED=10.0 / 3.6 ):
        self.ax = []
        self.ay = []
        for coords in path:
            self.ax.append(coords[0])
            self.ay.append(coords[1])
      
        self.cx, self.cy, cyaw, self.ck, self.s = cubic_spline_planner.calc_spline_course(self.ax, self.ay, ds=dl)
        self.cyaw = self.smooth_yaw(cyaw)
        self.sp = self.calc_speed_profile(self.cx, self.cy, self.cyaw, TARGET_SPEED)
        self.dl = dl
    
    def smooth_yaw(self, yaw):
        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]
            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]
            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]
        return yaw


    def calc_speed_profile(self, cx, cy, cyaw, target_speed):
        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward
        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]
            move_direction = math.atan2(dy, dx)
            if dx != 0.0 and dy != 0.0:
                dangle = abs(self.pi_2_pi(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0
            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed
        speed_profile[-1] = 0.0
        return speed_profile
    
    def pi_2_pi(self, angle):
        return self.angle_mod(angle)
    
    def angle_mod(self, x, zero_2_2pi=False, degree=False):
        """
        Angle modulo operation
        Default angle modulo range is [-pi, pi)

        Parameters
        ----------
        x : float or array_like
            A angle or an array of angles. This array is flattened for
            the calculation. When an angle is provided, a float angle is returned.
        zero_2_2pi : bool, optional
            Change angle modulo range to [0, 2pi)
            Default is False.
        degree : bool, optional
            If True, then the given angles are assumed to be in degrees.
            Default is False.

        Returns
        -------
        ret : float or ndarray
            an angle or an array of modulated angle.

        Examples
        --------
        >>> angle_mod(-4.0)
        2.28318531

        >>> angle_mod([-4.0])
        np.array(2.28318531)

        >>> angle_mod([-150.0, 190.0, 350], degree=True)
        array([-150., -170.,  -10.])

        >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
        array([300.])

        """
        if isinstance(x, float):
            is_float = True
        else:
            is_float = False

        x = np.asarray(x).flatten()
        if degree:
            x = np.deg2rad(x)

        if zero_2_2pi:
            mod_angle = x % (2 * np.pi)
        else:
            mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

        if degree:
            mod_angle = np.rad2deg(mod_angle)

        if is_float:
            return mod_angle.item()
        else:
            return mod_angle



class PurePersuit_Controller(object):
    def __init__(self,cx, cy, k=0.1, Lfc = 0.5, Kp = 1.0, WB = 0.35, MAX_ACCEL=1.0, MAX_SPEED=3, MIN_SPEED=-3, MAX_STEER=np.deg2rad(40.0), MAX_DSTEER=np.deg2rad(90.0) ):
        self.k = k  # look forward gain
        self.Lfc = Lfc   # [m] look-ahead distance
        self.Kp = Kp  # speed proportional gain
        self.WB = WB
        self.old_nearest_point_index = None
        self.cx = cx
        self.cy = cy
        self.MAX_ACCEL = MAX_ACCEL
        self.MAX_SPEED  = MAX_SPEED
        self.MIN_SPEED = MIN_SPEED
        self.MAX_STEER = MAX_STEER
        self.MAX_DSTEER = MAX_DSTEER

    def pure_pursuit_steer_control(self, state, trajectory, pind, dt):
        ind, Lf = self.search_target_index(state)
        if pind >= ind:
            ind = pind
        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1
        alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
        delta = math.atan2(2.0 * self.WB * math.sin(alpha) / Lf, 1.0)

        if abs(delta - state.predelta) > self.MAX_DSTEER * dt:
            if delta > state.predelta:
                delta = state.predelta + self.MAX_DSTEER * dt
            else: 
                delta = state.predelta - self.MAX_DSTEER * dt
        if delta >= self.MAX_STEER:
            delta = self.MAX_STEER
        elif delta <= -self.MAX_STEER:
            delta = -self.MAX_STEER
        return delta, ind
    
    def proportional_control_acceleration(self, target_speed, current_speed, dt):
        acc = self.Kp * (target_speed - current_speed)
        acc = min(acc, self.MAX_ACCEL )
        linear_velocity = current_speed + acc * dt
        if linear_velocity > self.MAX_SPEED:
            linear_velocity = self.MAX_SPEED
        elif linear_velocity < self.MIN_SPEED:
            linear_velocity = self.MIN_SPEED
        return linear_velocity
        

    def search_target_index(self, state):
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = self.calc_distance(state.rear_x,state.rear_y, self.cx[ind],self.cy[ind])
            while True:
                distance_next_index = self.calc_distance(state.rear_x,state.rear_y,self.cx[ind + 1], self.cy[ind + 1])              
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind
        Lf = self.k * state.v + self.Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > self.calc_distance(state.rear_x,state.rear_y,self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1
        return ind, Lf
    
    def calc_distance(self, rear_x, rear_y,point_x, point_y):
        dx = rear_x - point_x
        dy = rear_y - point_y
        return math.hypot(dx, dy)



def main():
    #  hyper-parameters
    k = 0.1  # look forward gain
    Lfc = 0.5  # [m] look-ahead distance
    Kp = 1.0  # speed proportional gain
    dt = 0.1  # [s] time tick
    WB = 0.35 
    target_speed = 2  # [m/s]
    T = 100.0  # max simulation time
    MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
    MAX_DSTEER = np.deg2rad(90.0)  # maximum steering speed [rad/s]
    MAX_SPEED = 10.0 / 3.6  # maximum speed [m/s]
    MIN_SPEED = 0.1  # minimum speed [m/s]
    MAX_ACCEL = 1.0  # maximum accel [m/ss]

    """
    load path and convert it to trajectory, smooth curve, 
    """
    path = np.load('path_index.npy')
    trajectory = Trajectory(dl=0.5, path =path, TARGET_SPEED=target_speed)
    # initial state
    state = State(x=trajectory.cx[0], y=trajectory.cy[0], yaw=trajectory.cyaw[0], v=0.0)
    # initial yaw compensation
    if state.yaw - trajectory.cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - trajectory.cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0
    lastIndex = len(trajectory.cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    pp = PurePersuit_Controller(trajectory.cx, trajectory.cy, k, Lfc, Kp, WB, MAX_ACCEL, MAX_SPEED, MIN_SPEED, MAX_STEER, MAX_DSTEER)
    target_ind, _ = pp.search_target_index(state)
    simulator = Simulator(trajectory, state)
    while T >= time and lastIndex > target_ind:
        state.v = pp.proportional_control_acceleration(target_speed, state.v, dt)
        delta, target_ind = pp.pure_pursuit_steer_control(state, trajectory, target_ind, dt)
        state.predelta = delta
        simulator.update_state(state, delta)  # Control vehicle
        time += dt
        states.append(time, state, delta)
    simulator.show_simulation(states)

        


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()

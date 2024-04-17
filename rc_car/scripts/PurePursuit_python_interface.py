import numpy as np
import math
import matplotlib.pyplot as plt

from utils import Trajectory




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
    
    def proportional_control_acceleration(self, target_speed, current_speed, dt=0.2):
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
    from py_simulator import State, States, Simulator
    #  hyper-parameters
    k = 0.1  # look forward gain
    Lfc = 1.0  # [m] look-ahead distance
    Kp = 1.0  # speed proportional gain
    dt = 0.1  # [s] time tick
    WB = 0.335 
    target_speed = 1.0  # [m/s]
    T = 100.0  # max simulation time
    MAX_STEER = np.deg2rad(40.0)  # maximum steering angle [rad]
    MAX_DSTEER = np.deg2rad(150.0)  # maximum steering speed [rad/s]
    MAX_SPEED = 10.0 / 3.6  # maximum speed [m/s]
    MIN_SPEED = 0.1  # minimum speed [m/s]
    MAX_ACCEL = 1.0  # maximum accel [m/ss]

    """
    load path and convert it to trajectory, smooth curve, 
    """
    # path = np.load('path_meter_test.npy')
    path = np.load('path_meter.npy')
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

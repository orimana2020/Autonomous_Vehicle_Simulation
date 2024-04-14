import matplotlib.pyplot as plt
import math
import numpy as np
import sys
import pathlib
# import  rc_car.scripts.utils.cubic_spline_planner as cubic_spline_planner
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))


MAX_TIME = 500.0  # max simulation time
TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
# DT = 0.2  # [s] time tick

# Vehicle parameters
LENGTH = 0.5  # [m]
WIDTH = 0.3  # [m]
BACKTOWHEEL = 0.1  # [m]
WHEEL_LEN = 0.1  # [m]
WHEEL_WIDTH = 0.07  # [m]
TREAD = 0.2  # [m]
WB = 0.35  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]

show_animation = True


class State:
    """
    vehicle state class
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.predelta = 0.0
    
    

class States(object):
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.d = []
        self.a = []

    def append(self, t, state,a=0, delta=0 ):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)
        self.d.append(delta)
        self.a.append(a)




class Simulator(object):
    def __init__(self, trajectory, state, DT = 0.1):
        self.trajectory = trajectory
        self.time = 0.0
        self.dt = DT
        

    def plot_car(self, x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

        outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
        fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                            [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])
        rr_wheel = np.copy(fr_wheel)
        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                        [-math.sin(yaw), math.cos(yaw)]])
        Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                        [-math.sin(steer), math.cos(steer)]])
        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB
        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T
        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y

        plt.plot(np.array(outline[0, :]).flatten(),
                np.array(outline[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fr_wheel[0, :]).flatten(),
                np.array(fr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rr_wheel[0, :]).flatten(),
                np.array(rr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fl_wheel[0, :]).flatten(),
                np.array(fl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rl_wheel[0, :]).flatten(),
                np.array(rl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(x, y, "*")


    def update_state(self, state, delta):
        # input check
        # if delta >= MAX_STEER:
        #     delta = MAX_STEER
        # elif delta <= -MAX_STEER:
        #     delta = -MAX_STEER
        state.x = state.x + state.v * math.cos(state.yaw) * self.dt
        state.y = state.y + state.v * math.sin(state.yaw) * self.dt
        state.yaw = state.yaw + state.v / WB * math.tan(delta) * self.dt
        # state.v = state.v + a * self.dt
        # if state.v > MAX_SPEED:
        #     state.v = MAX_SPEED
        # elif state.v < MIN_SPEED:
        #     state.v = MIN_SPEED
        state.rear_x = state.x - ((WB / 2) * math.cos(state.yaw))
        state.rear_y = state.y - ((WB / 2) * math.sin(state.yaw))
        self.time +=  self.dt
        return state

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
    

    def show_simulation(self, states):
        for i in range(len(states.x)):
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(self.trajectory.cx, self.trajectory.cy, "-r", label="course")
            plt.plot(self.trajectory.ax, self.trajectory.ay, "-b", label="course")
            plt.plot(states.x[:i], states.y[:i], "ob", label="trajectory")
            self.plot_car(states.x[i], states.y[i], states.yaw[i], steer=states.d[i])
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(states.t[i], 2))
                    + ", speed[km/h]:" + str(round(states.v[i] * 3.6, 2)))
            plt.pause(0.0001)
    
    def save_simulation(self):
        pass
    
def get_control_command():
    return 0.1, 0.1


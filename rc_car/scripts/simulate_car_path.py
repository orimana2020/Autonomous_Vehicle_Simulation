import numpy as np 
import matplotlib.pyplot as plt 
from typing import NamedTuple
import math
from scipy.spatial.transform import Rotation


class AckermannState(NamedTuple):
    position: np.array
    orientation: Rotation
    track_wheel_speed: float
    steering_angle: float
    time: float




class FeedBack(object):
    def __init__(self):
        self.traction_wheels_velocity = 18.0
        self.steer_positions = np.deg2rad(15)


class Odometry(object):
    def __init__(self):
        self.wheelbase_length = 0.3429
        self.wheel_radius = 0.056
        self.center_of_mass_offset = 0.0
        self.damping_factor = 1.0

    def state_update(self, state: AckermannState, feedback: FeedBack):
        """Calculate the next state based off the current state"""
        
        average_wheel_speed = state.track_wheel_speed
        linear_speed = average_wheel_speed * self.wheel_radius
        turn_radius = self.turn_radius(state.steering_angle)
        angular_speed = linear_speed / turn_radius # This is zero if turn_radius is infinite

        time_delta = 0.1

        heading_delta = angular_speed * time_delta # This is zero if angular_speed is zero

        orientation_delta = Rotation.from_euler('xyz', [0, 0, heading_delta])
        if math.isfinite(turn_radius):
            lateral_delta = turn_radius * (1 - math.cos(heading_delta))
            forward_delta = turn_radius * math.sin(heading_delta)
            relative_delta = np.array([forward_delta, lateral_delta, 0])
            position_delta = state.orientation.apply(relative_delta)
        else:
            position_delta = time_delta * self.linear_velocity(state.orientation, linear_speed)

        return AckermannState(
            position= state.position + self.damping_factor * position_delta,
            orientation = orientation_delta * state.orientation,
            track_wheel_speed = feedback.traction_wheels_velocity,
            steering_angle = feedback.steer_positions,
            time = 0.1
        )
    
    def turn_radius(self, steering_angle: float) -> float:
        """
        Calculate the radius of the circle tracked by a turning vehicle
        For left turns, the radius is positive. For right turns, the radius is negative.
        """
        # If the steering angle is 0, then cot(0) is undefined
        if steering_angle == 0:
            return math.inf
        else:
            radius = math.sqrt(self.center_of_mass_offset**2 + self.wheelbase_length**2 * 1 / math.tan(steering_angle)**2)
            return math.copysign(radius, steering_angle)

    def linear_velocity(self, orientation: Rotation, speed: float) -> np.array:
        return orientation.apply([speed, 0, 0]) # rotate from +x direction
    

def main():
    feedback = FeedBack()
    state = AckermannState(
            position=np.array([0,0,0]),
            orientation=Rotation([0, 0, 0, 1]), # identity
            track_wheel_speed=0.0,
            steering_angle=0.0,
            time= 0.0
        )
    
    odom = Odometry()
    max_itr = 50
    states = []
    for _ in range(max_itr):
        state = odom.state_update(state, feedback)
        states.append(state)
    


    # plotting
    fig = plt.figure()
    ax = fig.add_subplot(111)
    position_x = []
    position_y = []
    for state in states:
        position_x.append(state.position[0])
        position_y.append(state.position[1])
    ax.scatter(position_x, position_y)
    plt.show()

if __name__ == '__main__':
    main()
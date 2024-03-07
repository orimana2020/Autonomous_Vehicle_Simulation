import matplotlib.pyplot as plt
import numpy as np
import math
#from Circle import Circle
class Circle:
    def __init__(self,x,y,r):
        self.x = x
        self.y = y
        self.r = r

def tangent_line_construction(c1, c2):
    x1, y1, r1 = c1.x, c1.y, c1.r
    x2, y2, r2 = c2.x, c2.y, c2.r

    d_sq = (x2 - x1)**2 + (y2 - y1)**2
    tangent_lines = []

    if d_sq < (r1 - r2)**2:
        if d_sq != max(r1, r2) and d_sq < max(r1, r2):
            print("Circles are contained within each other and not tangent. No tangent lines exist.")
            return tangent_lines

    d = math.sqrt(d_sq)
    vx = (x2 - x1) / d
    vy = (y2 - y1) / d

    for sign1 in [+1, -1]:
        c = (r1 - sign1 * r2) / d
        if c**2 > 1.0:
            continue
        h = math.sqrt(max(0.0, 1.0 - c**2))

        for sign2 in [+1, -1]:
            nx = vx * c - sign2 * h * vy
            ny = vy * c + sign2 * h * vx
            tangent_lines.append(((x1 + r1 * nx, y1 + r1 * ny), (x2 + sign1 * r2 * nx, y2 + sign1 * r2 * ny)))

    return tangent_lines

def arc_length(center, lhs, rhs, radius, left):
    vec1 = (lhs[0] - center[0], lhs[1] - center[1])
    vec2 = (rhs[0] - center[0], rhs[1] - center[1])

    theta = math.atan2(vec2[1], vec2[0]) - math.atan2(vec1[1], vec1[0])
    if theta < -1e-6 and left:
        theta += 2.0 * math.pi
    elif theta > 1e-6 and not left:
        theta -= 2.0 * math.pi

    return abs(theta * radius)   

def left_circle_center(x, y, theta, r):
    h = x - r * math.sin(theta)
    k = y + r * math.cos(theta)
    return Circle(h,k, r)

def right_circle_center(x,y, theta, r):
    h = x + r * math.sin(theta)
    k = y - r * math.cos(theta)
    return Circle(h, k, r)

def dubins_curve_RSR(start_state,end_state, r_min):
    x0, y0, theta0 = start_state
    x1, y1, theta1 = end_state
    c1 = right_circle_center(x0, y0, theta0, r_min)
    c2 = right_circle_center(x1, y1, theta1, r_min)
    tangent_lines = tangent_line_construction(c1, c2)

    path_S_start_xy = tangent_lines[0][0]
    path_S_end_xy = tangent_lines[0][1]
    
    path_S_len = math.sqrt((path_S_start_xy[0] - path_S_end_xy[0])**2 + (path_S_start_xy[1] - path_S_end_xy[1])**2)
    arc1_len = arc_length((c1.x, c1.y), (start_state[0], start_state[1]), (path_S_start_xy[0], path_S_start_xy[1]), r_min, False)    
    arc2_len = arc_length((c2.x, c2.y), (path_S_end_xy[0], path_S_end_xy[1]), (end_state[0], end_state[1]),  r_min, False)
    dubins_curve_RSR_len = path_S_len + arc1_len + arc2_len
    return [dubins_curve_RSR_len, path_S_len, arc1_len, arc2_len]
    
def dubins_curve_LSR(start_state,end_state, r_min):
    x0, y0, theta0 = start_state
    x1, y1, theta1 = end_state
    c1 = left_circle_center(x0, y0, theta0, r_min)
    c2 = right_circle_center(x1, y1, theta1, r_min)
    tangent_lines = tangent_line_construction(c1, c2)

    path_S_start_xy = tangent_lines[3][0]
    path_S_end_xy = tangent_lines[3][1]
    
    path_S_len = math.sqrt((path_S_start_xy[0] - path_S_end_xy[0])**2 + (path_S_start_xy[1] - path_S_end_xy[1])**2)
    arc1_len = arc_length((c1.x, c1.y), (start_state[0], start_state[1]), (path_S_start_xy[0], path_S_start_xy[1]), r_min, True)    
    arc2_len = arc_length((c2.x, c2.y),  (path_S_end_xy[0], path_S_end_xy[1]), (end_state[0], end_state[1]), r_min, False)
    dubins_curve_LSR_len = path_S_len + arc1_len + arc2_len
    return [dubins_curve_LSR_len, path_S_len, arc1_len, arc2_len]

def dubins_curve_RSL(start_state,end_state, r_min):
    x0, y0, theta0 = start_state
    x1, y1, theta1 = end_state
    c1 = right_circle_center(x0, y0, theta0, r_min)
    c2 = left_circle_center(x1, y1, theta1, r_min)
    tangent_lines = tangent_line_construction(c1, c2)

    path_S_start_xy = tangent_lines[2][0]
    path_S_end_xy = tangent_lines[2][1]
    
    path_S_len = math.sqrt((path_S_start_xy[0] - path_S_end_xy[0])**2 + (path_S_start_xy[1] - path_S_end_xy[1])**2)
    arc1_len = arc_length((c1.x, c1.y), (start_state[0], start_state[1]), (path_S_start_xy[0], path_S_start_xy[1]), r_min, False)    
    arc2_len = arc_length((c2.x, c2.y),  (path_S_end_xy[0], path_S_end_xy[1]), (end_state[0], end_state[1]), r_min, True)
    dubins_curve_RSL_len = path_S_len + arc1_len + arc2_len
    return [dubins_curve_RSL_len, path_S_len, arc1_len, arc2_len]

def dubins_curve_LSL(start_state,end_state, r_min):
    x0, y0, theta0 = start_state
    x1, y1, theta1 = end_state
    c1 = left_circle_center(x0, y0, theta0, r_min)
    c2 = left_circle_center(x1, y1, theta1, r_min)
    tangent_lines = tangent_line_construction(c1, c2)

    path_S_start_xy = tangent_lines[1][0]
    path_S_end_xy = tangent_lines[1][1]
    
    path_S_len = math.sqrt((path_S_start_xy[0] - path_S_end_xy[0])**2 + (path_S_start_xy[1] - path_S_end_xy[1])**2)
    arc1_len = arc_length((c1.x, c1.y), (start_state[0], start_state[1]), (path_S_start_xy[0], path_S_start_xy[1]), r_min, True)    
    arc2_len = arc_length((c2.x, c2.y), (path_S_end_xy[0], path_S_end_xy[1]), (end_state[0], end_state[1]), r_min, True)
    dubins_curve_LSL_len = path_S_len + arc1_len + arc2_len
    return [dubins_curve_LSL_len, path_S_len, arc1_len, arc2_len]

def test_plot_RSR():
    start_state = (0, 1, np.pi/2)
    end_state = (4, 0, np.pi)
    [dubins_curve_RSR_len, path_S_len, arc1_len, arc2_len] = dubins_curve_RSR(start_state, end_state, 1)
    fig, ax = plt.subplots()
    c1 = right_circle_center(start_state[0], start_state[1], start_state[2], 1)
    c2= right_circle_center(end_state[0], end_state[1], end_state[2], 1)
    # Calculate the tangent lines
    tangent_lines = tangent_line_construction(c1, c2)
    # Plot the start and end state arrows
    ax.arrow(start_state[0], start_state[1], np.cos(start_state[2]), np.sin(start_state[2]), head_width=0.2, head_length=0.3, fc='blue', ec='blue')
    ax.arrow(end_state[0], end_state[1], np.cos(end_state[2]), np.sin(end_state[2]), head_width=0.2, head_length=0.3, fc='red', ec='red')
    # Plot the circles
    circle1 = plt.Circle((c1.x, c1.y), c1.r, color='red', fill=False)
    circle2 = plt.Circle((c2.x, c2.y), c2.r, color='green', fill=False)
    ax.add_artist(circle1)
    ax.add_artist(circle2)
    # Plot the tangent lines
    colors = ['blue', 'purple', 'orange', 'brown']
    #for i, line in enumerate(tangent_lines):
    #    ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], color=colors[i])
    ax.plot([tangent_lines[0][0][0], tangent_lines[0][1][0]], [tangent_lines[0][0][1], tangent_lines[0][1][1]], color='blue')
    # Set the aspect of the plot to equal to ensure the circles appear as circles
    ax.set_aspect('equal')

    # Set the x and y limits to zoom out
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    # Add a grid
    ax.grid(True)
    ax.set_xticks(np.arange(-10, 11, 1))
    ax.set_yticks(np.arange(-10, 11, 1))
    # Display the plot
    plt.show()

def test_plot_LSR():
    start_state = (0, 1, np.pi/2)
    end_state = (4, 0, np.pi)
    [dubins_curve_LSR_len, path_S_len, arc1_len, arc2_len] = dubins_curve_LSR(start_state, end_state, 1)
    fig, ax = plt.subplots()
    c1 = left_circle_center(start_state[0], start_state[1], start_state[2], 1)
    c2= right_circle_center(end_state[0], end_state[1], end_state[2], 1)
    # Calculate the tangent lines
    tangent_lines = tangent_line_construction(c1, c2)
    # Plot the start and end state arrows
    ax.arrow(start_state[0], start_state[1], np.cos(start_state[2]), np.sin(start_state[2]), head_width=0.2, head_length=0.3, fc='blue', ec='blue')
    ax.arrow(end_state[0], end_state[1], np.cos(end_state[2]), np.sin(end_state[2]), head_width=0.2, head_length=0.3, fc='red', ec='red')
    # Plot the circles
    circle1 = plt.Circle((c1.x, c1.y), c1.r, color='red', fill=False)
    circle2 = plt.Circle((c2.x, c2.y), c2.r, color='green', fill=False)
    ax.add_artist(circle1)
    ax.add_artist(circle2)
    # Plot the tangent lines
    colors = ['blue', 'purple', 'orange', 'brown']
    #for i, line in enumerate(tangent_lines):
    #    ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], color=colors[i])
    ax.plot([tangent_lines[3][0][0], tangent_lines[3][1][0]], [tangent_lines[3][0][1], tangent_lines[3][1][1]], color='brown')
    # Set the aspect of the plot to equal to ensure the circles appear as circles
    ax.set_aspect('equal')

    # Set the x and y limits to zoom out
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    # Add a grid
    ax.grid(True)
    ax.set_xticks(np.arange(-10, 11, 1))
    ax.set_yticks(np.arange(-10, 11, 1))
    # Display the plot
    plt.show()

def test_plot_RSL():
    start_state = (0, 1, np.pi/2)
    end_state = (4, 0, np.pi)
    [dubins_curve_RSL_len, path_S_len, arc1_len, arc2_len] = dubins_curve_RSL(start_state, end_state, 1)
    fig, ax = plt.subplots()
    c1 = right_circle_center(start_state[0], start_state[1], start_state[2], 1)
    c2= left_circle_center(end_state[0], end_state[1], end_state[2], 1)
    # Calculate the tangent lines
    tangent_lines = tangent_line_construction(c1, c2)
    # Plot the start and end state arrows
    ax.arrow(start_state[0], start_state[1], np.cos(start_state[2]), np.sin(start_state[2]), head_width=0.2, head_length=0.3, fc='blue', ec='blue')
    ax.arrow(end_state[0], end_state[1], np.cos(end_state[2]), np.sin(end_state[2]), head_width=0.2, head_length=0.3, fc='red', ec='red')
    # Plot the circles
    circle1 = plt.Circle((c1.x, c1.y), c1.r, color='red', fill=False)
    circle2 = plt.Circle((c2.x, c2.y), c2.r, color='green', fill=False)
    ax.add_artist(circle1)
    ax.add_artist(circle2)
    # Plot the tangent lines
    colors = ['blue', 'purple', 'orange', 'brown']
    #for i, line in enumerate(tangent_lines):
    #    ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], color=colors[i])
    ax.plot([tangent_lines[2][0][0], tangent_lines[2][1][0]], [tangent_lines[2][0][1], tangent_lines[2][1][1]], color='orange')
    # Set the aspect of the plot to equal to ensure the circles appear as circles
    ax.set_aspect('equal')

    # Set the x and y limits to zoom out
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    # Add a grid
    ax.grid(True)
    ax.set_xticks(np.arange(-10, 11, 1))
    ax.set_yticks(np.arange(-10, 11, 1))
    # Display the plot
    plt.show()

def test_plot_LSL():
    start_state = (0, 1, np.pi/2)
    end_state = (4, 0, np.pi)
    [dubins_curve_LSL_len, path_S_len, arc1_len, arc2_len] = dubins_curve_LSL(start_state, end_state, 1)
    fig, ax = plt.subplots()
    c1 = left_circle_center(start_state[0], start_state[1], start_state[2], 1)
    c2= left_circle_center(end_state[0], end_state[1], end_state[2], 1)
    # Calculate the tangent lines
    tangent_lines = tangent_line_construction(c1, c2)
    # Plot the start and end state arrows
    ax.arrow(start_state[0], start_state[1], np.cos(start_state[2]), np.sin(start_state[2]), head_width=0.2, head_length=0.3, fc='blue', ec='blue')
    ax.arrow(end_state[0], end_state[1], np.cos(end_state[2]), np.sin(end_state[2]), head_width=0.2, head_length=0.3, fc='red', ec='red')
    # Plot the circles
    circle1 = plt.Circle((c1.x, c1.y), c1.r, color='red', fill=False)
    circle2 = plt.Circle((c2.x, c2.y), c2.r, color='green', fill=False)
    ax.add_artist(circle1)
    ax.add_artist(circle2)
    # Plot the tangent lines
    #colors = ['blue', 'purple', 'orange', 'brown']
    #for i, line in enumerate(tangent_lines):
    #    ax.plot([line[0][0], line[1][0]], [line[0][1], line[1][1]], color=colors[i])
    ax.plot([tangent_lines[1][0][0], tangent_lines[1][1][0]], [tangent_lines[1][0][1], tangent_lines[1][1][1]], color='purple')
    # Set the aspect of the plot to equal to ensure the circles appear as circles
    ax.set_aspect('equal')

    # Set the x and y limits to zoom out
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    # Add a grid
    ax.grid(True)
    ax.set_xticks(np.arange(-10, 11, 1))
    ax.set_yticks(np.arange(-10, 11, 1))
    # Display the plot
    plt.show()

def LRL_trajectory(start_state, end_state, r_min):
    x0, y0, theta0 = start_state
    x1, y1, theta1 = end_state

    c1 = left_circle_center(x0, y0, theta0, r_min)
    c2 = left_circle_center(x1, y1, theta1, r_min)

    D = math.sqrt((c2.x - c1.x)**2 + (c2.y - c1.y)**2)

    if D < 4 * r_min:
        interior_theta = math.acos(D / (4 * r_min))
        theta = math.atan2(c2.y - c1.y, c2.x - c1.x) - interior_theta

        r_circle = (c1.x + 2.0 * r_min * math.cos(theta), c1.y + 2.0 * r_min * math.sin(theta))

        arcL1 = arc_length((c1.x, c1.y), start_state, (r_circle[0], r_circle[1]), r_min, True)
        arcL2 = arc_length(r_circle, (c1.x, c1.y), (c2.x, c2.y), r_min, False)
        arcL3 = arc_length((c2.x, c2.y), (c2.x, c2.y), end_state, r_min, True)

        # Plotting
        circle1 = plt.Circle((c1.x, c1.y), r_min, color='green', fill = False)
        circle2 = plt.Circle((c2.x, c2.y), r_min,color='red', fill = False)
        circle3 = plt.Circle(r_circle, r_min,color='blue', fill = False)
        fig, ax = plt.subplots()
        ax.add_artist(circle1)
        ax.add_artist(circle2)
        ax.add_artist(circle3)
        ax.arrow(x0, y0, 0.5 * math.cos(theta0), 0.5 * math.sin(theta0), head_width=0.05, head_length=0.1, fc='k', ec='k')
        ax.arrow(x1, y1, 0.5 * math.cos(theta1), 0.5 * math.sin(theta1), head_width=0.05, head_length=0.1, fc='k', ec='k')
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

        return arcL1, arcL2, arcL3
    else:
        return None

def RLR_trajectory(start_state, end_state, r_min):
    x0, y0, theta0 = start_state
    x1, y1, theta1 = end_state

    c1 = right_circle_center(x0, y0, theta0, r_min)
    c2 = right_circle_center(x1, y1, theta1, r_min)

    D = math.sqrt((c2.x - c1.x)**2 + (c2.y - c1.y)**2)

    if D < 4 * r_min:
        interior_theta = math.acos(D / (4 * r_min))
        theta = math.atan2(c2.y - c1.y, c2.x - c1.x) + interior_theta

        r_circle = (c1.x + 2.0 * r_min * math.cos(theta), c1.y + 2.0 * r_min * math.sin(theta))

        arcL1 = arc_length((c1.x, c1.y), start_state, (r_circle[0], r_circle[1]), r_min, False)
        arcL2 = arc_length(r_circle, (c1.x, c1.y), (c2.x, c2.y), r_min, True)
        arcL3 = arc_length((c2.x, c2.y), (c2.x, c2.y), end_state, r_min, False)

        # Plotting
        circle1 = plt.Circle((c1.x, c1.y), r_min, color='green', fill = False)
        circle2 = plt.Circle((c2.x, c2.y), r_min,color='red', fill = False)
        circle3 = plt.Circle(r_circle, r_min,color='blue', fill = False)
        fig, ax = plt.subplots()
        ax.add_artist(circle1)
        ax.add_artist(circle2)
        ax.add_artist(circle3)
        ax.arrow(x0, y0, 0.5 * math.cos(theta0), 0.5 * math.sin(theta0), head_width=0.05, head_length=0.1, fc='k', ec='k')
        ax.arrow(x1, y1, 0.5 * math.cos(theta1), 0.5 * math.sin(theta1), head_width=0.05, head_length=0.1, fc='k', ec='k')
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()

        return arcL1, arcL2, arcL3
    else:
        return None
RLR_trajectory((0, 1, np.pi/2), (3.999, 0, np.pi), 1 )
#LRL_trajectory((3.999, 0, np.pi/2), (0, 0, 1.5*np.pi), 1 )

def shortest_trajectory(R_min, Velocity, start_state, end_state):
    # Calculate all possible trajectories
    RSR = dubins_curve_RSR(start_state, end_state, R_min)
    LSR = dubins_curve_LSR(start_state, end_state, R_min)
    RSL = dubins_curve_RSL(start_state, end_state, R_min)
    LSL = dubins_curve_LSL(start_state, end_state, R_min)
    RLR = RLR_trajectory(start_state, end_state, R_min)
    LRL = LRL_trajectory(start_state, end_state, R_min)

    # Calculate the lengths of the trajectories
    lengths = [sum(RSR), sum(LSR), sum(RSL), sum(LSL)]
    if RLR is not None:
        lengths.append(sum(RLR))
    if LRL is not None:
        lengths.append(sum(LRL))

    # Find the shortest trajectory
    min_length = min(lengths)
    min_index = lengths.index(min_length)

    # Return the corresponding controls and durations
    if min_index == 0:
        return [("Right", RSR[0] / Velocity), ("Straight", RSR[1] / Velocity), ("Right", RSR[2] / Velocity)]
    elif min_index == 1:
        return [("Left", LSR[0] / Velocity), ("Straight", LSR[1] / Velocity), ("Right", LSR[2] / Velocity)]
    elif min_index == 2:
        return [("Right", RSL[0] / Velocity), ("Straight", RSL[1] / Velocity), ("Left", RSL[2] / Velocity)]
    elif min_index == 3:
        return [("Left", LSL[0] / Velocity), ("Straight", LSL[1] / Velocity), ("Left", LSL[2] / Velocity)]
    elif min_index == 4:
        return [("Right", RLR[0] / Velocity), ("Left", RLR[1] / Velocity), ("Right", RLR[2] / Velocity)]
    else:
        return [("Left", LRL[0] / Velocity), ("Right", LRL[1] / Velocity), ("Left", LRL[2] / Velocity)]
test_plot_LSL()
"""
draw_rrt_movie.py

Author: Chang-Hong Chen
Email: longhongc@gmail.com
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import os
import sys

if (len(sys.argv) != 8):
    print("Error: requires problem number, start x, start y, start theta, "
       "goal x, goal y, goal tolerance radius")
    exit()

problem_number = sys.argv[1]
start = (float(sys.argv[2]), float(sys.argv[3]))
start_theta = (float(sys.argv[4]))
goal = (float(sys.argv[5]), float(sys.argv[6]))
goal_tolerance = float(sys.argv[7])

# Create plot
fig, ax = plt.subplots(dpi=800)
ax.set_aspect('equal')

# Set limits and axis label
ax.set_xlim((-50, 50))
ax.set_ylim((-50, 50))
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')

# Draw start and goal
goal_circle = plt.Circle(goal, goal_tolerance, color='gold', fill=False)
ax.add_patch(goal_circle)

# Draw obstacles
obstacles = []
obstacles_file_path = './data/obstacles.txt'
line_count = 0
for line in open(obstacles_file_path, 'r'):
    line_count += 1
    if (line_count == 1):
        continue
    obstacles.append([float(s) for s in line.split(',')])

for obstacle in obstacles:
    x, y, r = obstacle
    circle = plt.Circle((x, y), r, color='r')
    ax.add_patch(circle)

problem_dir = './results/Problem' + \
    str(problem_number) + '/'

# Draw path
path_file_path = problem_dir + \
    'Problem' + str(problem_number) + '_path.txt'

path = []
x1, y1 = start
for line in open(path_file_path, 'r'):
    coords = [float(s) for s in line.split(',')]
    path.append(coords)
    t, x2, y2, theta2, v, w, a, r = coords
    plt.plot([x1, x2], [y1, y2], color='lime', linestyle='--', linewidth=1)
    x1, y1 = x2, y2

# Draw robot
robot = []
robot_file_path = './data/H3_robot.txt'
for line in open(robot_file_path, 'r'):
    robot.append([float(s) for s in line.split(',')])

def rigid_transform(dx, dy, dtheta, target):
    translation = np.array([[1, 0, dx],
                            [0, 1, dy],
                            [0, 0,  1]])

    rotation = np.array([[math.cos(dtheta), -math.sin(dtheta), 0],
                         [math.sin(dtheta),  math.cos(dtheta), 0],
                         [               0,                 0, 1]])

    return translation @ rotation @ target

robot_homo_coord = np.hstack((np.array(robot), np.ones((len(robot), 1)))).T

start_x, start_y = start
transformed_robot = rigid_transform(
        start_x, start_y, start_theta, robot_homo_coord).T

for point in transformed_robot:
    x, y, _ = point
    circle = plt.Circle((x, y), 0.1, color='black', zorder=3)
    ax.add_patch(circle)

curr_index = 0
def animate(time):
    global curr_index
    for index in range(curr_index, len(path)):
        t, x, y, theta, v, w, a, r = path[index]
        # For every 2 seconds, draw robot
        if (t > time * 2):
            curr_index = index + 1
            transformed_robot = rigid_transform(
                x, y, theta, robot_homo_coord).T

            for point in transformed_robot:
                x, y, _ = point
                circle = plt.Circle((x, y), 0.1, color='black', zorder=3)
                ax.add_patch(circle)

            break


max_time = int(path[-1][0] / 2)
anim = animation.FuncAnimation(fig, animate, frames=max_time, interval=10)

# Save gif
os.makedirs('./results', exist_ok=True)
image_file = problem_dir + \
    'Problem' + str(problem_number) + '_graph.gif'

anim.save(image_file, writer='pillow')

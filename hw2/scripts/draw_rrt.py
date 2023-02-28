"""
draw_rrt.py

Author: Chang-Hong Chen
Email: longhongc@gmail.com
"""

import matplotlib.pyplot as plt
import os
import sys

if (len(sys.argv) != 7):
    print("Error: requires problem number, start x, start y, "
       "goal x, goal y, goal tolerance radius")
    exit()

problem_number = sys.argv[1]
start = (float(sys.argv[2]), float(sys.argv[3]))
goal = (float(sys.argv[4]), float(sys.argv[5]))
goal_tolerance = float(sys.argv[6])

# Create plot
fig, ax = plt.subplots(dpi=300)
ax.set_aspect('equal')

# Set limits and axis label
ax.set_xlim((-50, 50))
ax.set_ylim((-50, 50))
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')

# Draw start and goal
start_circle = plt.Circle(start, 1, color='lime', fill=False)
goal_circle = plt.Circle(goal, goal_tolerance, color='gold', fill=False)
ax.add_patch(start_circle)
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

# Draw search tree
problem_dir = './results/Problem' + \
    str(problem_number) + '/'

search_tree_file_path = problem_dir + \
    'Problem' + str(problem_number) + '_search_tree.txt'

for line in open(search_tree_file_path, 'r'):
    coords = [float(s) for s in line.split(',')]
    x1, y1, x2, y2 = coords[0], coords[1], coords[2], coords[3]
    plt.scatter([x1, x2], [y1, y2], marker='o', color='black', s=0.3)
    plt.plot([x1, x2], [y1, y2], color='black', linewidth=0.2)

# Draw path
path_file_path = problem_dir + \
    'Problem' + str(problem_number) + '_path.txt'

x1, y1 = start
for line in open(path_file_path, 'r'):
    coords = [float(s) for s in line.split(',')]
    x2, y2 = coords[0], coords[1]
    plt.plot([x1, x2], [y1, y2], color='lime', linestyle='--', linewidth=1)
    x1, y1 = x2, y2

# Save image
os.makedirs('./results', exist_ok=True)
image_file = problem_dir + \
    'Problem' + str(problem_number) + '_graph.png'
fig.savefig(image_file)

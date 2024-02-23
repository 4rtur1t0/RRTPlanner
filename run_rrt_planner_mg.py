#!/usr/bin/env python
# encoding: utf-8
"""
Find a path in a 2D environment with obstacles using RRT.
- RRT Planning with multiple goals.
- A holonomic robot is considered.
- The robot is considered a point. The robot is inside an obstacle if the point lies on any of the specified circles.

Two slight modifications are performed over the stantard RRT:
    a) The algorithm looks for a path to any of the goals specified.
    b) The algorithm stops when a path is found for any of the goals.
    c) The algorithm returns, always, as a solution:
        b.1) The paths that connects the start and one of the goals.
        b.2) If no path was found, the algorithm finds the closest node to any of the goals and returns the path that
        connects the nearest neighbour node with the start node.

In the RRTPlannerMG a solution is always found by the planner, either  by reaching any of the goals or
by finding the closest node to any of the goals. This, of course, is a desired behaviour, since the robot
has always a movement plan to carry out.


@Authors: Arturo Gil
@Time: April 2024
"""
from rrtplanner.rrtplannermg import RRTPlannerMG


def find_path():
    # define the dimensions +-X, +-Y of the map
    dimensions = [30, 30]
    # start and end position
    start = [-8, -8]
    goals = [[10, 15],
             [15, 15],
             [18, 18]]
    # define a list of obstacles as (x, y, Radius)
    obstacles = [[0, 0, 6],
                 [0, 15, 6],
                 [5, 5, 6],
                 [10, -5, 6],
                 [15, -10, 6],
                 [0, -3, 6]]
    # Create Multiple Goal RRT planner
    planner = RRTPlannerMG(start=start,
                           goals=goals,
                           dimensions=dimensions,
                           obstacles=obstacles,
                           epsilon=1,
                           max_nodes=10000)
    planner.build_rrt()
    planner.print_info()
    path_found = planner.get_path()
    print('Optimal path: ', path_found)
    planner.plot_tree()
    planner.plot_solution(show=True)
    print('FINISHED')


if __name__ == "__main__":
    find_path()


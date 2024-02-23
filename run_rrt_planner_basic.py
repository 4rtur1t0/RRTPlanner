#!/usr/bin/env python
# encoding: utf-8
"""
Find a path in a 2D environment using RRT.
A holonomic robot is considered.
The robot is considered a point. The robot is inside an obstacle if the point lies on any of the specified circles.

Two slight modifications are performed over the stantard RRT:
    a) The algorithm returns, always, as a solution:
        b.1) The path to the goal. This is the first path found by the algorithm.
             Of course, it may not be the shortest path.
        b.3) If the path is nof found in max_nodes, iterations. The algorithm looks for the closest node to
             the goal. Then, the path that connects the start node and the nearest neighbour node is
             returned.

In the RRTPlannerBasic class, a solution is always found by the planner, either  by reaching the goal or
by finding the closest node to the goal. This, of course, is a desired behaviour, since the robot
has always a movement plan to carry out.


@Authors: Arturo Gil
@Time: April 2024
"""
from rrtplanner.rrtplannerbasic import RRTPlannerBasic


def find_path():
    # define the dimensions +-X, +-Y of the map
    dimensions = [20, 20]
    # start and end position
    start = [-8, -8]
    goal = [15, 15]
    # define a list of obstacles as (x, y, Radius)
    obstacles = [[0, 0, 3],
                 [0, 15, 5],
                 [5, 5, 6],
                 [15, -10, 5],
                 [0, -3, 2]]
    # a solution is always found by the planner, either  by
    # reachin the goal or by offering
    planner = RRTPlannerBasic(start=start,
                              goal=goal,
                              dimensions=dimensions,
                              obstacles=obstacles,
                              epsilon=1,
                              max_nodes=1500)
    planner.build_rrt()
    planner.print_info()
    path_found = planner.get_path()
    print('Optimal path: ', path_found)
    planner.plot_tree()
    planner.plot_solution(show=True)
    print('FINISHED')


if __name__ == "__main__":
    find_path()


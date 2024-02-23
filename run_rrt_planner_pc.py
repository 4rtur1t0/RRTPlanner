#!/usr/bin/env python
# encoding: utf-8
"""
Find a path on a point cloud.
Each point in the pointcloud is classified as traversable or non-traversable.
We are looking for a 3D trajectory using RRT, with the following characteristics:
    - The robot (sphere), should have traversable points, ensuring that there exists a surface to navigate on.
    - In addition the robot (sphere), should not have non-traversable points, considered as obstacles.
    - An RRT Planning with multiple goals considered.
    - A holonomic robot is considered.

- Under development:
    - Find a smooth trajectory.
    - Integrate RRT CONNECT.
    - Integrate max delta(phi) between nodes in the tree.
    - Integrate max delta(Z) between the points.

    - Probabilistic roadmap.

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
from keyframe.keyframe import KeyFrame
from rrtplanner.rrtplannerPC import RRTPlannerPC


def find_path():
    # simulation: must subscribe to lidar and compute traversability
    pointcloud = KeyFrame('dataset/robot0/lidar/001.pcd')
    # pointcloud.draw_cloud()
    points_traversable, points_obstacle = pointcloud.compute_traversability()

    # start and goals
    start = [0, 0, 0]
    # goals should be projected to the local reference system from the global UTM coordinates
    goals = [[10, 15, 0],
             [15, 15, 0],
             [18, 18, 0]]
    # Create Multiple Goal RRT planner
    planner = RRTPlannerPC(start=start,
                           goals=goals,
                           pc_traversable=points_traversable,
                           pc_obstacle=points_obstacle,
                           epsilon=1,
                           max_nodes=10000)
    planner.plot_tree(show=True)
    planner.build_rrt()
    planner.print_info()
    path_found = planner.get_path()
    print('Optimal path: ', path_found)
    planner.plot_tree()
    planner.plot_solution(show=True)
    print('FINISHED')


if __name__ == "__main__":
    find_path()


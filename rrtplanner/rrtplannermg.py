"""
    The main class RRTPlannerMG and the node class.
    The code is very similar to the RRTPlannerBasic. Both codes are maintained separately for Educational purposes.
    This constitutes a demo of a 2D planning of a holonomic robot with circle-like obstacles.
    A set of circle-like obstacles are considered.

    In this case, compared to RRTPlannerBasic, RRTPlannerMG considers multiple goals. The algorithm stops whenever any
    of the goals is reached.

    Please, read:
    - RRT-connect: An efficient approach to single-query path planning. J. J. Kuffner and S. M. LaValle. In Proceedings IEEE International Conference on Robotics and Automation, pages 995-1001, 2000. [pdf].
    - Rapidly-exploring random trees: A new tool for path planning. S. M. LaValle. TR 98-11, Computer Science Dept., Iowa State University, October 1998, [pdf].

    The minor variation with respect to the basic algorithm is that, after max_nodes iterations, the
    algorithm always returns a solution:
        a) If any the goals is reached, the path that connects the start and goal is returned.
        b) If no goal reached, the node that is found closest to any of the goals goal is found.
           Next, the path that connects the start and the closest node is returned.

    This behaviour is not optimal, however, it yields, sometimes, necessary to have always
    a local path to follow in the context of mobile robotics. It yields probable, that, in the next
    observations, a valid path is found that allows the robot to reach the goal.
"""
import numpy as np
import matplotlib.pyplot as plt


class RRTPlannerMG():
    """
    A Basic RRT Planner in a 2D space for holonomic robots.
    RRTPlannerMG considers multiple-goals, instead of just one as the basic RRT.
    """
    def __init__(self, start, goals, dimensions, obstacles, epsilon, max_nodes):
        self.goals = np.array(goals)
        self.start = np.array(start)
        self.max_nodes = max_nodes
        self.epsilon = epsilon
        # space, +- dimensions in XY
        self.dim = np.array(dimensions)
        # a set of circle-like obstacles, (x, y, R)
        self.obstacles = np.array(obstacles)
        # stores the nodes of the tree
        self.tree = []
        # stores info on the result of the algorithm
        self.goal_reached = False
        self.iterations_performed = 0

        if self.in_obstacle(self.start):
            raise Exception('The start coordinates are inside the obstacle space')

        for i in range(self.goals.shape[0]):
            if self.in_obstacle(self.goals[i]):
                raise Exception('One of the goals is inside the obstacle space')

        # add root node to tree: start the root of the tree
        self.add_vertex(parent_id=0, coordinates=start)

    def build_rrt(self):
        for k in range(self.max_nodes):
            print('Iteration: ', k)
            qrand = self.random_config()
            reached = self.extend(qrand)
            if reached:
                self.goal_reached = True
                self.iterations_performed = k
                break
            # self.plot_tree()

    def add_vertex(self, parent_id, coordinates, reached=False):
        # new id depending on the number of nodes
        id = len(self.tree)
        node = Node(id=id, parent_id=parent_id, coordinates=coordinates, reached=reached)
        self.tree.append(node)

    def random_config(self):
        """
        This generates a new random configuration
        :return:
        """
        x = -self.dim[0] + 2*self.dim[0]*np.random.rand()
        y = -self.dim[1] + 2*self.dim[1] * np.random.rand()
        return np.array([x, y])

    def extend(self, qrand):
        """
        The extend operation of the tree.
        :param qrand:
        :return:
        """
        inear, qnear = self.nearest_neighbour(qrand)
        qnew = self.new_config(qnear, qrand)
        if not self.in_obstacle(qnew):
            # if not in collision, add node to the tree
            reached = self.reached_goal(qnew)
            self.add_vertex(parent_id=inear, coordinates=qnew, reached=reached)
            if reached:
                return True
        return False

    def new_config(self, qnear, qrand):
        """
        Computes a new configuration between qnear and qrand that is placed at an epsilon distance from qnear
        :param qnear:
        :param qrand:
        :return:
        """
        ds = qrand - qnear
        phi = np.arctan2(ds[1], ds[0])
        du = np.array([np.cos(phi), np.sin(phi)])
        qnew = qnear + self.epsilon*du
        return qnew

    def in_obstacle(self, q):
        """
        Returns True if q is in the obstacle space, false elsewise
        :param q:
        :return:
        """
        for i in range(len(self.obstacles)):
            obstacle = self.obstacles[i]
            d = np.linalg.norm(q-obstacle[0:2])
            if d < obstacle[2]:
                return True
        return False

    def nearest_neighbour(self, qrand):
        """
        Returns the index in the tree that is nearest to qrand.
        Also returns the coordinates of the nearest index
        :param qrand:
        :return:
        """
        coords = []
        # build a list to compute distance fast with numpy?
        for i in range(len(self.tree)):
            coords.append(self.tree[i].coordinates)
        coords = np.array(coords)
        dists = np.linalg.norm(coords - qrand, axis=1)
        index_nn = np.argmin(dists)
        return index_nn, self.tree[index_nn].coordinates

    def closest_node_to_goals(self):
        """
        Returns the index in the tree that is nearest to qrand
        :param qrand:
        :return:
        """
        distances = []
        indices_nn = []
        for i in range(self.goals.shape[0]):
            index_nn, coords_nn = self.nearest_neighbour(self.goals[i])
            d = self.distance(coords_nn, self.goals[i])
            distances.append(d)
            indices_nn.append(index_nn)
        i = np.argmin(np.array(distances))
        j = indices_nn[i]
        return self.tree[j]

    def closest_goal_to_node(self, node):
        """
        Returns the index in the tree that is nearest to qrand
        :param qrand:
        :return:
        """
        distances = []
        for i in range(self.goals.shape[0]):
            d = self.distance(node.coordinates, self.goals[i])
            distances.append(d)
        i = np.argmin(np.array(distances))
        return self.goals[i]

    def reached_goal(self, q):
        """
        Checks if the coordinates in q have reached any of the goals. A goal is reached if q is within a radius epsilon
        around any of the goals.
        :param q:
        :return:
        """
        for i in range(self.goals.shape[0]):
            # d = np.linalg.norm(q-self.goals[i])
            d = self.distance(q, self.goals[i])
            if d < self.epsilon:
                return True
        return False

    def distance(self, qa, qb):
        d = np.linalg.norm(qa-qb)
        return d

    def get_path(self):
        """
        This method:
        a) looks for the node/nodes that are marked as "reached the goal"
        b) for each node marked as reached, backtraces the path until the root is found.
        c) The path is returned.
        If no node is found as goal_reached, then the closest node to the goal is found and treated as the solution node
        :return:
        """
        solution_node = self.find_node_reached()
        # if no solution is found, then find the nearest neighbour in the tree to the goal
        if solution_node is None:
            solution_node = self.closest_node_to_goals()
            solution_path = self.back_trace_path_from_node(solution_node)
        else:
            # standard solution
            solution_path = self.back_trace_path_from_node(solution_node)
            goal = self.closest_goal_to_node(solution_node)
            # in this case (reached goal), the goal coordinates are inserted at the beginning
            solution_path.insert(0, np.array(goal))

        # change order of the path!!
        solution_path.reverse()
        solution_path = np.array(solution_path)
        return solution_path

    def find_node_reached(self):
        """
        Finds the node that reached the target. We must iterate through all the nodes in the tree. However, doing
        it in reverse order is probably faster
        :return:
        """
        for i in reversed(range(len(self.tree))):
            if self.tree[i].goal_reached:
                return self.tree[i]
        return None

    def back_trace_path_from_node(self, node):
        path = []
        while True:
            path.append(node.coordinates)
            # this happens only at the root node with id==parent_id==0
            if node.id == node.parent_id:
                break
            # backtrace the node
            node = self.tree[node.parent_id]
        return path

    def print_info(self):
        print(30*'*')
        print('PERFORMED ITERATIONS: ', self.iterations_performed)
        print('GOAL REACHED: ', self.goal_reached)
        print(30 * '*')

    def plot_tree(self, show=False):
        # plot obstacles
        theta = np.linspace(0, 2 * np.pi, 150)
        xobs = np.array([])
        yobs = np.array([])
        for i in range(len(self.obstacles)):
            obstacle = self.obstacles[i]
            x = obstacle[2] * np.cos(theta) + obstacle[0]
            y = obstacle[2] * np.sin(theta) + obstacle[1]
            xobs = np.append(xobs, x)
            yobs = np.append(yobs, y)
        obspoints = np.column_stack((xobs, yobs))
        # plot the edges of the tree
        for i in range(len(self.tree)):
            coords = self.tree[i].coordinates
            parent_id = self.tree[i].parent_id
            coords_parent = self.tree[parent_id].coordinates
            c = np.vstack((coords, coords_parent))
            plt.plot(c[:, 0], c[:, 1], color='magenta')
        # plot vertices (node coordinates) as scatter
        coords = []
        # leave out i=0, since it corresponds to the start node
        for i in range(1, len(self.tree)):
            coords.append(self.tree[i].coordinates)
        coords = np.array(coords)

        # plot obstacles
        plt.scatter(obspoints[:, 0], obspoints[:, 1], color='black')
        # plot start
        plt.scatter(self.start[0], self.start[1], color='green')
        # plot goals
        for i in range(self.goals.shape[0]):
            goal = self.goals[i]
            plt.scatter(goal[0], goal[1], color='red')
        # lastly plot all nodes in the tree
        plt.scatter(coords[:, 0], coords[:, 1], color='blue')
        if show:
            plt.show()

    def plot_solution(self, show=True):
        solution_path = self.get_path()
        solution_path = np.array(solution_path)
        K = solution_path.shape[0]
        for i in range(K-1):
            coords = solution_path[i, :]
            coordsn = solution_path[i+1, :]
            c = np.vstack((coords, coordsn))
            plt.plot(c[:, 0], c[:, 1], color='cyan')
        if show:
            plt.show()


class Node():
    """
    A node in the tree, storing its id, parent_id and coordinates.
    """
    def __init__(self, id, parent_id, coordinates, reached=False):
        self.id = id
        self.parent_id = parent_id
        self.coordinates = coordinates
        self.goal_reached = reached

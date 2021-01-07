"""

Path planning Sample Code with RRT*

author: Atsushi Sakai(@Atsushi_twi)

"""

import math
import numpy as np
#sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../RRT/")

try:
    from rrt import RRT
except ImportError:
    raise

show_animation = True


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y, z):
            super().__init__(x, y, z)
            self.cost = 0.0

    def __init__(self,
                 obstacle_list,
                 searchTheta=3.14/3,
                 expand_dis=30.0,
                 path_resolution=1.0,
                 goal_sample_rate=20,
                 max_iter=300,
                 connect_circle_dist=50.0,
                 search_until_max_iter=False):
        """
        Setting Parameter

        start:Start Position [x,y,z]
        goal:Goal Position [x,y,z]
        obstacleList:obstacle Positions [[x,y,z,size],...]
        randArea:Random Sampling Area [min,max]

        """
        super().__init__(obstacle_list, expand_dis,
                         path_resolution, goal_sample_rate, max_iter)
        self.connect_circle_dist = connect_circle_dist
        self.search_until_max_iter = search_until_max_iter
        self.searchTheta = searchTheta

    #edit for 3d
    def planning(self,start, goal):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """
                
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1],  goal[2])
        self.goal_node = self.Node(goal[0], goal[1], goal[2])
        #setup for conical search
        self.goalDist, goalTheta, goalPhi = self.calc_distance_and_angle(self.start,self.end)
        self.goalDist *= 1.1
        self.goalDir = np.array([(goal[0]-start[0])/self.goalDist, (goal[1]-start[1])/self.goalDist, (goal[2]-start[2])/self.goalDist])
        self.R = np.array([[math.cos(goalTheta)*math.cos(goalPhi), -math.sin(goalTheta), math.cos(goalTheta)*math.sin(goalPhi)],
                         [math.sin(goalTheta)*math.cos(goalPhi), math.cos(goalTheta), math.sin(goalTheta)*math.sin(goalPhi)],
                         [-math.sin(goalPhi), 0, -math.sin(goalPhi)]])
        self.R = np.linalg.inv(self.R)
        self.constSinTheta = math.sin(goalTheta/2)
        
        
        for (ox, oy, oz, dx, dy, dz) in self.obstacle_list:
            ex = goal[0]-ox
            ey = goal[1]-oy
            ez = goal[2]-oz                  
            if ex >= 0 and ex <= dx and ey >= 0 and ey <= dy and ez >= 0 and ez <= dz:
                print("Goal is in a box")
                return None
        
        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            
            #generate random node
            rnd = self.get_random_node()
            
            #find nearest node
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            
            #connect nearest node to the new node
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)
            near_node = self.node_list[nearest_ind]
            
            #calculate cost for nearest node
            new_node.cost = self.calc_new_cost(near_node, new_node)

            #check collision and connect new node to tree structure
            if self.check_collision(new_node, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            #check for the goal
            if ((not self.search_until_max_iter) and new_node):  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    #edit for 3d
    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y, n.z) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    #edit for 3d
    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the tree that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 + (node.z - new_node.z)**2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    #edit for 3d
    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.z = edge_node.z
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.path_z = edge_node.path_z
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    #edit for 3d
    def calc_new_cost(self, from_node, to_node):
        d, _, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

"""

Path planning Sample Code with Randomized Rapidly-Exploring Random Trees (RRT)

author: AtsushiSakai(@Atsushi_twi)

"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z
            self.path_x = []
            self.path_y = []
            self.path_z = []
            self.parent = None

    #editted for 3d
    def __init__(self,
                 obstacle_list,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500):
        """
        Setting Parameter

        start:Start Position [x,y,z]
        goal:Goal Position [x,y,z]
        obstacleList:obstacle Positions [[x,y,z,size],...]
        randArea:Random Sampling Area [min,max]

        """
        
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.imposible = False
        
    #editted for 3d
    def planning(self, animation=False, collision=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """
        
        if(self.imposible == True):
            return None
        
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if collision and self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y,
                                      self.node_list[-1].z) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    #editted for 3d
    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y, from_node.z)
        d, theta, phi = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_z = [new_node.z]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta) * math.cos(phi)
            new_node.y += self.path_resolution * math.sin(theta) * math.cos(phi)
            new_node.z += self.path_resolution * math.sin(phi)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
            new_node.path_z.append(new_node.z)

        d, _, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.path_z.append(to_node.z)
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.z = to_node.z

        new_node.parent = from_node
        #print(new_node.x, new_node.y, new_node.z)

        return new_node

    #editted for 3d
    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.z]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.z])
            node = node.parent
        path.append([node.x, node.y, node.z])

        return path

    #editted for 3d
    def calc_dist_to_goal(self, x, y, z):
        dx = x - self.end.x
        dy = y - self.end.y
        dz = z - self.end.z
        return math.hypot(math.hypot(dx,dy), dz)
    
    #editted for 3d
    def get_random_node(self):
        """if random.randint(0, 100) > self.goal_sample_rate:
            
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.z)"""
            
        #TODO generate in a cone
        L = random.uniform(0, self.goalDist)
        r = random.uniform(0, L*self.constSinTheta)
        p = random.uniform(0, 6.28)     #angle aroung goalDir
        cone_center = self.goalDir*L
        rNode = np.matmul(self.R,np.array([0, r*math.cos(p), r*math.sin(p)])) + cone_center
        rnd = self.Node(rNode[0], rNode[1], rNode[2])      
        
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    #editted for 3d
    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 + (node.z - rnd_node.z)**2
                 for node in node_list]
 
        return dlist.index(min(dlist))

    #editted for 3d, need configuring for square obstacles
    @staticmethod
    def check_collision(node, obstacleList):
        
        if node is None:
            print("Geen nits")
            return False

        #calculate the distance between obstacle edges and node
        for (ox, oy, oz, dx, dy, dz) in obstacleList:
            dx_list = [x - ox for x in node.path_x]
            dy_list = [y - oy for y in node.path_y]
            dz_list = [z - oz for z in node.path_z]
            
            for ex,ey,ez in zip(dx_list, dy_list, dz_list):
                if ex >= 0 and ex <= dx and ey >= 0 and ey <= dy and ez >= 0 and ez <= dz:
                    return False
                
        return True  # safe

    #editted for 3d
    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        d = math.sqrt(dx**2 + dy**2 + dz**2)
        theta = math.atan2(dy, dx)
        phi = math.atan2(dz, math.hypot(dx, dy))
        return d, theta, phi

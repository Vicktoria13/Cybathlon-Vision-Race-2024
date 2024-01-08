import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

import numpy as np
import matplotlib.pyplot as plt
import random
import math


CAM_RESOLUTION = 0.05 #1 cellule de l'occupany = 5cm
PILOT_RADIUS = 2
CALC_RESOLUTION = 1

GOAL_X = 3.0/CAM_RESOLUTION
GOAL_Y = 0.5/CAM_RESOLUTION

show_animation = True



class Dijkstra_publisher (Node) :

    posX = 0.0
    posY = 0.0
    ortW = 0.0
    ortZ = 1.0

    og_width = 0
    og_height = 0
    grid = []

    min_x = 0.0
    min_y = 0.0
    max_x = 0.0
    max_y = 0.0

    resolution = 0.0
    rr = 0.0

    motion = []

    obstacle_map = None

    angle = 0.0
    theta = 0.0


    def __init__(self):
        super().__init__('dijkstra_publisher')

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/zed2/zed_node/pose',
            self.pose_callback,
            10)

        self.og_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.og_callback,
            10)

        self.ort_publisher = self.create_publisher(
            String,
            '/string_dijkstra',
            1)
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    class Node_:

        cost = 0
        x = 0
        y = 0
        parent_index = 0

        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)
    

    def timer_callback(self):
        msg = String()
        msg.data = '0'

        # trouver la position suivante du robot 
        #ntx_angle = 0
        ntx_angle = self.main_planning()

        if (ntx_angle >= 67):
            msg.data = '3h'
        elif (ntx_angle >= 22):
            msg.data = '1h30'
        elif (ntx_angle >= -22):
            msg.data = '12h'
        elif (ntx_angle >= -67):
            msg.data = '10h30'
        elif (ntx_angle >= -112):
            msg.data = '9h'
        
        self.ort_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.get_logger().info('Angle: "%s"' % self.angle)
        self.get_logger().info('Orientation: "%s"' % self.theta)



    def pose_callback(self, msg : PoseStamped):
        #self.get_logger().info('pose callback start')

        self.posX = msg.pose.position.x
        self.posY = msg.pose.position.y
        self.ortZ = msg.pose.orientation.z
        self.ortW = msg.pose.orientation.w

        if self.ortZ<0 :
            self.theta = -2*math.acos(self.ortW)
        else:
            self.theta = 2*math.acos(self.ortW)

        self.theta = self.theta*180/math.pi


    def og_callback(self, msg : OccupancyGrid):
        #self.get_logger().info('occupancy grid callback start')
        self.og_width = msg.info.width
        self.og_height = msg.info.height
        print("og_width : ", self.og_width)
        print("og_height : ", self.og_height)
        #self.grid = np.flip(np.array(msg.data).reshape((self.og_height, self.og_width)), axis=0)
        
        self.grid = []
        for i in range (0,self.og_height):
            self.grid.append([])
            for j in range(0,self.og_width):
                self.grid[i].append(msg.data[i*self.og_width+j])

    def main_planning(self):
        #print(__file__ + " start!!")

        # start and goal position
        #sx = self.posX  # [m]
        #sy = self.posY  # [m]
        sx = self.posX/CAM_RESOLUTION
        sy = self.posY/CAM_RESOLUTION + self.og_height/2 #+ .5/CAM_RESOLUTION
        gx = GOAL_X  # [m]
        gy = GOAL_Y  # [m]
        grid_size = 3 # [m]
        robot_radius = PILOT_RADIUS          # [m]


        #print("Position Actuelle : ", sx, ",", sy)

        pos_prec_x = sx # calcul de la position précédente, on l'initialise comme cela pour diriger le pilote directement en face de la course
        pos_prec_y = sy-1

        # set obstacle positions
        occupancyMap2D = self.grid
        
        ox, oy = [], []
        ox = self.obstacleOx(occupancyMap2D, ox)
        oy = self.obstacleOy(occupancyMap2D, oy)
        
        self.resolution = grid_size
        self.rr = robot_radius
        self.motion = self.get_motion_model()
        if (len(ox) != 0) and (len(oy) != 0):
            self.calc_obstacle_map(ox, oy)

        if show_animation:  # pragma: no cover
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")

        rx, ry = self.planning(sx, sy, gx, gy)

        if (len(rx) >= 7):
            pos_suiv_x = rx[len(rx)-7]
            pos_suiv_y = ry[len(ry)-7]
        else :
            pos_suiv_x = rx[len(rx)-2]      # position suivante que l'on devra atteindre
            pos_suiv_y = ry[len(ry)-2]



        #self.angle = math.floor(signe*(180-math.acos((d1**2+d2**2-d3**2)/(2*d1*d2))*180/math.pi))    # formule al kashi
        self.angle = (math.atan((pos_suiv_y-self.posY)/(pos_suiv_x-self.posX)))

        self.angle = self.angle*180/math.pi
        #print(self.angle)

        angle2 = self.theta + self.angle

        if show_animation:  # pragma: no cover
            #print(rx)
            #print(ry)
            plt.plot(rx, ry, "-r")
            plt.pause(0.001)
            # plt.show()    
        plt.clf()
        return angle2
    
    """PAS DE FONCTION OBSTACLE"""
    """DIFF"""
    def obstacleOx(self, occupancyMap, ox):
        if len(occupancyMap) != 0:
            for i in range(0,self.og_height):
                for j in range(0,self.og_width):

                    if self.grid[i][j] >= 50:
                        
                        ox.append(j)
        return ox
    
    """DIFF"""
    def obstacleOy(self, occupancyMap, oy):
        if len(occupancyMap) != 0:
            for i in range(0,self.og_height):
                for j in range(0,self.og_width):
                    if self.grid[i][j] >= 50:
                        oy.append(i)
        return oy    

    """Same"""
    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node_(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node_(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        while True:
            if len(open_set) == 0:
                #print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                #print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node_(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry
    
    """Same"""
    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.og_width*CAM_RESOLUTION + (node.x - self.min_x)
    
    """Same"""
    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)
    
    """Same"""
    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d
    
    """Same"""
    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        #self.get_logger().info('entering calc_grid_position()')

        pos = index * self.resolution + min_position
        return pos
    
    """Same"""
    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
    
    """TYPE DE WIDTH ET HEIGHT ?? Ptete pas des pixels"""
    def calc_obstacle_map(self, ox, oy):
        #self.get_logger().info('entering calc_obstacle_map()')

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        #print("min_x:", self.min_x)
        #print("min_y:", self.min_y)
        #print("max_x:", self.max_x)
        #print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        #print("x_width:", self.x_width)
        #print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(0, self.x_width, CALC_RESOLUTION):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(0, self.y_width, CALC_RESOLUTION):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break


    """Same"""
    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if node.x < self.x_width and node.y < self.y_width:
            if self.obstacle_map[node.x][node.y]:
                return False

        return True
    
    """Same"""
    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry



def main(args=None):
    rclpy.init(args=args)

    dks_publisher = Dijkstra_publisher()

    rclpy.spin(dks_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dks_publisher.destroy_node()
    rclpy.shutdown()
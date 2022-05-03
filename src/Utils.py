import numpy as np
import cv2
import math

"""
@brief Implementation of Custom Priority Queue
"""
class priority_queue():
    def __init__(self):
        self.pr_queue = list()
        self.astar_cost = list()
        self.cost_to_come = list()

    """
    @brief Method to add a new node to the queue
    @details The new node is added in the list according to its Astar cost
    """
    def add(self, child, star_cost):
        self.astar_cost.append(star_cost)
        self.astar_cost.sort(reverse = True)
        ind = self.astar_cost.index(star_cost)
        self.cost_to_come.insert(ind, child.cost)
        self.pr_queue.insert(ind, child)

    """
    @brief Method to remove a node from the queue
    """
    def remove(self):
        ast = self.astar_cost.pop()
        cst = self.cost_to_come.pop()
        check = self.pr_queue.pop()
        return check

    """
    @brief Method that prints the entire queue
    @details helpful for debugging the planner
    """
    def show(self):
        print("pr_queue ",self.pr_queue,"\n")
        print("astar cost ",self.astar_cost,"\n")
        print("cost to come ",self.cost_to_come,"\n")

"""
@brief Node class
@details This class supports the implementation of trees by storing the
         the parent node
"""
class Node():
    def __init__(self,current,theta,cost,parent,action_index,time,small_steps):
        self.current = current
        self.theta = theta
        self.cost = cost
        self.parent = parent
        self.action_index = action_index
        self.time = time
        self.small_steps = small_steps


"""
@brief World class that initializes the map in OpenCV
"""
class World():
    def __init__(self,map_size):
        self.robot_radius = 0.105
        self.clearance = 0.05    # 0.01
        self.cl = self.robot_radius + self.clearance
        self.scale = 80
        self.canvas_size = map_size
        self.canvas_size[0] = self.canvas_size[0]*self.scale
        self.canvas_size[1] = self.canvas_size[1]*self.scale
        self.canvas = np.zeros([self.canvas_size[0],self.canvas_size[1],3])
        self.canvas = self.canvas.astype(np.uint8)

    """
    @brief Method to convert the x and y coordinates to OpenCV scale
    @param currentnode: contains list of xy coordinates
    @return list of scaled x and y coordinates of current node
    """
    def convert_to_scale(self,currentnode):
        nodet = currentnode.copy()
        nodet[0] = round(self.scale*(currentnode[0]+5))
        nodet[1] = round(self.scale*(currentnode[1]+5))
        return nodet

    """
    @brief Method that defines the obstacles
    @param x: x-coordinate on the map
    @param y: y-coordinate on the map
    """
    def obstacles(self,x,y):
        #center rectangle
        if x>=(self.scale*(3.75-self.cl)) and x<=(self.scale*(6.25+self.cl)) and y>=(self.scale*(4.25-self.cl)) and y<=(self.scale*(5.75+self.cl)):     #scale the coordinates
            self.canvas[(self.canvas_size[0]-1)-y,x,0] = 255

        #right rectangle
        if x>=(self.scale*(7.25-self.cl)) and x<=(self.scale*(8.75+self.cl)) and y>=(self.scale*(2-self.cl)) and y<=(self.scale*(4+self.cl)):
            self.canvas[(self.canvas_size[0]-1)-y,x,0] = 255

        #square
        if x>=(self.scale*(0.25-self.cl)) and x<=(self.scale*(1.75+self.cl)) and y>=(self.scale*(4.25-self.cl)) and y<=(self.scale*(5.75+self.cl)):
            self.canvas[(self.canvas_size[0]-1)-y,x,0] = 255

        #bottom circle
        if (x-(self.scale*2))**2 + (y-(self.scale*8))**2 <= (self.scale*(1+self.cl))**2:
            self.canvas[(self.canvas_size[0]-1)-y,x,0] = 255

        #top circle
        if (x-(self.scale*2))**2 + (y-(self.scale*2))**2 <=(self.scale*(1+self.cl))**2:
            self.canvas[(self.canvas_size[0]-1)-y,x,0] = 255


        #center rectangle
        if x>=(self.scale*3.75) and x<=(self.scale*6.25) and y>=(self.scale*4.25) and y<=(self.scale*5.75):
            self.canvas[(self.canvas_size[0]-1)-y,x,:] = [0,128,128]

        #right rectangle
        if x>=(self.scale*7.25) and x<=(self.scale*8.75) and y>=(self.scale*2) and y<=(self.scale*4):
            self.canvas[(self.canvas_size[0]-1)-y,x,:] = [0,128,128]

        #square
        if x>=(self.scale*0.25) and x<=(self.scale*1.75) and y>=(self.scale*4.25) and y<=(self.scale*5.75):
            self.canvas[(self.canvas_size[0]-1)-y,x,:] = [0,128,128]

        #bottom circle
        if (x-(self.scale*2))**2 + (y-(self.scale*8))**2 <= (self.scale*1)**2:
            self.canvas[(self.canvas_size[0]-1)-y,x,:] = [0,128,128]

        #top circle
        if (x-(self.scale*2))**2 + (y-(self.scale*2))**2 <=(self.scale*1)**2:
            self.canvas[(self.canvas_size[0]-1)-y,x,:] = [0,128,128]

        return self.canvas

    """
    @brief Method to display the obstacles in the map
    @return Returns the map, map size and scale factor
    """
    def prepare_canvas(self):
        for i in range(self.canvas_size[0]):
            for j in range(self.canvas_size[1]):
                self.canvas = self.obstacles(j,i)
        return self.canvas, self.canvas_size, self.scale

    """
    @brief Method to assign goal area in the map
    @return Returns the map, map size and the scale factor
    """
    def goal_threshold(self,goal_pos):
        for i in range(self.canvas_size[0]):
            for j in range(self.canvas_size[1]):
                if (i-goal_pos[0])**2 + (j-goal_pos[1])**2 <= ((self.scale*1)/4)**2:
                    self.canvas[(self.canvas_size[0]-1)-i, j,1] = 255
        return self.canvas, self.canvas_size, self.scale

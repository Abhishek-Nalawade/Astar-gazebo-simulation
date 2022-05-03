from Utils import *

"""
@brief Class Astar Planner
"""
class Astar():
    # currently duplicate_canvas is used to store the cost info at index 0
    # and orientation info at index 1
    def __init__(self,start_pos, goal_pos, canvas, canvas_size, scale):
        self.start = start_pos
        self.goal = goal_pos[0]
        self.scale = scale
        self.canvas_size = canvas_size
        self.canvas = canvas
        self.duplicate_canvas = np.zeros([self.canvas_size[0],self.canvas_size[1],1],dtype='object')
        self.queue = priority_queue()
        self.start_node = Node(start_pos[0],start_pos[1],0,None,None,0, None)
        self.visited_dict = dict()

    """
    @brief Method to convert the x and y coordinates to OpenCV scale
    @param currentnode: contains list of xy coordinates
    @return list of scaled x and y coordinates of current node
    """
    def convert_to_scale(self,curr_node):
        nodet = curr_node.copy()
        nodet[0] = round(self.scale*(curr_node[0]+5))
        nodet[1] = round(self.scale*(curr_node[1]+5))
        return nodet

    """
    @brief Method that pops a node from the queue
    @return check1: contains the [y, x] pair of coordinates
    """
    def remove_node_from_queue(self):
        check1 = self.queue.remove()
        return check1

    """
    @brief Method to check if the popped node is visited or not
    @details If the node is visited then the cost of the parent node can be
             updated
    @return Returns None if the new node is already visited else returns the
            node
    """
    def check_if_visited(self,currentnode):
        scale_coor = self.convert_to_scale(currentnode.current)
        #print("scaled ",scale_coor,"\n")
        a = "%s " %scale_coor
        b = "%s" %round((currentnode.theta) * (180/math.pi))
        c = a + b
        if self.canvas[(self.canvas_size[0]-1)-scale_coor[0],scale_coor[1],2] == 255:
            try:
                self.visited_dict[c]
                #print("completely same")
                if self.visited_dict[c].cost > currentnode.cost:
                    #print("replaced")
                    self.visited_dict[c].parent = currentnode.parent
                    self.visited_dict[c].cost = currentnode.cost
                    self.visited_dict[c].small_steps = currentnode.small_steps
                    self.visited_dict[c].action_index = currentnode.action_index
                    self.visited_dict[c].time = currentnode.time
                return None
            except:
                pass

        self.canvas[(self.canvas_size[0]-1)-scale_coor[0],scale_coor[1],2] = 255
        self.visited_dict[c] = currentnode
        cv2.imshow("canvas", self.canvas)
        cv2.waitKey(1)

        return currentnode

    """
    @brief Method that simulates the move action of the robot
    @details Simulates the moving of robot by applying velocity angular
             velocities to the wheels. This action generates 5 children nodes
    @return Returns the list of children nodes and the list of actions performed
    """
    def move(self, currentnode):

        def check_if_in_obstacle_space(coor):
            if coor[0]<0 or coor[1]<0 or coor[0]>=self.canvas_size[0] or coor[1]>=self.canvas_size[1]:
                return None
            elif self.canvas[(self.canvas_size[0]-1)-coor[0],coor[1],0] == 255:
                return None
            else:
                return coor


        def perform(currentnode,idx,UL,UR):
            t = 0
            r = 0.033
            L = 0.160
            dt = 0.1
            intermediate_path = list()

            adult = currentnode.current.copy()
            Xn = adult[1]
            Yn = adult[0]
            Thetai = currentnode.theta
            #Thetan = (math.pi * Thetai)/180
            Thetan = Thetai

            # Xi, Yi,Thetai: Input point's coordinates
            # Xs, Ys: Start point coordinates
            # Xn, Yn, Thetan: End point coordinates
            D = currentnode.cost
            while t<1:
                t = t + dt
                Xs = Xn
                Ys = Yn
                Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
                Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
                Thetan += (r / L) * (UR - UL) * dt
                #D=D+ math.sqrt(math.pow((0.5*r *(UL + UR)*math.cos(Thetan)*dt),2)+math.pow((0.5*r*(UL + UR)*math.sin(Thetan) * dt),2))
                scale_coor = self.convert_to_scale([Yn,Xn])
                scale_coor = check_if_in_obstacle_space(scale_coor)
                if scale_coor == None:
                    return None
                self.canvas[(self.canvas_size[0]-1)-scale_coor[0],scale_coor[1],1] = 255
                intermediate_path.append([Yn,Xn])

            if Thetan >= (2*math.pi):
                Thetan = Thetan - (2*math.pi)
            D = D + ((currentnode.current[0]-Yn)**2 + (currentnode.current[1]-Xn)**2)**(1/2)
            ##D = D + (((currentnode.current[0]-Yn)**2 + (currentnode.current[1]-Xn)**2)**(1/2))*self.scale

            return Node([Yn,Xn],Thetan,D,currentnode,idx,t,intermediate_path)

        children = list()
        # actions = [[10,10],[5,0],[0,5],[5,10],[10,5]]
        actions = [[10,10],[5,2],[2,5],[5,10],[10,5]]
        for action in actions:
            ind = actions.index(action)
            children.append(perform(currentnode, ind, action[0],action[1]))

        return children, actions

    """
    @brief Method to compute the heuristic cost
    @details computes the euclidean distance to the goal node
    @param child: node object of the new child
    @return dist: contains the euclidean distance to the goal node
    """
    def euclidean_distance(self,child):
        dist = ((self.goal[0] - child.current[0])**2 + (self.goal[1] - child.current[1])**2)**(1/2)
        return dist

    """
    @brief Method to check if goal has been reached or not
    @param children: contains the list of all newly generated child nodes
    @return Returns goal node if goal has reached or else returns the
    """
    def check_if_goal(self, children):
        for child in children:
            if child is not None:
                coor = self.convert_to_scale(child.current)
                t_goal = self.convert_to_scale(self.goal)
                if (coor[0]-t_goal[0])**2 + (coor[1]-t_goal[1])**2 <= ((self.scale*1)/4)**2:
                    print("\n Goal has been reached \n")
                    return child
                else:
                    heuristic_cost = child.cost + (1.2*self.euclidean_distance(child))
                    self.queue.add(child, heuristic_cost)
        return None

    """
    @brief Method to find the path after the goal has been reached
    @param child: contains the goal coordinates [y, x] that has been reached
    @return lis: contains the list of index of action used to reach a node and
            the time required in seconds to reach from the previous node.
    """
    def backtracking(self,child):
        lis = list()
        while child.parent is not None:
            micro_path = child.small_steps
            for i in micro_path:
                mark = self.convert_to_scale(i)
                cv2.circle(self.canvas, (mark[1],(self.canvas_size[0]-1)-mark[0]),2,(255,0,0),-1)

            lis.append([child.action_index,child.time])
            child = child.parent
            cv2.imshow("canvas",self.canvas)
            cv2.waitKey(1)
        return lis

    """
    @brief Method that runs the complete Astar algorithm
    @return final_path: contains a list of [action, time]
    """
    def run_Astar(self):
        self.queue.add(self.start_node,0)
        while True:
            while True:
                nde = self.remove_node_from_queue()
                nde = self.check_if_visited(nde)
                if nde is not None:
                    break

            children1, action_list = self.move(nde)
            goal_node = self.check_if_goal(children1)
            if goal_node is not None:
                path = self.backtracking(goal_node)
                break

        final_path = list()
        for i in path:
            if i[0] == None:
                break
            final_path.append([action_list[i[0]],i[1]])
        return final_path

import numpy as np
import cv2
import math


class priority_queue():
    def __init__(self):
        self.pr_queue = list()
        self.astar_cost = list()
        self.cost_to_come = list()

    def add(self, child, star_cost):
        self.astar_cost.append(star_cost)
        self.astar_cost.sort(reverse = True)
        ind = self.astar_cost.index(star_cost)
        self.cost_to_come.insert(ind, child.cost)
        self.pr_queue.insert(ind, child)

    def remove(self):
        ast = self.astar_cost.pop()
        cst = self.cost_to_come.pop()
        check = self.pr_queue.pop()
        #print("popped cost ",cst)
        #print("popped ",ast)
        #print("child ",check.current)
        return check

    def show(self):
        print("pr_queue ",self.pr_queue,"\n")
        print("astar cost ",self.astar_cost,"\n")
        print("cost to come ",self.cost_to_come,"\n")


class node():
    def __init__(self,current,theta,cost,parent,action_index,time,small_steps):
        self.current = current
        self.theta = theta
        self.cost = cost
        self.parent = parent
        self.action_index = action_index
        self.time = time
        self.small_steps = small_steps



class world():
    def __init__(self,map_size):
        self.robot_radius = 0.105
        self.clearance = 0.01
        self.cl = self.robot_radius + self.clearance
        self.scale = 80
        self.canvas_size = map_size
        self.canvas_size[0] = self.canvas_size[0]*self.scale
        self.canvas_size[1] = self.canvas_size[1]*self.scale
        self.canvas = np.zeros([self.canvas_size[0],self.canvas_size[1],3])
        self.canvas = self.canvas.astype(np.uint8)

    def convert_to_scale(self,currentnode):
        nodet = currentnode.copy()
        nodet[0] = round(self.scale*(currentnode[0]+5))
        nodet[1] = round(self.scale*(currentnode[1]+5))
        return nodet

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

    def prepare_canvas(self):
        for i in range(self.canvas_size[0]):
            for j in range(self.canvas_size[1]):
                self.canvas = self.obstacles(j,i)
        return self.canvas, self.canvas_size, self.scale


    def goal_threshold(self,goal_pos):
        for i in range(self.canvas_size[0]):
            for j in range(self.canvas_size[1]):
                if (i-goal_pos[0])**2 + (j-goal_pos[1])**2 <= ((self.scale*1)/4)**2:
                    self.canvas[(self.canvas_size[0]-1)-i, j,1] = 255
        return self.canvas, self.canvas_size, self.scale


#currently duplicate_canvas is used to store the cost info at index 0 and orientation info at index 1
class Astar():
    def __init__(self,start_pos, goal_pos, canvas, canvas_size, scale):
        self.start = start_pos
        self.goal = goal_pos[0]
        self.scale = scale
        self.canvas_size = canvas_size
        self.canvas = canvas
        self.duplicate_canvas = np.zeros([self.canvas_size[0],self.canvas_size[1],1],dtype='object')
        self.queue = priority_queue()
        self.start_node = node(start_pos[0],start_pos[1],0,None,None,0, None)
        self.visited_dict = dict()


    def convert_to_scale(self,curr_node):
        nodet = curr_node.copy()
        nodet[0] = round(self.scale*(curr_node[0]+5))
        nodet[1] = round(self.scale*(curr_node[1]+5))
        return nodet

    def remove_node_from_queue(self):
        check1 = self.queue.remove()
        return check1

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
            Xn=adult[1]
            Yn=adult[0]
            Thetai = currentnode.theta
            #Thetan = (math.pi * Thetai)/180
            Thetan = Thetai


            # Xi, Yi,Thetai: Input point's coordinates
            # Xs, Ys: Start point coordinates for plo    Thetan = 180 * (Thetan) / math.pit function
            # Xn, Yn, Thetan: End point coordintes
            D=currentnode.cost
            while t<1:
                t = t + dt
                Xs = Xn
                Ys = Yn
                Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
                Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
                Thetan += (r / L) * (UR - UL) * dt
                #D=D+ math.sqrt(math.pow((0.5*r *(UL + UR)*math.cos(Thetan)*dt),2)+math.pow((0.5*r*(UL + UR)*math.sin(Thetan) * dt),2))
                #print([round(8*Xs*10),round(8*Xn*10)])
                #plt.plot([round(80*Xs),round(80*Xn)],[round(80*Ys),round(80*Yn)],color="blue")
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
            #print("cost is ",D)
            #print("Yn, Xn ", Yn,Xn)
            return node([Yn,Xn],Thetan,D,currentnode,idx,t,intermediate_path)

        children = list()

        actions = [[10,10],[5,0],[0,5],[5,10],[10,5]]
        for action in actions:
            ind = actions.index(action)
            children.append(perform(currentnode, ind, action[0],action[1]))

        #print(len(children))

        return children, actions

    def euclidean_distance(self,child):
        dist = ((self.goal[0] - child.current[0])**2 + (self.goal[1] - child.current[1])**2)**(1/2)
        return dist


    def check_if_goal(self, children):
        for child in children:
            if child is not None:
                coor = self.convert_to_scale(child.current)
                t_goal = self.convert_to_scale(self.goal)
                if (coor[0]-t_goal[0])**2 + (coor[1]-t_goal[1])**2 <= ((self.scale*1)/4)**2:
                    print("\n Goal has been reached \n")
                    return child
                else:
                    heuristic_cost = child.cost + (1.1*self.euclidean_distance(child))
                    #heuristic_cost = child.cost + (self.euclidean_distance(child)*self.scale)
                    #print("cost ",child.cost)
                    #print("heuristic_cost ",heuristic_cost,"\n")
                    self.queue.add(child, heuristic_cost)
        return None

    def backtracking(self,child):
        lis = list()
        while child.parent is not None:
            micro_path = child.small_steps
            for i in micro_path:
                mark = self.convert_to_scale(i)
                cv2.circle(self.canvas, (mark[1],(self.canvas_size[0]-1)-mark[0]),2,(255,0,0),-1)

            #mark = self.convert_to_scale(par.current)
            lis.append([child.action_index,child.time])
            child = child.parent
            cv2.imshow("canvas",self.canvas)
            cv2.waitKey(1)
        return lis

    def run_Astar(self):
        self.queue.add(self.start_node,0)
        #self.queue.show()
        while True:
            #print("hi")
            while True:
                nde = self.remove_node_from_queue()
                nde = self.check_if_visited(nde)
                #print(nde)
                if nde is not None:
                    break

            #print("cost",nde.cost)
            children1, action_list = self.move(nde)
            goal_node = self.check_if_goal(children1)
            #self.queue.show()
            if goal_node is not None:
                path = self.backtracking(goal_node)
                break

        final_path = list()
        for i in path:
            if i[0] == None:
                break
            final_path.append([action_list[i[0]],i[1]])
        return final_path

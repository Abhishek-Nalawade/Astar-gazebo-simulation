#!/usr/bin/env python
from Project3Phase3_classes import *
import rospy
from geometry_msgs.msg import Twist
import sys


def initiate_Astar():
    obs_check = 0
    map = world([10,10])
    canvas, canvas_size, scale = map.prepare_canvas()

    while True:
        # x1 = input("Enter the x co-ordinate of the start point: ")
        # y1 = input("Enter the y co-ordinate of the start point: ")
        # st_theta = input("Enter the orientation of start point: ")
        # x2 = input("Enter the x co-ordinate of the goal point: ")
        # y2 = input("Enter the y co-ordinate of the goal point: ")
        # gl_theta = input("Enter the orientation of goal point: ")
        x1 = sys.argv[1]
        y1 = sys.argv[2]
        st_theta = sys.argv[3]
        x2 = sys.argv[4]
        y2 = sys.argv[5]
        gl_theta = sys.argv[6]
        start1 = [float(y1),float(x1)]
        goal1 = [float(y2),float(x2)]
        start_cv = map.convert_to_scale([float(y1),float(x1)])
        goal_cv = map.convert_to_scale([float(y2),float(x2)])
        start_orient = float(st_theta)
        goal_orient = float(gl_theta)
        if canvas[(canvas_size[0]-1)-start_cv[0],start_cv[1],0] == 255 or canvas[(canvas_size[0]-1)-goal_cv[0],goal_cv[1],0] == 255 or \
            canvas[(canvas_size[0]-1)-start_cv[0],start_cv[1],1] == 128 or canvas[(canvas_size[0]-1)-goal_cv[0],goal_cv[1],1] == 128:
            print("\n\n\n\n\n\n")
            print("Error: One of the entered point is either in obstacle space or out of map boundary")
            print("\n\n\n\n\n\n")
            obs_check = 1
            break
        else:
            break

    canvas, canvas_size, scale = map.goal_threshold(goal_cv)
    cv2.imshow("canvas",canvas)
    cv2.waitKey(0)
    return [start1,start_orient], [goal1,goal_orient], canvas, canvas_size, scale, obs_check

start, goal, canvas, c_size, factor, obs_check = initiate_Astar()
astar = Astar(start, goal, canvas, c_size, factor)
if obs_check == 0:
    ultimate_path = astar.run_Astar()
else:
    print()
    print("\n\n\n\nOne of the either point is in obstacles or out of map boundary\nPlease try again\n\n\n\n\n")
#print("ultimate path ",ultimate_path)
#print("action set",actions_set)

def talker(path):
    msg = Twist()
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    rospy.init_node('talking',anonymous=True)
    rate = rospy.Rate(0.90909)
    rate.sleep()
    rate.sleep()
    for i in range(len(path)):
        r=0.033
        l=0.160
        msg.linear.x = (r/2)*(path[i][0][0]+path[i][0][1])
        msg.angular.z = (r/l)*(path[i][0][1]-path[i][0][0])
        print((r/l)*(path[i][0][1]-path[i][0][0]))
        pub.publish(msg)
        rate.sleep()

    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

new_path = list()
for i in range(len(ultimate_path)):
    new_path.append(ultimate_path.pop())
print("reversed ",new_path)
talker(new_path)

import numpy as np
import matplotlib.pyplot as plt
import math
from queue import PriorityQueue as pq
#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

obstacle_space=[]
visited_nodes=[]
viz=[]
x_plot=[]
y_plot=[]
new_goal=()
table=[]
open_list=pq()
check_reach=False
steps=0.1



def range_floats(start,end,step):
    while end>start:
        yield start
        start+=step

def getting_obstacle_points(clearance):
    for x in range_floats (1.5-clearance,1.65+clearance+steps,steps):
        for y in range_floats (0.75-clearance,2+steps,steps):
            obstacle_space.append((round(x,1),round(y,1)))
    
    for x in range_floats (2.5-clearance,2.65+clearance+steps,steps):
        for y in range_floats (0,1.25+clearance+0.1,steps):
            obstacle_space.append((round(x,1),round(y,1)))

    for x in range_floats (3.5-clearance,4.5+clearance+steps,steps):
        for y in range_floats (0.6-clearance,1.6+clearance+steps,steps):
            if math.pow((x-4),2)+math.pow((y-1.1),2)<=0.25:
                obstacle_space.append((round(x,1),round(y,1)))
    
    for x in range_floats(0,6+steps,steps):
        for y in range_floats(0,clearance+steps,steps):
            obstacle_space.append((round(x,1),round(y,1)))
        for y in range_floats(2-clearance,2.01,steps):
            obstacle_space.append((round(x,1),round(y,1)))
    
    for y in range_floats(0,2.01,steps):
        for x in range_floats(0,clearance+steps,steps):
            obstacle_space.append((round(x,1),round(y,1)))
        for x in range_floats(6-clearance,6.01,steps):
            obstacle_space.append((round(x,1),round(y,1)))

def change_points(points, length):
    return (points[0], length - points[1])

def change_points_rect(points, length, obj_height):
    return (points[0], length - points[1] - obj_height)


def dist(p1,p2):
    l_2_norm=math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
    l_2_norm=round(l_2_norm,2)
    return l_2_norm


def check_theta(theta):
    if theta>=360:
        theta=theta-360
    if theta<0:
        theta=theta+360
    return theta

def get_input():
    while(True):
        start_x=float(input("input start x coordinate: "))
        start_y=float(input("input start y coordinate: "))
        start_theta=int(input("input start orientation: "))
        start_theta=check_theta(start_theta)
        if (start_x,start_y) not in obstacle_space:
            start=(start_x,start_y,start_theta)
            break
    
    while(True):
        goal_x=float(input("input goal x coordinate: "))
        goal_y=float(input("input goal y coordinate: "))
        if (goal_x,goal_y) not in obstacle_space:
            goal=(goal_x,goal_y)
            break
    
    RPM_1=int(input("enter RPM1: "))
    RPM_2=int(input("enter RPM2: "))
    return start,goal,RPM_1,RPM_2

def rpm_to_velocity(rpm):
    radius=0.033
    vel=2*math.pi*radius*rpm/60
    return vel

def cost(node,ul,ur,c2c,goal):
    global check_reach
    global new_goal
    t=0
    r=0.033
    l=0.16
    dt=1
    D=0
    xn=node[0]
    yn=node[1]
    thetan=3.14*node[2]/180
    reached=0
    while t<10:
        t=t+dt
        xs=xn
        ys=yn
        delta_xn= 0.5*r * (ul + ur) * math.cos(thetan) * dt
        delta_yn= 0.5*r * (ul + ur) * math.sin(thetan) * dt
        thetan_now=(r/l)*(ur-ul)*dt
        thetan+=thetan_now
        xn+=delta_xn
        yn+=delta_yn
        D=D+dist((xs,ys),(xn,yn))
        if (round(xn,2),round(yn,2)) not in obstacle_space:
            reached+=1
        else:
            break
    if reached<10:
        reached=0
        return 
    else:
        if xn>0 and xn<6 and yn>0 and yn<2:
            thetan = 180 * (thetan) / 3.14
            thetan=check_theta(thetan)
            if (round(xn,1),round(yn,1),round(thetan,1))not in visited_nodes and (round(xn,1),round(yn,1)) not in obstacle_space:
                new_node=(round(xn,1),round(yn,1),round(thetan,1))
                new_c2c=round(D+c2c,1)
                new_c2g=round(dist(new_node,goal),1)
                cost=round(new_c2c+new_c2g,1)
                print(new_c2g)
                for i in range(open_list.qsize()):
                    if open_list.queue[i][3]==new_node:
                        if open_list.queue[i][0]>cost:
                            open_list.queue[i]=(cost,new_c2g,new_c2c,new_node)
                            for i in range(len(table)):
                                if table[i][2]==new_node:
                                    if cost<table[i][0]:
                                        table[i][0]=cost
                                        table[i][1]=node
                                        table[i][2]=new_node
                                        return
                                    else:
                                        return
                table.append([cost,node,new_node])
                open_list.put((cost,new_c2g,new_c2c,new_node))
                if dist(new_node,goal)<0.1:
                    check_reach=True
                    new_goal=new_node
                reached = 0

def points_to_vel(back_track):
    x_pts=back_track[:,0]
    y_pts=back_track[:,1]
    theta_pts=back_track[:,2]
    for i in range (len(theta_pts)):
        theta_pts[i]=3.14*theta_pts[i]/180
    print(len(theta_pts))
    print(len(back_track))
    x_vel=0
    y_vel=0
    theta_vel=0
    vel=[]
    for i in range (len(back_track)-1):
        x_vel=(x_pts[i+1]-x_pts[i])/10
        y_vel=(y_pts[i+1]-y_pts[i])/10
        theta_vel=(theta_pts[i+1]-theta_pts[i])/10
        vel.append([x_vel,y_vel,theta_vel])
    return vel

getting_obstacle_points(0.105)
start,goal,RPM1,RPM2=get_input()
vel1,vel2=rpm_to_velocity(RPM1),rpm_to_velocity(RPM2)
actions=[[0,vel1],[vel1,0],[vel1,vel1],[0,vel2],[vel2,0],[vel2,vel2],[vel1,vel2],[vel2,vel1]]
table.append([0,start,start])
back_track=[]
c2g_init=round(dist(start,goal),1)
open_list.put((round(dist(start,goal),1),round(dist(start,goal),1),0,start))
while(open_list.empty()==False):
    current_node=open_list.get()
    now_node=current_node[3]
    if now_node not in visited_nodes:
        visited_nodes.append(now_node)
        for action in actions:
            cost(now_node,action[0],action[1],current_node[2],goal)
            if check_reach==True:
                break
        if check_reach==True:
            print('goal reached')
            break
if check_reach==False:
    print("goal not reached")
else:
    print("goal reached")
    print(new_goal)
    back_node=new_goal
    print("backnode",back_node)
    back_track=[back_node]
    while(True):
        for i in range(len(table)):
            if table[i][2]==back_node:
                back_node=table[i][1]
                back_track.append(back_node)
                break
        if back_node==start:
            break
    back_track.reverse()
    print()
    print("BACK TRACK")
    print(back_track)
    back_track=np.array(back_track)
    x_plot=back_track[:,0]
    y_plot=back_track[:,1]
    obstacle_space=np.array(obstacle_space)
    x_plot_obstacle=obstacle_space[:,0]
    y_plot_obstacle=obstacle_space[:,1]
    plt.scatter(x_plot_obstacle,y_plot_obstacle)
    plt.scatter(x_plot,y_plot)
    plt.show()
    vel=points_to_vel(back_track)
    print(vel)

#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

def test():
	msg=Twist()
	pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	rospy.init_node('robot_talker',anonymous=True)
	for i in back_track:
		msg.angular.z=i[1]
		msg.linear.x=i[0]
		#buff='my current time is %s" %rospy.get_time()
		pub.publish(msg)
		time.sleep(0.1)
		
if __name__=='__main__':
	test()
	


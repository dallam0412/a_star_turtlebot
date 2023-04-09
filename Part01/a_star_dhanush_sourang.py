import numpy as np
import matplotlib.pyplot as plt
import math
from queue import PriorityQueue as pq
from sortedcollections import OrderedSet
import pygame as pg

obstacle_space=OrderedSet()
visited_nodes=[]
viz=[]
x_plot=[]
y_plot=[]
new_goal=()
table=[]
open_list=pq()
check_reach=False
steps=0.01
back_track_intermediate=[]

def range_floats(start,end,step):
    while end>start:
        yield start
        start+=step

def getting_obstacle_points(clearance):
    for x in range_floats (1.5-clearance,1.65+clearance+steps,steps):
        for y in range_floats (0.75-clearance,2+steps,steps):
            obstacle_space.add((round(x,2),round(y,2)))
    
    for x in range_floats (2.5-clearance,2.65+clearance+steps,steps):
        for y in range_floats (0,1.25+clearance+0.1,steps):
            obstacle_space.add((round(x,2),round(y,2)))

    for x in range_floats (3.5-clearance,4.5+clearance+steps,steps):
        for y in range_floats (0.6-clearance,1.6+clearance+steps,steps):
            if math.pow((x-4),2)+math.pow((y-1.1),2)<=0.25+clearance:
                obstacle_space.add((round(x,2),round(y,2)))
    
    for x in range_floats(0,6+steps,steps):
        for y in range_floats(0,clearance+steps,steps):
            obstacle_space.add((round(x,2),round(y,2)))
        for y in range_floats(2-clearance,2.01,steps):
            obstacle_space.add((round(x,2),round(y,2)))
    
    for y in range_floats(0,2.01,steps):
        for x in range_floats(0,clearance+steps,steps):
            obstacle_space.add((round(x,2),round(y,2)))
        for x in range_floats(6-clearance,6.01,steps):
            obstacle_space.add((round(x,2),round(y,2)))

def change_points(points, length):
    return (points[0], length - points[1])

def change_points_plot(points, length):
    return (points[0]*100, length - points[1]*100)

def change_points_rect(points, length, obj_height):
    return (points[0], length - points[1] - obj_height)


def dist(p1,p2):
    l_2_norm=math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
    return l_2_norm


def check_theta(theta):
    if theta>=360:
        theta=theta%360
    elif -360 < theta<0:
        theta += 360
    elif theta <= -360:
        theta=theta%360+360
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
    #vel=2*math.pi*radius*rpm/60
    vel=2*math.pi*rpm/60
    return vel

def cost(node,ul,ur,c2c,goal):
    global check_reach
    global new_goal
    t=0
    r=0.033
    l=0.16
    dt=0.2
    D=0
    back_track_intermediate=[]
    xn=node[0]
    yn=node[1]
    thetan=(3.14*node[2])/180
    back_track_intermediate.append(change_points_plot((round(xn,2),round(yn,2)), 200))
    reached=0
    while t<1:
        xs=xn
        ys=yn
        delta_xn= 0.5*r * (ul + ur) * math.cos(thetan) * dt
        delta_yn= 0.5*r * (ul + ur) * math.sin(thetan) * dt
        thetan_now=(r/l)*(ur-ul)*dt
        xn+=delta_xn
        yn+=delta_yn
        thetan+=thetan_now
        D=D+dist((xs,ys),(xn,yn))
        t=t+dt
        if (round(xn,2),round(yn,2)) not in obstacle_space:
            reached+=1
            back_track_intermediate.append(change_points_plot((round(xn,2),round(yn,2)), 200))

        else:
            break
    if reached<5:
        reached=0
        return 
    else:
        if xn>=0 and xn<=6 and yn>=0 and yn<=2:
            thetan = round(thetan,2)
            thetan = (180 * (thetan)) / 3.14
            thetan=check_theta(thetan)
            if (round(xn,2),round(yn,2))not in visited_nodes and (round(xn,2),round(yn,2)) not in obstacle_space:
                new_node=(round(xn,2),round(yn,2),round(thetan,2))
                new_c2c=D+c2c
                new_c2g=dist(new_node,goal)
                cost=new_c2c+new_c2g*1.25
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
                table.append([cost,node,new_node,back_track_intermediate])
                open_list.put((cost,new_c2g,new_c2c,new_node))
                visited_nodes.append((new_node[0], new_node[1]))
                if dist(new_node,goal)<0.15:
                    check_reach=True
                    new_goal=new_node
                reached = 0

def points_to_vel(back_track):
    back_track= np.array(back_track)
    x_pts=back_track[:,0]
    y_pts=back_track[:,1]
    theta_pts=back_track[:,2]
    for i in range (len(theta_pts)):
        theta_pts[i]=3.14*theta_pts[i]/180
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

def game(bloat,optimal_path):
    pg.init()
    size = [600, 200]
    display_view = pg.display.set_mode(size)
    pg.display.set_caption("Visualization")
    clock = pg.time.Clock()
    Cond = True

    rect_1_x, rect_1_y = change_points_rect([150, 75], 200, 125)
    rect_2_x, rect_2_y = change_points_rect([250, 0], 200, 125)
    rect_1_x_c, rect_1_y_c = change_points_rect([150-bloat, 75-bloat], 200, 125+bloat)
    rect_2_x_c, rect_2_y_c = change_points_rect([250-bloat, 0], 200, 125+bloat)
    circle_1_x, circle_1_y = change_points([400, 110], 200)

    while Cond:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                Cond = False
        pg.draw.rect(display_view, [0,255,0], [rect_1_x_c, rect_1_y_c, 15+2*bloat, 125+bloat], 0)
        pg.draw.rect(display_view, [0,255,0], [rect_2_x_c, rect_2_y_c, 15+2*bloat, 125+bloat], 0)
        pg.draw.rect(display_view, [255,0,0], [rect_1_x, rect_1_y, 15, 125], 0)
        pg.draw.rect(display_view, [255,0,0], [rect_2_x, rect_2_y, 15, 125], 0)
 

        pg.draw.circle(display_view, [0,255,0], (circle_1_x, circle_1_y), 50 + bloat)
        pg.draw.circle(display_view, [255,0,0], (circle_1_x, circle_1_y), 50)
        pg.draw.rect(display_view, [255,0,0], [0, 0, bloat, 200], 0)
        pg.draw.rect(display_view, [255,0,0], [0, 0, 600, bloat], 0)
        pg.draw.rect(display_view, [255,0,0], [0, 200-bloat, 600, bloat], 0)
        pg.draw.rect(display_view, [255,0,0], [600-bloat, 0, bloat, 200], 0)
    
        for l in range(1,len(table)):
        
            inter_list=table[l][3]
            #print("inter_list1",inter_list[1])
            pg.draw.lines(display_view, "white",False,inter_list,2)
            pg.display.flip()
            clock.tick(300)

        for i in optimal_path:
            # print(i)
            item = (i[0], i[1])
            pg.draw.circle(display_view, "blue", change_points_plot(item, 200), 4)
            pg.display.flip()
            clock.tick(20)
        Cond = False

    pg.display.flip()
    pg.time.wait(3000)
    pg.quit()

clearance=int(input("enter the clearance in mm: "))
clearance=clearance/1000
getting_obstacle_points(0.105+clearance)
start,goal,RPM1,RPM2=get_input()
vel1,vel2=rpm_to_velocity(RPM1),rpm_to_velocity(RPM2)
actions=[[0,vel1],[vel1,0],[vel1,vel1],[0,vel2],[vel2,0],[vel2,vel2],[vel1,vel2],[vel2,vel1]]
table.append([0,start,start])
back_track=[]
c2g_init= (dist(start,goal))
open_list.put((c2g_init,c2g_init,0,start))
while(open_list.empty()==False):
    current_node=open_list.get()
    now_node=current_node[3]
    if dist(now_node, goal) > 0.15:
    # if (now_node[0], now_node[1]) not in visited_nodes:
    #     visited_nodes.append((now_node[0], now_node[1]))
        for action in actions:
            cost(now_node,action[0],action[1],current_node[2],goal)
            if check_reach==True:
                break
        if check_reach==True:
            break
if check_reach==False:
    print("goal not reached")
else:
    print("goal reached")
    back_node=new_goal
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
    print(back_track)
    vel=points_to_vel(back_track)

    game(clearance*100, back_track)

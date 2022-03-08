#!/usr/bin/env python
# coding: utf-8

# In[1]:


import csv
import numpy as np 
from numpy import ndarray 
import math
import matplotlib.pyplot as plt
from sympy import symbols, Eq, solve
from numpy.random import RandomState

obstacles_info = open('D:/ROBOTICS/V-REP_scenes/PRM_ProjectFiles/Planning_coursera/obstacles.csv')
obRead = csv.reader(obstacles_info)
obstacles = []

for obs in obRead:
    obs[0] = float(obs[0])
    obs[1] = float(obs[1])
    obs[2] = float(obs[2])
    obstacles.append(obs)

obstacles = np.array(obstacles)    # Obstacles extracted successfully from the csv files

start_node = np.array([-0.5,-0.5])
goal_node = np.array([0.5,0.5])
    


# In[2]:


# Defining the obstacle region through circle equations(or expressions)

x, y = symbols('x y')
padding = 1.1    # padding on the obstacle radius to ensure obstacle avoidance
obsExpr = []        
for j in range(0,obstacles.shape[0]):
    obsExpr.append((x-obstacles[j,0])**2 + (y-obstacles[j,1])**2 -0.25*(obstacles[j,2]*padding)**2)
print(obsExpr)


# In[3]:


# Sampling
number_of_points = 180
np.random.seed(44)
points = -0.5 + np.random.rand(number_of_points,2)*(0.5 -(-0.5))  # 100 sampled points in the given domain


# In[4]:


# Function to plot obstacles, sampled points and edges when they are created
# This function will be used to visualize sampled points, even before edges are formed
def plotter(obstacles, points,edges):
    
    plt.axis("equal")

    for i in range(0, obstacles.shape[0]):
        c = plt.Circle((obstacles[i,0], obstacles[i,1]), radius=0.5*obstacles[i,2], fill = False)
        plt.gca().add_artist(c)

    plt.xlim(-0.6, 0.6)
    plt.ylim(-0.6, 0.6)

    start = np.array([-0.5,-0.5])
    goal = np.array([0.5,0.5])
    plt.plot(start[0], start[1], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")
    plt.plot(goal[0], goal[1], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")

    plt.scatter(points[:,0],points[:,1])
    if any(edges):             # This will be executed only if there are any edges inside the 'edges' variable
        for edge in edges:
            x_values = [points[edge[0]-1,0],points[edge[1]-1,0]]
            y_values = [points[edge[0]-1,1],points[edge[1]-1,1]]
            plt.plot(x_values, y_values)
            
            
plotter(obstacles, points,[])


# In[5]:


# Function to check whether a given point lies in obstacles

def cfree_check(X,Y,obsEqns):
    flag  = 0
    for k in range(0,len(obsExpr)):
        value = obsExpr[k].subs(x,X).subs(y,Y)  
        if value <= 0:
            flag = 1        # flag becomes 1 if point inside obstacles
            break   # Ending the for loop that iterates through all the obstacle equations
    return flag
        


# In[6]:


# Removing all sampled points that are not in Cfree. 
i = 0
while i in range(0, points.shape[0]):
  
    if cfree_check(points[i,0], points[i,1], obsExpr) == 1:
        points = np.delete(points, i, 0)          # Removing any sampled point that lies inside obstacles
    else :
        i+=1 
        
# Adding the start and the goal points to this array of points 

points = np.insert(points,0,[start_node],axis= 0)
points = np.insert(points,points.shape[0],[goal_node],axis= 0)
plotter(obstacles, points,[])


# In[7]:


# Function to find the 4 nearest neighbours of a node

def NN4(points,i):
    distances = np.empty([points.shape[0],1])    # An array to store distances
    for j in range(0, points.shape[0]):
        distances[j] = math.sqrt((points[j,0]-points[i,0])**2 + (points[j,1]-points[i,1])**2)
    K = 5         # Number of nearest neighbours      
    four_NN = sorted(range(len(distances)), key = lambda sub: distances[sub])[:K+1] 
    four_NN.remove(i)             # Removing the indices of the point from the list of indices to get 4 nearest neighbours
    return four_NN                # Would output the list of indices of the points with least four Eulerian distances 


# Function to find the equation of line between two points

def Line(x1,x2,y1,y2):
    if x1 == x2 :   # Vertical Line 
        a = 1
        b = 0
        c = -x1
    else:
        a = (y2-y1)/(x2-x1)
        b = -1
        c = y1 + x1*(y1-y2)/(x2-x1)
    return a,b,c


# In[8]:


# Function to check whether a line intersects any obstacle circle

# If the distance between a straight line and a circle centre <= radius, there is a collision.
# The function would return the shortest distance and a value 1. 
def intersection_points(a,b,c,obstacles):
    int_points_list = []
    for i in range (0,obstacles.shape[0]): 
        shortest_distance = abs(a*obstacles[i,0] + b*obstacles[i,1] + c)/math.sqrt(a**2 + b**2)
        x,y = symbols('x y')
        
        if shortest_distance <=  0.5*obstacles[i,2]:
            circle = Eq(obsExpr[i],0)     # Using circle expression created before
            line = Eq(a*x+b*y+c,0)
            sol = solve((circle, line),(x, y)) 
            for point in sol:
                point = list(point)            # Converting all  intersection points to list
                int_points_list.append(point)  # Adding the intersection to the list 
           
    return int_points_list  # Will be an empty list if there are no collisions with the line 


# In[9]:


# A function to check whether a given list of points lie in a particular line segment 

def location_check(x1,x2,y1,y2,pointList):
    edge_feasibility = 1
    for i in range(0,len(pointList)):
        x_alert = 0  # becomes 1 if a point's x coordinate lies in between that of the end points
        point = pointList[i]
        if (min(x1,x2)<point[0])and((max(x1,x2)>point[0])):
            x_alert = 1
        if x_alert==1:
            edge_feasibility = 0 
            # Since both the alerts are 1, it means the point lies on the line segment and hence edge feasibility is 0
            break
    return edge_feasibility
        


# In[10]:


# PRM 
edges = []
for k in range(0, points.shape[0]):
    four_NN = NN4(points,k)
    for neighbour in four_NN:
        a,b,c = Line(points[k,0],points[neighbour,0],points[k,1],points[neighbour,1])
        int_points_list = intersection_points(a,b,c,obstacles)      # Getting the intersection points
        edge_feasibility = 1
        if any(int_points_list):     
            edge_feasiblity = location_check(points[k,0],points[neighbour,0],points[k,1],points[neighbour,1],int_points_list)
        if edge_feasibility == 1:
            edge_length  = math.sqrt((points[k,0]-points[neighbour,0])**2+(points[k,1]-points[neighbour,1])**2)
            edge = []                         
            edge = [max(neighbour,k)+1, min(neighbour,k)+1,edge_length]  # The point with the higher index is in 1st column
             # The +1 ensures that nodes indices begin from 1   
            if edge in edges:
                pass
            else :
                edges.append(edge)               


# In[11]:


plotter(obstacles, points, edges)


# In[12]:


# WRITING THE EDGES TO A CSV FILE
edges = np.array(edges)       # Converting the edges list to a numpy array for convenience

np.savetxt("D:/ROBOTICS/V-REP_scenes/PRM_ProjectFiles/planning_coursera/edges.csv", edges, 
              delimiter = ",")
print(edges)


# In[13]:


# WRITING THE NODES TO A CSV FILE

nodes = np.empty([points.shape[0],4])   # Initializing the nodes array
for i in range(0, points.shape[0]):
    nodes[i,0] = i+1             # So that the node index begins from 1 
    nodes[i,1] = points[i,0]
    nodes[i,2] = points[i,1]
    # Heuristic Cost to go is considered as the Eulerian distance from the node to the goal
    heuristic_cost_to_go = math.sqrt((points[i,0]-goal_node[0])**2 + (points[i,1]-goal_node[1])**2)
    nodes[i,3] = heuristic_cost_to_go
for i in range(0,nodes.shape[0]):
    nodes[i,0] = int(nodes[i,0])
    nodes[i,1] = float(nodes[i,1])
    nodes[i,2] = float(nodes[i,2])
    nodes[i,3] = float(nodes[i,3])
np.savetxt("D:/ROBOTICS/V-REP_scenes/PRM_ProjectFiles/planning_coursera/nodes.csv", nodes, 
              delimiter = ",")


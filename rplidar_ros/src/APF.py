#!/usr/bin/env python
# -- coding: utf-8 --
import numpy as np
import math
import matplotlib as plt
import time

def vector(a,b):
    d=math.sqrt((a[0]-b[0])**2+ (a[1]-b[1])**2)
    direction=[0,0]
    if a[0]-b[0]==0:
        direction[0]=0
    else:
        direction[0]=-(a[0]-b[0])/d
    if a[1] - b[1] == 0:
        direction[1] = 0
    else:
        direction[1] =-(a[1]-b[1])/d
    return direction
def univector(u):
    d=math.sqrt((u[0])**2+ (u[1])**2)
    if d == 0:
        return [0,0]
    direction= [(u[0])/d,(u[1])/d]
    return direction

def attractForce(goal,current,a,b):
    d=math.sqrt((goal[0]-current[0])**2+ (goal[1]-current[1])**2)
    F=(a/(b**2))*d+a/((d+b)**2)
    d=vector(current,goal)
    Fd=[d[0] * F * 0.5, d[1] * F * 0.5]
    return [0,0]#Fd

def pulseForce(obstacle,current,goal,a,b,R1,R2):
    d=math.sqrt((obstacle[0]-current[0])**2+ (obstacle[1]-current[1])**2)
    d1=math.sqrt((goal[0]-current[0])**2+ (goal[1]-current[1])**2)
    if d<R1 and d1>=R2:
      F=0.02*(a/(b**2))*(((R1-d)*5)**1.2)
    if d<R1 and d1<R2:
      F= ((d1/R2)**2)*(a/(b**2))*(((R1-d)*5)**1.2)
    if d>=R1:
      F=0
    d=vector(obstacle,current)
    Fd=[d[0] * F, d[1] * F]
    return Fd

def checkobstacles(pic):
    d=np.where(pic>254)
    return d

def F_all(current,pic,goal,a,b,R1,R2,path):
    d=[[i,k] for i in range(int(current[0])-R1,int(current[0])+R1+1,1) for k in range(int(current[1]),int(current[1])+R1+1,1) if(pic[i,k]==0)]
    d1=[pulseForce(d[i],current,goal,a,b,R1,R2) for i in range(len(d))]
    #math.sqrt((d[i][0] - current[0]) ** 2 + (d[i][1] - current[1]) ** 2)
    F_all=np.sum(d1,axis=0) + attractForce(goal,current,a,b)
    if F_all[0]<=50 and F_all[1]<=50 and len(path[0])>=2:
        v2=math.atan2(path[1][-1]-path[1][-2], path[0][-1]-path[0][-2]) - math.pi/2
        F_all=[F_all[0]+math.cos(v2), F_all[1]+math.sin(v2)]
    return F_all


def APFplanning(start,goal,a,b,v,pic,R1,R2,r):
    #a = time.time()
    w=0
    current=start
    path=[[],[]]
    if (abs(goal[0]-current[0])+abs(goal[1]-current[1]))>=r :
        direct=univector(F_all(current,pic,goal,a,b,R1,R2,path))
        current=np.array(current)+v*np.array(direct)
        path[0].append(current[0])
        path[1].append(current[1])
    elif  (abs(goal[0]-current[0])+abs(goal[1]-current[1]))<=r:
        path[0].append(current[0])
        path[1].append(current[1]) 
        
    size = len(path[0])
    # return (int(path[0][1]),int(path[1][1]))
    if int(path[1][0])==0:
        w=math.pi/2
    else:
        w=math.atan(int((path[1][0]-start[1])*100.0)/int((path[0][0]-start[0])*100.0))
    
    return w

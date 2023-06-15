import numpy as np
from matplotlib.path import Path

#class defining walls
class wall:
    def __init__(self,startPoint,endPoint,type):
        self.startPoint = startPoint
        self.endPoint = endPoint
        self.type = type

    def xvals(self):
        return [self.startPoint[0],self.endPoint[0]]
    
    def yvals(self):
        return[self.startPoint[1],self.endPoint[1]]
    
#class defining lines
class line:
    def __init__(self,startPoint,endPoint):
        self.startPoint = startPoint
        self.endPoint = endPoint
        
    def xvals(self):
        return [self.startPoint[0],self.endPoint[0]]
    
    def yvals(self):
        return[self.startPoint[1],self.endPoint[1]]
    
#function which computes the intersection of two lines, given all four endpoints
def computeIntersection(pt1,pt2,pt3,pt4):
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]
    x3 = pt3[0]
    y3 = pt3[1]
    x4 = pt4[0]
    y4 = pt4[1]
    e = +0.01#epsilon. Deals with floating point errors
    try:
        Px = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))
        Py = ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)) / ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4))
        # print(Px,Py)
    except:
        return [None,None]
    
    #checking that [Px,Py] represents the correct vector (since this eqn calculates intersections for infinite length lines)
    if (Px > x1 + e and Px > x2 + e) or (Py > y1 + e and Py > y2 + e) or (Px < x1 - e and Px < x2 - e) or (Py < y1 - e and Py < y2 - e):
        return [None,None]
    if (Px > x3 + e and Px > x4 + e) or (Py > y3 + e and Py > y4 + e) or (Px < x3 - e and Px < x4 - e) or (Py < y3 - e and Py < y4 - e):
        return [None,None]
    
    return [Px,Py]

#get length of line between two points
def lineLength(pt1,pt2):
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]
    try:
        return np.sqrt((x2-x1)**2+(y2-y1)**2)
    except:
        return 10
    
def lengthToOpenDoor(angle,doorStart,doorEnd,pointStart,length):
    x1 = pointStart[0]
    y1 = pointStart[1]
    x2 = pointStart[0] + length * np.cos(angle)
    y2 = pointStart[1] + length * np.sin(angle)
    x3 = doorStart[0]
    y3 = doorStart[1]
    x4 = doorEnd[0]
    y4 = doorEnd[1]

    [Px,Py] = computeIntersection([x1,y1],[x2,y2],[x3,y3],[x4,y4])
    length = lineLength(pointStart,[Px,Py])
    return length

def bezier(P,res,spacing = 'lin'):
    x0 = P[0]
    y0 = P[1]
    x1 = P[2]
    y1 = P[3]
    x2 = P[4]
    y2 = P[5]
    x3 = P[6]
    y3 = P[7]

    if spacing == 'lin':
        t = np.linspace(0,1,res)
    else:
        t = np.linspace(0,1,res)


    X = (1-t)**3 * x0 + t * x1 * (3*(1-t)**2) + x2 * 3*(1-t)*t**2 + x3 * t**3
    Y = (1-t)**3 * y0 + t * y1 * (3*(1-t)**2) + y2 * 3*(1-t)*t**2 + y3 * t**3
    return [X,Y]

def getMidPoint(pt1,pt2):
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]

    mx = (x1 + x2)/2
    my = (y1 + y2)/2
    return [mx,my]

def getAngleBetween2Points(pt1,pt2):
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]

    theta = np.arctan2((y2-y1),(x2-x1))
    return theta

def newPointAlongNormal(pt1,pt2,midpt,dist):
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]
    mx = midpt[0]
    my = midpt[1]

    #points for the normal
    dx = x2-x1
    dy = y2-y1

    #for now, randomly choosing one vector and hoping it's the right one TODO
    nx = dy + mx
    ny = -dx + my

    #normalising
    deltaX = nx-mx
    deltaY = ny-my
    L = np.sqrt(deltaX**2 + deltaY**2)

    normX = mx + dist*deltaX/L
    normY = my + dist*deltaY/L

    return [normX,normY]

#new point is [dist] length from pt1
def pointOnLineAtDist(pt1,pt2,dist):
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]

    #normalising
    deltaX = x2-x1
    deltaY = y2-y1
    L = np.sqrt(deltaX**2 + deltaY**2)

    newX = x1 + dist*deltaX/L
    newY = y1 + dist*deltaY/L


    return [newX,newY]

def reverseNormal(midpt,normpt):
    [newX,newY] = pointOnLineAtDist(normpt,midpt,20)#TODO
    newNorm = [newX,newY]
    return newNorm

def createCabinetPath(walls):
    vertices = []
    codes = []
    for i in range(len(walls)-1):#discounting last entry which will be a thickness
        vertices.append(walls[i].startPoint)
        if i == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    
    vertices.append(walls[0].startPoint)
    codes.append(Path.CLOSEPOLY)

    return Path(vertices,codes)

def isPointOnLine(pt1,pt2,point):
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]
    xp = point[0]
    yp = point[1]
    epsilon = 0.01
    LHS = (yp-y1)*(x1-x2)
    RHS = (xp-x1)*(y1-y2)

    return -epsilon < LHS-RHS < epsilon

def pointsTheSame(pt1,pt2):
    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]
    epsilon = 0.01

    Xequal = -epsilon < x1-x2 < epsilon
    Yequal = -epsilon < y1-y2 < epsilon
    return Xequal and Yequal
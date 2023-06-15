#designing pull-out shelving for arbitrary cabinet dimensions

# initial attempt: disregarding rotation. Only translational movement along a path.
# steps:
#1. create dimensions of cabinet. Specify door.
#2. create path to move along. Specify start and end points.
#3. for each direction, traverse path to find shortest vector. Save that.
#4. assemble vectors around a single start point to create the shelf.

#step 1. Cabinent.
#The cabinet will be constructed of line segments, each denoted by a linear equation. 

import matplotlib.pyplot as plt
import numpy as np
from geometryFunctions import *
from highLevelFunctions import *
from matplotlib.animation import FuncAnimation
import bisect

#must be constructed such that startpoint is the same as the endpoint for the next vertex in the anticlockwise direction
backwall  = wall([0,3],[5,3],"solid")
sidewallL = wall([0,1],[0,3],"solid")
sidewallR = wall([5,3],[5,1],"solid")
frontwall = wall([5,1],[2.5,1],"solid")
door      = wall([0,1],[2.5,1],"open")
thickness1 = wall(door.startPoint,[door.startPoint[0],door.startPoint[1]-0.1],"solid")
thickness2 = wall(door.endPoint,[door.endPoint[0],door.endPoint[1]-0.1],"solid")


#will be converted to matplotlib path, which requires vertices in order.
#convention will be: append walls in clockwise order starting from left side of door.
#TODO may be wrong, but for now: leave the thickness as the last entry and do not count it for purposes of the path.
cabinetWalls = []
cabinetWalls.append(sidewallL)
cabinetWalls.append(backwall)
cabinetWalls.append(sidewallR)
cabinetWalls.append(frontwall)
cabinetWalls.append(door)
# cabinetWalls.append(thickness1)
cabinetWalls.append(thickness2)

#fiddle-able variables
lineNumPoints = 1000
numAngles = 400
maxLength = 10
vectorLength = 10 #TODO: should be tied to dimension of cabinet ideally

#step 2. Two lines, each defined by bezier curves. Line1 defines the translational component of the shelf's movement. Line2 defines the rotational component.

#use bezier curve to create path for shelf to follow.
# line1 = [3,2.2 , 1.8,1.5 , 1.8,-0.1 , 0.3,2.5]
# [lineX,lineY] = bezier(line1,lineNumPoints)

#using circular path for shelf translational motion
theta = np.linspace(-0.4*np.pi,0.15*np.pi,lineNumPoints)
theta = theta[::-1]#reverse the points so the start point is inside the cabinet
lineX = 1.5*np.cos(theta)
lineY = 1.5*np.sin(theta)+1

#use another bezier to define the angle through which the shelf turns at each point along the line
line2 = [0,0 , 0.1*lineNumPoints,50 , 0.9*lineNumPoints,20 , lineNumPoints,70]
# line2 = [0,0 , 0.1*lineNumPoints,50 , 0.9*lineNumPoints,20 , lineNumPoints,70]
[rotX,rotY] = bezier(line2,lineNumPoints)
rotY = rotY*np.pi/180
# rotY = np.zeros(lineNumPoints)
# rotY = np.linspace(0,100*np.pi/180,lineNumPoints)#TEMP for testing


#step 3: get vectors for each angle, traverse path, find shortest vector for each.

#3.a create list of angles
angleList = np.linspace(0*np.pi,2*np.pi,num=numAngles,endpoint=False)
angleList = angleList.tolist()

#for each point on the line, add a vector with appropriate length and angle. Get resultant end point to find line points. Calculate intersections with all walls. Find minimum

# print(angleList*180/np.pi)

print("getting max vector length for each angle")
[minLengthList,linePosForMinL] = computeMinLengthForEachAngle(maxLength,angleList,lineNumPoints,lineX,lineY,vectorLength,rotY,cabinetWalls)
print("done")
#initiating plot
fig, ax = plt.subplots()

print("assembling vectors around single point")
[xFinal,yFinal] = assembleVectorsAroundPoint(lineX,lineY,numAngles,vectorLength,angleList,rotY,door,minLengthList,linePosForMinL,maxLength)
print("done")

print("secondary stage")
#expanding the area in a secondary stage. Find all sides with length above threshold, get midpoint, create new point expanded along the normal
lengthThreshold = 0.15
allDistancesBelowThreshold = False
distanceTooLong = False
iterativeExtensionAlongNormal = 10
test_iter = 0
max_test_iter = 60#when < test_iter, nothing here is used
while not allDistancesBelowThreshold:
    distanceTooLong = False
    # xFinal3 = xFinal
    # yFinal3 = yFinal
    for i in range(len(xFinal)):

        #making sure the thing can wrap around
        x1 = xFinal[i]
        y1 = yFinal[i]
        try:#getting next point. Wraps around when at end of the list
            x2 = xFinal[i+1]
            y2 = yFinal[i+1]
        except:
            x2 = xFinal[0]
            y2 = yFinal[0]

        length = lineLength([x1,y1],[x2,y2])#get length between two neighbouring points

        
        if (length > lengthThreshold and (test_iter < max_test_iter)):#if length too large, get midpoint. Create new point along the normal (for now just iterate on small distance)
            
            #get midpoint of offending section
            [mx,my] = getMidPoint([x1,y1],[x2,y2])

            #get normal to line segment at midpoint
            #normalise then scale to iterativeExtensionAlongNormal
            # print([mx,my])
            [normX,normY] = newPointAlongNormal([x1,y1],[x2,y2],[mx,my],iterativeExtensionAlongNormal)

            
            #find maximum extensible length along normal before hitting a wall, generate new point along that vector.
            [newX,newY] = ComputeMaxLengthAlongNormal(lineX,lineY,rotY,cabinetWalls,[mx,my],[normX,normY])
            ax.plot(mx,my,'x')
            xFinal.insert(i+1,newX)
            yFinal.insert(i+1,newY)
            # i = i + 1
            test_iter = test_iter + 1
            
            distanceTooLong = True#if new points needed to be added, continue to loop
    
    # xFinal = xFinal2
    # yFInal = yFinal2
    if distanceTooLong == False or len(xFinal) > 1000:
        allDistancesBelowThreshold = True

print("secondary stage complete")

print("beging plotting")
#plotting from here on
ax.plot(xFinal,yFinal,'-o')#plot the outline of the shelf
ax.plot(rotX/lineNumPoints + 3,rotY - 1)#plot the rotational curve
ax.plot(lineX,lineY,'g:')#plot the path of travel

#plot the walls (door different colour)
for wall in cabinetWalls:
    ax.plot(wall.xvals(),wall.yvals(),'k' if wall.type == "solid" else 'r--')



xFinal2 = []
yFinal2 = []
line, = ax.plot([], [],'-s', lw=3)
#animating the shelf to move along the pathway
#TODO make this work with new method

#testing
# xFinal = np.linspace(0,1,200)+2 - lineX[0]
# yFinal = 1*np.sin(xFinal)+2 - lineY[0]
xFinal = xFinal - lineX[0]
yFinal = yFinal - lineY[0]

def init():
    line.set_data([], [])
    return line,
def animate(i):
    xFinal2 = []
    yFinal2 = []
    #for each point in the line, redraw the whole shelf
    
    for j in range(len(xFinal)):
        tx = lineX[i] - lineX[0]
        ty = lineY[i] - lineY[0]
        theta = rotY[i]
        # xFinal[j] = xFinal[j] - lineX[0]
        # yFinal[j] = yFinal[j] - lineY[0]

        xFinal2.append(xFinal[j]*np.cos(theta) - yFinal[j]*np.sin(theta)+tx+lineX[0])
        yFinal2.append(xFinal[j]*np.sin(theta) + yFinal[j]*np.cos(theta)+ty+lineY[0])



    # for j in range(len(angleList)):
    #     xFinal2.append((minLengthList[j] * np.cos(angleList[j]+rotY[i]) + lineX[i])*1)
    #     yFinal2.append((minLengthList[j] * np.sin(angleList[j]+rotY[i]) + lineY[i])*1)
    xFinal2.append(lineX[i])
    yFinal2.append(lineY[i])
    # print(lineLength([lineX[i],lineY[i]],[xFinal2[-2],yFinal2[-2]]))
    line.set_data(xFinal2, yFinal2)
    return line,



def animate2(i):
    xFinal2 = []
    yFinal2 = []
    #for each point in the line, redraw the whole shelf
    
    for j in range(len(angleList)):
        xFinal2.append((minLengthList[j] * np.cos(angleList[j]+rotY[i]) + lineX[i])*1)
        yFinal2.append((minLengthList[j] * np.sin(angleList[j]+rotY[i]) + lineY[i])*1)
    xFinal2.append(lineX[i])
    yFinal2.append(lineY[i])
    # xFinal2.append(lineX[i] + minLengthList[i] * np.cos(angleList[i]))
    # yFinal2.append(lineY[i] + minLengthList[i] * np.sin(angleList[i]))

    line.set_data(xFinal2, yFinal2)
    return line,

anim = FuncAnimation(fig, animate, init_func=init,
                               frames=lineNumPoints, interval=4000/lineNumPoints, blit=True)


# anim2 = FuncAnimation(fig, animate2, init_func=init,
#                                frames=numAngles, interval=4000/numAngles, blit=True)

# anim.save('rotating_shelf.gif', writer='imagemagick')

plt.axis('equal')
plt.show()
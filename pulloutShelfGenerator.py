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
from config import *

#must be constructed such that startpoint is the same as the endpoint for the next vertex in the anticlockwise direction
#cabinet 1
# backwall  = wall([0,3],[5,3],"solid")
# sidewallL = wall([0,1],[0,3],"solid")
# sidewallR = wall([5,3],[5,1],"solid")
# frontwall = wall([5,1],[2.5,1],"solid")
# door      = wall([0,1],[2.5,1],"open")
# thickness1 = wall(door.startPoint,[door.startPoint[0],door.startPoint[1]-0.1],"solid")
# thickness2 = wall(door.endPoint,[door.endPoint[0],door.endPoint[1]-0.1],"solid")
# pivot = sidewallL.startPoint

# #will be converted to matplotlib path, which requires vertices in order.
# #convention will be: append walls in clockwise order starting from left side of door.
# #TODO may be wrong, but for now: leave the thickness as the last entry and do not count it for purposes of the path.
# cabinetWalls = []
# cabinetWalls.append(sidewallL)
# cabinetWalls.append(backwall)
# cabinetWalls.append(sidewallR)
# cabinetWalls.append(frontwall)
# cabinetWalls.append(door)
# # cabinetWalls.append(thickness1)
# cabinetWalls.append(thickness2)

#CABINET 1
# from cabinetsAndPaths import cabinet1 as cabinetWalls
# from cabinetsAndPaths import lineX1 as lineX
# from cabinetsAndPaths import lineY1 as lineY
# from cabinetsAndPaths import rotX1 as rotX
# from cabinetsAndPaths import rotY1 as rotY

#CABINET 2
from cabinetsAndPaths import cabinet2 as cabinetWalls
from cabinetsAndPaths import lineX2 as lineX
from cabinetsAndPaths import lineY2 as lineY
from cabinetsAndPaths import rotX2 as rotX
from cabinetsAndPaths import rotY2 as rotY
#step 2. Two lines, each defined by bezier curves. Line1 defines the translational component of the shelf's movement. Line2 defines the rotational component.


# #using circular path for shelf translational motion
# theta = np.linspace(-0.4*np.pi,0.15*np.pi,lineNumPoints)
# theta = theta[::-1]#reverse the points so the start point is inside the cabinet
# lineX = 1.5*np.cos(theta)+pivot[0]
# lineY = 1.5*np.sin(theta)+pivot[1]

# #use another bezier to define the angle through which the shelf turns at each point along the line
# line2 = [0,0 , 0.1*lineNumPoints,50 , 0.9*lineNumPoints,20 , lineNumPoints,70]
# # line2 = [0,0 , 0.1*lineNumPoints,50 , 0.9*lineNumPoints,20 , lineNumPoints,70]
# [rotX,rotY] = bezier(line2,lineNumPoints)
# rotY = rotY*np.pi/180
# # rotY = np.zeros(lineNumPoints)
# # rotY = np.linspace(0,100*np.pi/180,lineNumPoints)#TEMP for testing


#step 3: get vectors for each angle, traverse path, find shortest vector for each.

#3.a create list of angles
angleList = np.linspace(0*np.pi,2*np.pi,num=numAngles,endpoint=False)
angleList = angleList.tolist()

#for each point on the line, add a vector with appropriate length and angle. Get resultant end point to find line points. Calculate intersections with all walls. Find minimum


print("getting max vector length for each angle")
[minLengthList,linePosForMinL] = computeMinLengthForEachAngle(maxLength,angleList,lineNumPoints,lineX,lineY,vectorLength,rotY,cabinetWalls)
print("done")

#initiating plot
fig, ax = plt.subplots()

#find door
door = None
for wall in cabinetWalls:
    if wall.type == "open":
        door = wall

print("assembling vectors around single point")
[xShelfOutline,yShelfOutline] = assembleVectorsAroundPoint(lineX,lineY,numAngles,vectorLength,angleList,rotY,door,minLengthList,linePosForMinL,maxLength)
print("done")

print("Begin Midpoint-Normal Extension")
#expanding the area in a secondary stage. Find all sides with length above threshold, get midpoint, create new point expanded along the normal

allDistancesBelowThreshold = False
distanceTooLong = False
iterativeExtensionAlongNormal = MAX_TEST_VECTOR_LENGTH
test_iter = 0
max_test_iter = 100#when < test_iter, nothing here is used
master_iters = 0
max_master_iters = 1000

# looping through all pairs of adjacent points until the distance between each pair is below the threshold.
# if distance too large, find midpoint, get vector normal to pt1 & pt2, find nearest intersection with wall, and insert a point there into the shelf outline
while not allDistancesBelowThreshold:
    distanceTooLong = False
    firstPointOnWall = False
    secondPointOnWall = False
    continueFlag = False
    testBool = False
    
    for i in range(len(xShelfOutline)):

        #making sure the thing can wrap around
        x1 = xShelfOutline[i]
        y1 = yShelfOutline[i]
        try:#getting next point. Wraps around when at end of the list
            x2 = xShelfOutline[i+1]
            y2 = yShelfOutline[i+1]
        except:
            x2 = xShelfOutline[0]
            y2 = yShelfOutline[0]

        length = lineLength([x1,y1],[x2,y2])#get length between two neighbouring points


        #edge case: if both points are on the same wall, just make a new midpt and leave it at that
        # instead of trying to expand along an undefined normal vector (it could go either way)
        if length > lengthThreshold and test_iter < max_test_iter:
            for wall in cabinetWalls:
                if not pointsTheSame([x1,y1],[x2,y2]):
                    firstPointOnWall = isPointOnLine(wall.startPoint,wall.endPoint,[x1,y1])
                    if firstPointOnWall:
                        secondPointOnWall = isPointOnLine(wall.startPoint,wall.endPoint,[x2,y2])

                        if secondPointOnWall:
                            [newX,newY] = getMidPoint([x1,y1],[x2,y2])
                            if newX == None:
                                print("whoops")
                            xShelfOutline.insert(i+1,newX)
                            yShelfOutline.insert(i+1,newY)
                            # print("wall point inserted",newX,newY,i)
                            continueFlag = True
                            break
            if continueFlag:
                continue

        #if length too large, get midpoint. Create new point along the normal at intersection with nearest wall
        if (length > lengthThreshold and (test_iter < max_test_iter)):
            #get midpoint of offending section
            [mx,my] = getMidPoint([x1,y1],[x2,y2])

            #get normal to line segment at midpoint
            #normalise then scale to iterativeExtensionAlongNormal
            [normX,normY] = newPointAlongNormal([x1,y1],[x2,y2],[mx,my],iterativeExtensionAlongNormal)

            
            #find maximum extensible length along normal before hitting a wall, generate new point along that vector.
            [newX,newY] = ComputeMaxLengthAlongNormal(lineX,lineY,rotY,cabinetWalls,[mx,my],[normX,normY])
            # ax.plot(mx,my,'x')
            xShelfOutline.insert(i+1,newX)
            yShelfOutline.insert(i+1,newY)
            if newX == None:
                print("whoops2")
            # print("line point inserted",newX,newY,i)
            # i = i + 1
            test_iter = test_iter + 1
            if testBool == True:
                distanceTooLong = True#if new points needed to be added, continue to loop
                # testBool = False
            testBool = True


    for i in range(len(xShelfOutline)):
        #making sure the thing can wrap around
        x1 = xShelfOutline[i]
        y1 = yShelfOutline[i]
        try:#getting next point. Wraps around when at end of the list
            x2 = xShelfOutline[i+1]
            y2 = yShelfOutline[i+1]
        except:
            x2 = xShelfOutline[0]
            y2 = yShelfOutline[0]

        length = lineLength([x1,y1],[x2,y2])#get length between two neighbouring points

        if length > lengthThreshold:
            distanceTooLong = True

    if distanceTooLong == False or master_iters >= max_master_iters or test_iter >= max_test_iter:
        allDistancesBelowThreshold = True
        if distanceTooLong == False:
            print("all distances below threshold")
        if test_iter >= max_test_iter:
            print("iteration limit: too many points added in secondary stage")
        if master_iters >= max_master_iters:
            print("iteration limit: master_iters too large")
    master_iters = master_iters + 1

print("secondary stage complete")

print("begin plotting")
#plotting from here on
ax.plot(xShelfOutline,yShelfOutline,'-b')#plot the outline of the shelf
ax.plot(rotX/lineNumPoints + 3,rotY - 1)#plot the rotational curve
ax.plot(lineX,lineY,'g:')#plot the path of travel

#plot the walls (door different colour)
for wall in cabinetWalls:
    ax.plot(wall.xvals(),wall.yvals(),'k' if wall.type == "solid" else 'r--')



xShelfOutline2 = []
yShelfOutline2 = []
line, = ax.plot([], [],'-gs', lw=3)
#animating the shelf to move along the pathway
#TODO make this work with new method

#testing
# xShelfOutline = np.linspace(0,1,200)+2 - lineX[0]
# yShelfOutline = 1*np.sin(xShelfOutline)+2 - lineY[0]
xShelfOutline = xShelfOutline - lineX[0]
yShelfOutline = yShelfOutline - lineY[0]

def init():
    line.set_data([], [])
    return line,
def animate(i):
    xShelfOutline2 = []
    yShelfOutline2 = []
    #for each point in the line, redraw the whole shelf
    
    for j in range(len(xShelfOutline)):
        tx = lineX[i] - lineX[0]
        ty = lineY[i] - lineY[0]
        theta = rotY[i] - rotY[0]
        # xShelfOutline[j] = xShelfOutline[j] - lineX[0]
        # yShelfOutline[j] = yShelfOutline[j] - lineY[0]

        xShelfOutline2.append(xShelfOutline[j]*np.cos(theta) - yShelfOutline[j]*np.sin(theta)+tx+lineX[0])
        yShelfOutline2.append(xShelfOutline[j]*np.sin(theta) + yShelfOutline[j]*np.cos(theta)+ty+lineY[0])



    # for j in range(len(angleList)):
    #     xShelfOutline2.append((minLengthList[j] * np.cos(angleList[j]+rotY[i]) + lineX[i])*1)
    #     yShelfOutline2.append((minLengthList[j] * np.sin(angleList[j]+rotY[i]) + lineY[i])*1)
    xShelfOutline2.append(lineX[i])
    yShelfOutline2.append(lineY[i])
    # print(lineLength([lineX[i],lineY[i]],[xShelfOutline2[-2],yShelfOutline2[-2]]))
    line.set_data(xShelfOutline2, yShelfOutline2)
    return line,



def animate2(i):
    xShelfOutline2 = []
    yShelfOutline2 = []
    #for each point in the line, redraw the whole shelf
    
    for j in range(len(angleList)):
        xShelfOutline2.append((minLengthList[j] * np.cos(angleList[j]+rotY[i]) + lineX[i])*1)
        yShelfOutline2.append((minLengthList[j] * np.sin(angleList[j]+rotY[i]) + lineY[i])*1)
    xShelfOutline2.append(lineX[i])
    yShelfOutline2.append(lineY[i])
    # xShelfOutline2.append(lineX[i] + minLengthList[i] * np.cos(angleList[i]))
    # yShelfOutline2.append(lineY[i] + minLengthList[i] * np.sin(angleList[i]))

    line.set_data(xShelfOutline2, yShelfOutline2)
    return line,

anim = FuncAnimation(fig, animate, init_func=init,
                               frames=lineNumPoints, interval=4000/lineNumPoints, blit=True)


# anim2 = FuncAnimation(fig, animate2, init_func=init,
#                                frames=numAngles, interval=4000/numAngles, blit=True)

# anim.save('rotating_shelf.gif', writer='Pillow',fps=60)
plt.axis('equal')
# plt.ylim([-6,5])
# plt.xlim([-2,9])

plt.show()
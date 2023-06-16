from geometryFunctions import *

#CABINET 1
backwall  = wall([0,3],[5,3],"solid")
sidewallL = wall([0,1],[0,3],"solid")
sidewallR = wall([5,3],[5,1],"solid")
frontwall = wall([5,1],[2.5,1],"solid")
door      = wall([0,1],[2.5,1],"open")
thickness1 = wall(door.startPoint,[door.startPoint[0],door.startPoint[1]-0.1],"solid")
thickness2 = wall(door.endPoint,[door.endPoint[0],door.endPoint[1]-0.1],"solid")
pivot1 = sidewallL.startPoint

#will be converted to matplotlib path, which requires vertices in order.
#convention will be: append walls in clockwise order starting from left side of door.
#TODO may be wrong, but for now: leave the thickness as the last entry and do not count it for purposes of the path.
cabinet1 = []
cabinet1.append(sidewallL)
cabinet1.append(backwall)
cabinet1.append(sidewallR)
cabinet1.append(frontwall)
cabinet1.append(door)
# cabinet1.append(thickness1)
cabinet1.append(thickness2)

#PATH 1
#using circular path for shelf translational motion
theta = np.linspace(-0.4*np.pi,0.15*np.pi,lineNumPoints)
theta = theta[::-1]#reverse the points so the start point is inside the cabinet
lineX1 = 1.5*np.cos(theta)+pivot1[0]
lineY1 = 1.5*np.sin(theta)+pivot1[1]

#use another bezier to define the angle through which the shelf turns at each point along the line
#note: starting at 20deg has an interesting effect
lineRotation = [0,0 , 0.1*lineNumPoints,50 , 0.9*lineNumPoints,20 , lineNumPoints,70]
# line2 = [0,0 , 0.1*lineNumPoints,50 , 0.9*lineNumPoints,20 , lineNumPoints,70]
[rotX1,rotY1] = bezier(lineRotation,lineNumPoints)
rotY1 = rotY1*np.pi/180


#CABINET 2
wall1 = wall([0,-2],[0,3],"solid")
wall2 = wall(wall1.endPoint,[7,3],"solid")
wall3 = wall(wall2.endPoint,[7,0],"solid")
wall4 = wall(wall3.endPoint,[2.5,1],"solid")
wall5 = wall(wall4.endPoint,wall1.startPoint,"open")
wallThick = wall(wall5.endPoint,[wall5.endPoint[0]-0.01,wall5.endPoint[1]-0.01],"solid")
cabinet2 = []
cabinet2.append(wall1)
cabinet2.append(wall2)
cabinet2.append(wall3)
cabinet2.append(wall4)
cabinet2.append(wall5)
cabinet2.append(wallThick)
pivot2 = wall4.endPoint

#PATH 2
theta = np.linspace(0.6*np.pi,1.2*np.pi,lineNumPoints)
lineX2 = 1.25*np.cos(theta)+pivot2[0]
lineY2 = 1.25*np.sin(theta)+pivot2[1]

#use another bezier to define the angle through which the shelf turns at each point along the line
lineRotation = [0,-40 , 0.1*lineNumPoints,-30 , 0.9*lineNumPoints,10 , lineNumPoints,20]
# line2 = [0,0 , 0.1*lineNumPoints,50 , 0.9*lineNumPoints,20 , lineNumPoints,70]
[rotX2,rotY2] = bezier(lineRotation,lineNumPoints)
rotY2 = rotY2*np.pi/180
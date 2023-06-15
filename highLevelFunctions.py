import numpy as np
from geometryFunctions import *
import matplotlib.pyplot as plt
from matplotlib.path import Path


def computeMinLengthForEachAngle(maxLength,angleList,lineNumPoints,lineX,lineY,vectorLength,rotY,cabinetWalls):
    minLengthList = []
    linePosForMinL = []
    length = maxLength

    devpt = 7#for debugging

    for angle in angleList:
        length = maxLength#reset length to max for each new angle

        for i in range(lineNumPoints):#iterate through all points on line to find shortest distance to a wall
            newX = lineX[i] + vectorLength * np.cos(angle+rotY[i])
            newY = lineY[i] + vectorLength * np.sin(angle+rotY[i])
            
            for wall in cabinetWalls:
                if wall.type == "open":
                    continue
                [Px,Py] = computeIntersection([lineX[i],lineY[i]],[newX,newY],wall.startPoint,wall.endPoint)
                lengthTemp = lineLength([lineX[i],lineY[i]],[Px,Py])
                # if (angle == angleList[devpt]):
                    # print(Px,Py,i,lineX[i],lineY[i],newX,newY,wall.startPoint,wall.endPoint)
                if lengthTemp < length:
                    length = lengthTemp
                    #debugging stuff
                    # Px_dev = Px
                    # Py_dev = Py
                    # lX_dev = lineX[i]
                    # lY_dev = lineY[i]
                    # nX_dev = newX
                    # nY_dev = newY
                    i_dev = i
                    # devLength = length
                    # print(angle,i,wall,length)
                    
        minLengthList.append(length)#take the shortest length found and append it to the list (position in this list is same as angle position in angle list. Thusly they are associated)
        try:
            linePosForMinL.append(i_dev)
        except:
            linePosForMinL.append(0)#TODO weird edge case that would require this, no idea if it's throwing anything off
            
        # if (angle == angleList[devpt]):
        #     print (i_dev)
        #     ax.plot(lineX[i_dev],lineY[i_dev],'g|',ms = 20)
        #     ax.plot(lineX[i_dev] + vectorLength * np.cos(angle),lineY[i_dev] + vectorLength * np.sin(angle),'ro')
        #     ax.plot(Px_dev,Py_dev,'mo')
        #     print (devLength)
        #     print(lineLength([lineX[i_dev],lineY[i_dev]],[Px_dev,Py_dev]))

        # ax.plot([lX_dev,nX_dev],[lY_dev,nY_dev],"ro-")
        # ax.plot([lX_dev,lineX[0] + length * np.cos(angle+rotY[0])],[lY_dev,lineY[0] + length * np.sin(angle+rotY[0])],"-yo")
        # ax.plot([lX_dev,lineX[i_dev] + length * np.cos(angle+rotY[i_dev])],[lY_dev,lineY[i_dev] + length * np.sin(angle+rotY[i_dev])],"-go")
        # ax.plot([lineX[0],lineX[0] + length*np.cos(angle)],[lineY[0],lineY[0] + length*np.sin(angle)],'bo-')
        # print(lX_dev,',',lY_dev,',',nX_dev,',',nY_dev,',',Px_dev,',',Py_dev,',',length)
    
    return [minLengthList,linePosForMinL]

#TODO change lineX,y to take in points[0] instead of the whole vector
def assembleVectorsAroundPoint(lineX,lineY,numAngles,vectorLength,angleList,rotY,door,minLengthList,linePosForMinL,maxLength):

    #step 4. Assemble all the vectors around a single point.
    finalPoint = [lineX[0],lineY[0]]#the point around which the arranging happens

    #step 4.1 make sure that any vectors passing through the door stop at the door (instead of wrapping around to hit a wall from the outside)

    for i in range(numAngles):
        test_x = lineX[0] + vectorLength * np.cos(angleList[i] + rotY[0])
        test_y = lineY[0] + vectorLength * np.sin(angleList[i] + rotY[0])
        [test_px,test_py] = computeIntersection(finalPoint,[test_x,test_y],door.startPoint,door.endPoint) 
        if test_px != None or test_py != None:#if the line does intersect with the door
            lengthTemp = lengthToOpenDoor(angleList[i] + rotY[0],door.startPoint,door.endPoint,finalPoint,minLengthList[i])
            if lengthTemp < minLengthList[i]:
                minLengthList[i] = lengthTemp



    xFinal = []
    yFinal = []
    for i in range(numAngles):
        theta = linePosForMinL[i]
        if minLengthList[i] == maxLength:#length will only be max if the vector never intersects with a wall. Therefore it must have gone through the door.
            
            minLengthList[i] = lengthToOpenDoor(angleList[i]+rotY[theta],door.startPoint,door.endPoint,finalPoint,minLengthList[i])#find distance to door along this angle, set length to that
        
        
        
        xFinal.append(minLengthList[i] * np.cos(angleList[i]+rotY[0]) + finalPoint[0])#
        yFinal.append(minLengthList[i] * np.sin(angleList[i]+rotY[0]) + finalPoint[1])
        # print(xFinal[i],yFinal[i])

    return [xFinal,yFinal]

#figure out maximum normal vector length by checking intersections with all walls along length of travel
def ComputeMaxLengthAlongNormal(lineX,lineY,rotY,cabinetWalls,midpt,normpt):

    cabinetPath = createCabinetPath(cabinetWalls)
    reversedNormalFlag = False
    length = 10#TODO set up defined variables for all these things
    midpt[0] = midpt[0] - lineX[0]
    midpt[1] = midpt[1] - lineY[0]
    normpt[0] = normpt[0] - lineX[0]
    normpt[1] = normpt[1] - lineY[0]
    pt1 = midpt.copy()
    pt2 = normpt.copy()
    maxLengthList = []
    maxLengthList2 = []
    for i in range(len(lineX)):#TODO chenge back to check all points in lineX
        length = 10#TODO
        reversedNormalFlag = False
        #computing transformation relative to startpoint
        tx = lineX[i] - lineX[0]
        ty = lineY[i] - lineY[0]
        theta = rotY[i]
        
        #new midpt (x then y)
        pt1[0] = midpt[0]*np.cos(theta) - midpt[1]*np.sin(theta) + tx + lineX[0]
        pt1[1] = midpt[0]*np.sin(theta) + midpt[1]*np.cos(theta) + ty + lineY[0]

        #new normpt (x then y)
        pt2[0] = normpt[0]*np.cos(theta) - normpt[1]*np.sin(theta) + tx + lineX[0]
        pt2[1] = normpt[0]*np.sin(theta) + normpt[1]*np.cos(theta) + ty + lineY[0]

        #if midpoint is outside of cabinet, reverse the normal and find the shortest length like that
        if(not cabinetPath.contains_point(pt1,radius=0.0)):
            # print("outside",midpt[0]+lineX[0],midpt[1]+lineY[0])
            pt2 = reverseNormal(pt1,pt2)
            reversedNormalFlag = True

        

        for wall in cabinetWalls:
            if wall.type == "open":
                continue
            [Px,Py] = computeIntersection(pt1,pt2,wall.startPoint,wall.endPoint)
            lengthTemp = lineLength(pt1,[Px,Py])
            
            if abs(lengthTemp) < abs(length):
                if reversedNormalFlag:
                    lengthTemp = - lengthTemp
                    # print(lengthTemp,midpt[0]+lineX[0],midpt[1]+lineY[0])
                length = lengthTemp

        maxLengthList.append(length)
        

        #TODO if maximum length is (close to) zero, try the whole thing again but with the normal reversed (as the midpt may be outside the shape)
    #2nd idea: push the midpoint back towards the middle of the shape so the normal is always positive
    if (min(maxLengthList) < 0.1 and False):
        reversedNormalFlag = True
        newNormpt = reverseNormal(midpt,normpt)
        length = 10#TODO
        maxLengthList2 = []
        for i in range(len(lineX)):
            length = 10#TODO

            #computing transformation relative to startpoint
            tx = lineX[i] - lineX[0]
            ty = lineY[i] - lineY[0]
            theta = rotY[i]
            #new midpt (x then y)
            pt1[0] = midpt[0]*np.cos(theta) - midpt[1]*np.sin(theta) + tx + lineX[0]
            pt1[1] = midpt[0]*np.sin(theta) + midpt[1]*np.cos(theta) + ty + lineY[0]
            #new normpt (x then y)
            pt2[0] = newNormpt[0]*np.cos(theta) - newNormpt[1]*np.sin(theta) + tx + lineX[0]
            pt2[1] = newNormpt[0]*np.sin(theta) + newNormpt[1]*np.cos(theta) + ty + lineY[0]

            for wall in cabinetWalls:
                if wall.type == "open":
                    continue
                [Px,Py] = computeIntersection(pt1,pt2,wall.startPoint,wall.endPoint)
                lengthTemp = lineLength(pt1,[Px,Py])

                if lengthTemp < length:
                    length = lengthTemp

            maxLengthList2.append(length)

    # absLengthList = maxLengthList.copy()
    absLengthList = [abs(i) for i in maxLengthList]
    mindex = absLengthList.index(min(absLengthList))

    threshold = 1
    minLength = maxLengthList[0]
    for i in range(len(maxLengthList)):
        minTemp = maxLengthList[i]
        if minTemp < minLength and abs(minTemp) < threshold:
            minLength = minTemp

    # minLength = maxLengthList[mindex]
    midpt[0] = midpt[0] + lineX[0]#backtransforming to world coordinates
    midpt[1] = midpt[1] + lineY[0]
    # if (reversedNormalFlag and min(maxLengthList2) > min(maxLengthList)):#if larger distance found for reversed normal, use that instead
    #     normpt = newNormpt.copy()
    #     minLength = min(maxLengthList2)
    normpt[0] = normpt[0] + lineX[0]
    normpt[1] = normpt[1] + lineY[0]
    # print(min(maxLengthList),min(maxLengthList2))
    [newX,newY] = pointOnLineAtDist(midpt,normpt,minLength)
    return [newX,newY]


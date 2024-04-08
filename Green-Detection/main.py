import cv2 as cv
import numpy as np
import math

vd = cv.VideoCapture(0)

vd.set(cv.CAP_PROP_FRAME_WIDTH, 320);
vd.set(cv.CAP_PROP_FRAME_HEIGHT, 240);

while(True):
    #Getting the image
    ret, frame = vd.read()
    greenCount = [[None]*2]*4



    #HSV values for black
    lower_bound_black = np.array([0, 0, 0])
    upper_bound_black = np.array([179, 255, 60])

    #HSV values for green
    lower_bound_green = np.array([68,40,28])
    upper_bound_green = np.array([109, 206, 99])

    #Bluring
    '''blur = cv.blur(frame, (10,10))

    blur = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
    blurc = cv.medianBlur(frame,5)
    blurc = cv.cvtColor(blurc, cv.COLOR_BGR2GRAY)'''

    blur = cv.blur(frame, (10,10))
    blur = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    #Threshholding and finding Contours for black
    blackThresh = cv.inRange(blur, lower_bound_black, upper_bound_black);


    #blackThresh = cv.adaptiveThreshold(blurc,255,cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV,11,2)

    (contoursB, h) = cv.findContours(blackThresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    #print(len(contoursB))
    framecopy = frame.copy()
    contourcolor = (255, 0, 255)
    cv.drawContours(framecopy, contoursB, -1, contourcolor, 3)

    #Getting center
    cxB = 0
    cyB = 0
    if(len(contoursB) > 0):
        areas = [cv.contourArea(c) for c in contoursB]
        max_index = np.argmax(areas)
        M = cv.moments(contoursB[max_index])
        if(M['m00'] != 0):
            cxB = int(M['m10']/M['m00'])
            cyB = int(M['m01']/M['m00'])
            cv.circle(framecopy, (cxB, cyB), 7, (0, 0, 255), -1)

    #Threshholding and finding Contours for green
    greenThresh = cv.inRange(blur, lower_bound_green, upper_bound_green);
    (contoursG, h) = cv.findContours(greenThresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    #print(len(contoursG))
    contourcolor = (255, 0, 255)
    cv.drawContours(framecopy, contoursG, -1, contourcolor, 3)

    #Getting green centers
    areaArray = []
    count = 1
    for i, c in enumerate(contoursG):
        area = cv.contourArea(c)
        if area > 80:
            areaArray.append(area)

    cxG = 0
    cyG = 0
    count = 0

    centers = []
    for i in range(len(contoursG)):
        if cv.contourArea(contoursG[i]) < 80:
            continue
        height = int(((math.sqrt(cv.contourArea(contoursG[i]))) + 10))
        M = cv.moments(contoursG[i])
        cxG = int(M['m10']/M['m00'])
        cyG = int(M['m01']/M['m00'])
        if(cyG < 140 or cyG > 220):
            continue
        if((cyG + height) < 240):
            if(blackThresh[cyG + height, cxG] == 0):
                cv.circle(framecopy, (cxG, cyG+height), 10, (0, 255, 0), -1)
                centers.append((cxG, cyG))
                print(centers)
            cv.circle(framecopy, (cxG, cyG), 3, (0, 0, 0), -1)





    if (len(centers) == 2):
        print("Double Green")
    elif(len(centers) == 1):
        length = int(math.sqrt(cv.contourArea(contoursG[0]))) + 5
        height = int(((math.sqrt(cv.contourArea(contoursG[0]))) + 10))
        cv.circle(framecopy, (cxG+length, cyG), 10, (0, 255, 0), -1)
        #heightdown = int(math.sqrt(areaArray[0])) + 10
        areas = [cv.contourArea(c) for c in contoursG]
        max_index = np.argmax(areas)
        M = cv.moments(contoursG[max_index])
        if(M['m00'] != 0):
            cxG = int(M['m10']/M['m00'])
            cyG = int(M['m01']/M['m00'])
            #cv.circle(framecopy, (cxG, cyG), 7, (0, 0, 255), -1)
            if((cxG - length) > 0 and (cxG + length) < 320 and (cyG + height) < 240):
                cv.circle(framecopy, (cxG + length, cyG), 7, (0,0, 255), -1)
                cv.circle(framecopy, (cxG - length, cyG), 7, (0,0, 255), -1)
                cv.circle(framecopy, (cxG+length, cyG), 7, (0,0, 255), -1)

                #if(blackThresh[cyG + height, cxG] == 255):
                   # print("False Green")
                if (blackThresh[cyG, cxG + length] == 255):
                    print("Left Green")
                elif (blackThresh[cyG, cxG - length]):
                    print("Right Green")

    elif(len(centers) == 0):
        print("Straight")
    #Displaying image
    cv.imshow("Black Thresh", blackThresh);
    cv.imshow("Green Thresh", greenThresh);
    cv.imshow("Contours", framecopy)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vd.release()
cv.destroyAllWindows
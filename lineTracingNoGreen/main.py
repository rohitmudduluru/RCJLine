import cv2 as cv
import numpy as np
import math
import serial
import time

ser = serial.Serial('/dev/ttyS0',9600);
vd = cv.VideoCapture(0)

width = 320
height = 240

vd.set(cv.CAP_PROP_FRAME_WIDTH, width);
vd.set(cv.CAP_PROP_FRAME_HEIGHT, height);
left = width//10
right = width - width//10
top = height//8

cset = 0
pset = 0
last_error = 0
integral = 0
haveRead = False
def read():
    global haveRead
    while (haveRead == False):
        while ser.in_waiting:
            data = ser.read()
            haveRead
            haveRead = True
            value = data.decode()
            print(value,end="")


def write(string):
    ser.write((string + '\n').encode())
    print(string)
    print("\n")

def setMotors(l, r):
    '''print(l)
    print(r)
    print("\n")'''
    l = int(l)
    r = int(r)
    le = False
    re = False
    if(l < 0):
        le = True
        le *= -1
    if(r < 0):
        re = True
        re *= -1

    speed = ""

    if(le):
        speed += "-"
    else:
        speed += "+"
    if(l < 10):
        speed += ("00" + str(l))
    elif(l <100):
        speed = speed + "0" + str(l)
    else:
        speed = speed + str(l)
    if(re):
        speed += "-"
    else:
        speed += "+"
    if(r < 10):
        speed += ("00" + str(r))
    elif(r <100):
        speed += ("0" + str(r))
    else:
        speed += str(r)
    #UNCOMMENT THIS WHEN MOVING ROBOT
    #write(speed)


def PID(cxB):
    global last_error
    global integral
    baseSpeed = 70
    error = width//2 - cxB
    kp = 4.0
    ki = 0
    kd = 0
    derivative = error - last_error
    integral += error
    adjustSpeed = (error*kp)+(derivative*kd)+(integral*ki)
    leftmotor = baseSpeed - adjustSpeed
    rightmotor = baseSpeed + adjustSpeed
    if(leftmotor >= 255):
        leftmotor = 255
    if(leftmotor <= -255):
        leftmotor = -255
    if(rightmotor >= 255):
        rightmotor = 255
    if(rightmotor <= -255):
        rightmotor = -255
    setMotors(leftmotor, rightmotor)

def left90():
    write("LG090")
def right90():
    write("RG090")

while(True):
    #Getting the image
    topB = False
    rightB = False
    leftB = False
    ret, frame = vd.read()

    #HSV values for black
    lower_bound_black = np.array([50, 0, 0])
    upper_bound_black = np.array([166, 64, 112])

    #Bluring
    blur = cv.blur(frame, (10,10))
    blur = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

    #Threshholding and finding Contours for black
    blackThresh = cv.inRange(blur, lower_bound_black, upper_bound_black);
    topImg = blackThresh[:top, :]
    rightImg = blackThresh[:, right:]
    leftImg = blackThresh[:, :left]

    (contoursB, h) = cv.findContours(blackThresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    (topCountors, t) = cv.findContours(topImg.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    (rightCountors, r) = cv.findContours(rightImg.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    (leftCountors, r) = cv.findContours(leftImg.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
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
            cv.circle(blackThresh, (cxB, cyB), 7, (0, 0, 255), -1)

    #Getting top center
    cxBT = 0
    cyBT = 0
    if(len(topCountors) > 0):
        areas = [cv.contourArea(c) for c in topCountors]
        max_index = np.argmax(areas)
        M = cv.moments(topCountors[max_index])
        if(M['m00'] != 0):
            cxBT = int(M['m10']/M['m00'])
            cyBT = int(M['m01']/M['m00'])
            cv.circle(framecopy, (cxBT, cyBT), 7, (0, 0, 255), -1)
            cv.circle(topImg, (cxBT, cyBT), 7, (0, 0, 255), -1)
            topB = True;


    #Getting left center
    cxBL = 0
    cyBL = 0
    if(len(leftCountors) > 0):
        areas = [cv.contourArea(c) for c in leftCountors]
        max_index = np.argmax(areas)
        M = cv.moments(leftCountors[max_index])
        if(M['m00'] != 0):
            cxBL = int(M['m10']/M['m00'])
            cyBL = int(M['m01']/M['m00'])
            cv.circle(framecopy, (cxBL, cyBL), 7, (0, 0, 255), -1)
            cv.circle(leftImg, (cxBL, cyBL), 7, (0, 0, 255), -1)
            if(cyBL > 200):
                leftB = True;

    #Getting right center
    cxBR = 0
    cyBR = 0
    if(len(rightCountors) > 0):
        areas = [cv.contourArea(c) for c in rightCountors]
        max_index = np.argmax(areas)
        M = cv.moments(rightCountors[max_index])
        if(M['m00'] != 0):
            cxBR = int(M['m10']/M['m00'])
            cyBR = int(M['m01']/M['m00'])
            cv.circle(framecopy, (width - width//10 + cxBR, cyBR), 7, (0, 0, 255), -1)
            cv.circle(rightImg, (cxBR, cyBR), 7, (0, 0, 255), -1)
            if(cyBR > 200):
                rightB = True;

    #setting up cases and target image
    cset = topB * 4 + leftB * 2 + rightB;
    target_img = np.zeros((height, width), np.uint8)
    print(cset)
    #print(cxBT)
    #print(cyBT)

    #doing the cases
    cxBB = 0
    cyBB = 0
    if(cset >= 4):
        cv.line(target_img, (width//2, height), (cxBT, cyBT), (255, 255, 255), 10)
        cv.circle(target_img, (cxBT, cyBT), 7, (0, 0, 255), -1)
        bottomImg = target_img[height - top*2:, :]
        (bottomCountors, b) = cv.findContours(bottomImg.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if(len(bottomCountors) > 0):
            areas = [cv.contourArea(c) for c in bottomCountors]
            max_index = np.argmax(areas)
            M = cv.moments(bottomCountors[max_index])
            if(M['m00'] != 0):
                cxBB = int(M['m10']/M['m00'])
                cyBB = int(M['m01']/M['m00'])
                cv.circle(framecopy, (cxBB, height - cyBB), 7, (0, 0, 255), -1)
                cv.circle(target_img, (cxBB, height - cyBB), 7, (0, 0, 255), -1)
                PID(cxBB)


        #insert line tracing code here
    elif(cset == 3):
        PID(width//2);
        #insert line tracing code here
    elif(cset == 2):
        target_img = blackThresh.copy()
        left90()
        #insert left turn here
        print("Left Turn")
    elif(cset == 1):
        target_img = blackThresh.copy()
        right90()
        #insert right code here
        print("Right Turn")
    else:
        PID(width//2)
        #insert line tracing code here

    #Displaying image
    cv.imshow("Target Image", target_img);
    cv.imshow("Black Thresh", blackThresh);
    cv.imshow("Frame Copy", framecopy);
    cv.imshow("Top Countours", topImg);
    cv.imshow("Left Countours", leftImg);
    cv.imshow("Right Countours", rightImg);

    pset = cset
    cset = 0

    #UNCOMMENT THIS WHEN RUNNING ROBOT
    read();
    haveRead = False


    if cv.waitKey(1) & 0xFF == ord('q'):
        break

vd.release()
cv.destroyAllWindows
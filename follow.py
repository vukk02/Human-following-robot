import cv2
import numpy as np
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import time
import math
import serial


ser = serial.Serial(
	port = '/dev/ttyS0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1
)
x=0
y=0
def tochar(u,v):
    
    a = u
    b = v
    y = [0] * 11
    y[0] = 0xff
    y[1] = 0x02
    y[10] = 0xAA
    y[9] = b % 10 + 0x30
    b = b // 10
    y[8] = b % 10 + 0x30
    b = b // 10
    y[7] = b % 10 + 0x30
    b = b // 10
    y[6] = b + 0x30
    y[5] = a % 10 + 0x30
    a = a // 10
    y[4] = a % 10 + 0x30
    a = a // 10
    y[3] = a % 10 + 0x30
    a = a // 10
    y[2] = a + 0x30
    
    return y
def swap(a, b):
   
    return b, a
def dis(a, b):
    global x,y

    l=math.sqrt((a-x)**2+(b-y)**2)
    return l


picam=Picamera2()
picam.preview_configuration.main.size=(1200,1200)
picam.preview_configuration.main.format="RGB888"
picam.preview_configuration.align()
picam.configure("preview")
picam.start()

while True:
    frame=picam.capture_array()
    cv2.imshow("picam",frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    lower_Yellow = np.array([25,70,125])
    upper_Yellow = np.array([35,255,255])

    lower_Green = np.array([65, 60, 60])
    upper_Green = np.array([80, 255, 255])

    lower_Red = np.array([0,70,150])
    upper_Red = np.array([15,255,255])

    lower_Blue = np.array([90,60,70])
    upper_Blue = np.array([114,255,255])

    #lower_Purple = np.array([[125, 30, 50]])
    #upper_Purple = np.array([140, 255, 255])

    lower_Purple = np.array([[90, 20, 30]])
    upper_Purple = np.array([120, 255, 255])




    mask1 = cv2.inRange(hsv, lower_Yellow, upper_Yellow)
    mask2 = cv2.inRange(hsv, lower_Green, upper_Green)
    mask3 = cv2.inRange(hsv, lower_Red, upper_Red)
    mask4 = cv2.inRange(hsv, lower_Blue, upper_Blue)
    mask5 = cv2.inRange(hsv, lower_Purple, upper_Purple)
    mask5 = cv2.erode(mask5,None,iterations=2)
    mask5 = cv2.dilate(mask5, None, iterations=2)

    cnts1 = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   # cnts1 = imutils.grab_contours(cnts1)

    cnts2 = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cnts2 = imutils.grab_contours(cnts2)

    cnts3 = cv2.findContours(mask3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cnts3 = imutils.grab_contours(cnts3)

    cnts4 = cv2.findContours(mask4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cnts4 = imutils.grab_contours(cnts4)

    cnts5,_ = cv2.findContours(mask5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cnts5 = imutils.grab_contours(cnts5)
    center1= None
    center2= None
    center3= None
    center4= None
    center5= None
    i=0
    cx=[]
    cy=[]
    for c in cnts5:
    

        area1 = cv2.contourArea(c)
        if area1 > 3000:
            i=i+1
            cv2.drawContours(frame, [c], -1, (0,255,0), 3)

            M = cv2.moments(c)

            cx.append(int(M["m10"]/M["m00"]))
            cy.append(int(M["m01"]/M["m00"]))

    for j in range(i):
        if dis(cx[j],cy[j])<700:
            cx[0],cx[j]=swap(cx[0],cx[j])
            cy[0],cy[j]=swap(cy[0],cy[j])
    for j in range(i):
            
            cx1= int(cx[j]/(1200/10))
            cy1= int(cy[j]/(1200/10))
          
            cv2.circle(frame, (cx[j],cy[j]), 7, (255,255,255), -1)

            cv2.putText(frame, (str(cx1)+str(cy1)), (cx[j]-20, cy[j]-50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            cv2.putText(frame, "i="+str(j+1), (cx[j]-20, cy[j]-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    if i>0:
        x=cx[0]
        y=cy[0]
    else:
        x=0
        y=0
    z = tochar(x,y)
    data = bytes(z)
    ser.write(data)
    ser.flush()


    cv2.imshow("frame",frame)
    if cv2.waitKey(1) == ord("q"):
        break


#cap.release()
cv2.destroyAllWindows()

'''
    Filename        : main.py
    Description     : Object Detection Robot KRSBI Beroda UNAMA
    Created By      : Arjuna Panji Prakarsa
    Date            : 06/06/2021
    Python Version  : 3.6.9
'''

import cv2
import numpy as np
import serial
from time import sleep

# serial motor driver
motor = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
motor.close()

# serial dribble
db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
db.close()

#serial OpenCM
cm = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=1)
cm.close()

# index camera
FRONT_CAM = 2
OMNI_CAM = 0
FONT = cv2.FONT_HERSHEY_SIMPLEX

# initialize
font = cv2.FONT_HERSHEY_SIMPLEX
FRONT_CAM = 0   # front camera
OMNI_CAM = 1   # omni camera

 # create opencv video capture object
FRONT_CAP = cv2.VideoCapture(FRONT_CAM) 
OMNI_CAP = cv2.VideoCapture(OMNI_CAM)

# set frame size
FRONT_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
FRONT_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)

OMNI_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
OMNI_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

def setMotor(ser,dki,dka,bki,bka) :
    dki = dki + (dki * 0.3)
    dka = dka + (dka * 0)
    bki = bki + (bki * 0)
    bka = bka + (bka * 0.3) 
    ser.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))

def dribble(ser,val) :
    if val == 1 :
        ser.write(b"DB ON\n")
    else : 
        ser.write(b"DB OFF\n")

def compass(ser, val) :
    if val == 1:
        ser.write(b"COMPASS ON\n")
    else:
        ser.write(b"COMPASS OFF\n")

def getBallInfo():
    infoFile = open("ballColor.txt","r")
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

def getMagentaInfo():
    infoFile = open("magentaColor.txt","r")
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

def read_from_port(ser,que_output):
    while True:
        try : 
            reading = ser.readline().decode()
            if len(reading) > 0 :
                degree = float(reading[10:-9])
                #print(degree)
                que_output.put(degree)
        except : 
            que_output.put(500)

def lurusArahBola(Ybola):
     # get center of the frame
    _, frame1 = FRONT_CAP.read()
    rows, cols, _ = frame1.shape
    cenX_frame1 = int(cols/2)
    cenY_frame1 = int(rows/2)

    _, frame2 = OMNI_CAP.read()
    rows1, cols1, _ = frame2.shape
    cenX_frame2 = int(cols1/2)
    cenY_frame2 = int(rows1/2)
    
    inner_left = cenX_frame1 - 100
    outer_left = cenX_frame1 - 250 
    inner_right = cenX_frame1 + 100
    outer_right = cenX_frame1 + 250
    inner_top = cenY_frame1 - 100
    outer_top = cenY_frame1 - 150
    inner_bottom = cenY_frame1 + 100
    outer_bottom = cenY_frame1 + 150

    # read ball color
    ballColor = getBallInfo()
    lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
    upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])

    second = 0
    speed = 90
    state = "START"

    # maju
    motor.open()
    setMotor(motor,speed,-speed,speed,-speed)
    motor.close()

    while(True):
        #print(state)
        second += 1
        print(second)
        for i in range(3):
            FRONT_CAP.grab()
            OMNI_CAP.grab()
        ## read frame
        _, frame1 = FRONT_CAP.read()
        _, frame2 = OMNI_CAP.read()
        
        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        if state == "FINISH"  or second > 12:
            motor.open()
            setMotor(motor,0,0,0,0)
            motor.close()
            break
         
        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            if ball_area > 500:
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2   
                print(cenY_ball)
                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                motor.open()
                if cenY_ball < Ybola :
                    setMotor(motor, speed , -speed, speed, -speed)
                    if cenX_ball > inner_right :
                        setMotor(motor, speed + 10,-speed, speed, -speed -10)
                    elif cenX_ball < inner_left :
                        setMotor(motor, speed ,-speed -10,speed +10, -speed)
                else :
                    setMotor(motor,-50,50,-50,50)
                    sleep(0.1)    
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)
                    state = "FINISH"
                motor.close()
                break

        # displays
        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        cv2.imshow("Kamra Atas", frame2)
        cv2.moveWindow("Kamera Atas" ,0,0)
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            motor.open()
            motor.write(b"#M|STP|0\n")
            motor.close()
            db.open()
            db.write(b"DB OFF")
            db.close()
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def main():
    lurusArahBola()

if __name__ == '__main__':
    # execute main program
    main()
'''
    Filename        : main.py
    Description     : Object Detection Robot KRSBI Beroda UNAMA
    Created By      : Arjuna Panji Prakarsa
    Date            : 06/06/2021
    Python Version  : 3.6.9
'''

import cv2
import numpy as np
import datetime
import serial
from time import sleep
import time
import threading
import queue

# serial motor driver
motor = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)

# serial dribble
db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)


#serial OpeCM
sr = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)

sr.close()

# initialize
font = cv2.FONT_HERSHEY_SIMPLEX
date = str(datetime.datetime.now().strftime("%d-%m-%Y %H:%M:%S"))
FRONT_CAM = 0   # front camera
OMNI_CAM = 1   # omni camera

 # create opencv video capture object
FRONT_CAP = cv2.VideoCapture(FRONT_CAM) 
OMNI_CAP = cv2.VideoCapture(OMNI_CAM)


# set frame size
FRONT_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
FRONT_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

OMNI_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
OMNI_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)




class KalmanFilter:
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    
    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return predicted



def getBallInfo():
    infoFile = open("ballColor.txt","r")
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

def getCyanInfo():
    infoFile = open("cyanColor.txt","r")
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

def getGoalInfo():
    infoFile = open("goalColor.txt","r")
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
            
def setMotor(ser,dki,dka,bki,bka) :
    dki = dki + (dki * 0.3)
    dka = dka + (dka * 0)
    bki = bki + (bki * 0)
    bka = bka + (bka * 0.3)
    
        
    ser.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))

def dribbling(ser,val) :
    if val == 1 :
        ser.write(b"DB ON\n")
    else : 
        ser.write(b"DB OFF\n")

def compassOn(ser) :
    ser.write(b"COMPASS ON\n")
 
def compassOff(ser) :
    ser.write(b"COMPASS OFF\n")

def tendangKuat(ser) :
    ser.write(b"TEND1\n")

def tendangOper(ser) :
    ser.write(("TEND2\n").encode('utf-8'))
    
def kananLurusBola():
  
 
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
  

    ballColor = getBallInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)
    
    
    second = 0

    speed = 110

    state = "START"
    
    setMotor(motor,speed,-speed,speed,-speed)
    sleep(0.5)
    
    #Ke Kanan
    setMotor(motor,speed,speed,-speed,-speed)
    
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
        
        # read ball color
        lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
        upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])
        lowerGoal = np.array([goalColor[0],goalColor[1],goalColor[2]])
        upperGoal = np.array([goalColor[3],goalColor[4],goalColor[5]])    

        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        _, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        if state == "FINISH"  or second > 16: 
            setMotor(motor,0,0,0,0)
            break
        
       
            
        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            
            
            if ball_area > 500  :
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2
                predicted = kfObj.Estimate(cenX_ball, cenY_ball)
                
                print(cenX_ball)

                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenX_ball < inner_left - 10:
                
                    setMotor(motor,-50,-50,50,50)
                    sleep(0.1)    
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)
                    state = "FINISH"
                    
                    
                break
            
                    
        
         
      

        # displays

        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        cv2.moveWindow("Kamera Atas" ,0,0)
        cv2.imshow("Kamra Atas", frame2)
        
       
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def SerongKananLurusBola():
  
 
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
  

    ballColor = getBallInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)
    
    
    second = 0

    speed = 200

    state = "START"
    
    
    #Ke Kanan
    setMotor(motor,speed,0,0,-speed)
    
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
        
        # read ball color
        lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
        upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])
        lowerGoal = np.array([goalColor[0],goalColor[1],goalColor[2]])
        upperGoal = np.array([goalColor[3],goalColor[4],goalColor[5]])    

        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        _, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        if state == "FINISH"  or second > 25: 
            setMotor(motor,0,0,0,0)
            break
        
       
            
        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            
            
            if ball_area > 500  :
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2
                predicted = kfObj.Estimate(cenX_ball, cenY_ball)
                
                print(cenX_ball)

                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenX_ball < inner_right :
                
                    setMotor(motor,-50,-50,50,50)
                    sleep(0.1)    
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)
                    state = "FINISH"
                    
                    
                break
            
                    
        
         
      

        # displays

        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        cv2.moveWindow("Kamera Atas" ,0,0)
        cv2.imshow("Kamra Atas", frame2)
        
       
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def LurusArahBola(Ybola):


 
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
  

    ballColor = getBallInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)
    
    
    second = 0

    speed = 90

    state = "START"
    
    setMotor(motor,speed,-speed,speed,-speed)
    
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
        
        # read ball color
        lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
        upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])
        lowerGoal = np.array([goalColor[0],goalColor[1],goalColor[2]])
        upperGoal = np.array([goalColor[3],goalColor[4],goalColor[5]])    

        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        _, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        if state == "FINISH"  or second > 12: 
            setMotor(motor,0,0,0,0)
            break
        
       
            
        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            
            
            if ball_area > 500  :
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2
                predicted = kfObj.Estimate(cenX_ball, cenY_ball)
                
                print(cenY_ball)

                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenY_ball < Ybola :
                    setMotor(motor,speed ,-speed,speed,-speed)
                
                    if cenX_ball > inner_right :
                        setMotor(motor,speed + 10,-speed,speed,-speed -10)
                        
                    elif cenX_ball < inner_left :
                        setMotor(motor,speed ,-speed -10,speed +10,-speed)
                else :
                    setMotor(motor,-50,50,-50,50)
                    sleep(0.1)    
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)
                    state = "FINISH"
                    
                    
                break
        # displays

        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        cv2.moveWindow("Kamera Atas" ,0,0)
        cv2.imshow("Kamra Atas", frame2)
        
       
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def dekatBola():
  
 
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
  

    ballColor = getBallInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)
    
    xAwal = 100
    xAkhir = 380
    yAwal = 105
    yAkhir = 210
        
    Kp = 1.0
    Kd = 5.0
    Ki = 0.0
    error = 0.0
    error_sebelumnya = 0.0
    jumlah_error = 0.0
    selisih_error = 0.0
    P = 0.0
    I = 0.0
    D = 0.0
    PID = 0.0
    
    dari = ""
    
    second = 0

    speed = 60

    state = "START"
   
    dribbling(db,1)
    
    db.flush()
    
    while(True):
        #print(state)
        second += 1
        #print(second)
       
        ## read frame
        _, frame1 = FRONT_CAP.read()
        _, frame2 = OMNI_CAP.read()
        
        # read ball color
        lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
        upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])
        lowerGoal = np.array([goalColor[0],goalColor[1],goalColor[2]])
        upperGoal = np.array([goalColor[3],goalColor[4],goalColor[5]])    

        # convert frame from BGR to HSV
        #hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        #blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        #BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        #_, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        #_, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        #BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        #ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        #goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        if state == "FINISH" : 
            setMotor(motor,0,0,0,0)
            break
        
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"        

       
            
        for ballContour1 in ballContours1:
            ball_area1 = cv2.contourArea(ballContour1)
            
            if ball_area1 > 5:
                (x_ball1, y_ball1, w_ball1, h_ball1) = cv2.boundingRect(ballContour1)
                cv2.putText(frame2, "X: "+str(x_ball1)+" Y: "+str(y_ball1), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball1 = (x_ball1+x_ball1+w_ball1)/2
                cenY_ball1 = (y_ball1+y_ball1+h_ball1)/2
                predicted1 = kfObj.Estimate(cenX_ball1, cenY_ball1)

                # draw actual coordinate from segmentation
                cv2.circle(frame2, (int(cenX_ball1), int(cenY_ball1)), 20, [0,255,0], 2, 8)
                cv2.line(frame2, (int(cenX_ball1), int(cenY_ball1 + 20)), (int(cenX_ball1 + 50), int(cenY_ball1 + 20)), [0,255,0], 2, 8)
                cv2.putText(frame2, "Actual", (int(cenX_ball1 + 50), int(cenY_ball1 + 20)), font, 0.5, [0,255,0], 2)
                
                print(cenX_ball1)
                
                if cenX_ball1 > 225 and dari == "kanan" :
                    dari = ""
                    setMotor(motor,-30,-30,-30,-30)
                    sleep(0.1)    
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)
                elif cenX_ball1 < 270 and   dari == "kiri" :
                    dari = ""
                    setMotor(motor,30,30,30,30)
                    sleep(0.1)    
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)
                
                if cenX_ball1 < 225  :
                    
                    setMotor(motor,30,30,30,30)
                    dari = "kanan"
                    print("PUTAR KANAN")
                
                elif cenX_ball1 > 270 :
                
                    setMotor(motor,-30,-30,-30,-30)
                    dari = "kiri"
                    print("PUTAR KIRI")
                    
                else:
                
                  
                        
                    error =  cenX_frame2 - cenX_ball1 
                    selisih_error = error_sebelumnya - error
                    jumlah_error += (0.001 * error)
                    error_sebelumnya = error
                    P = Kp * error
                    D = Kd * selisih_error
                    I = Ki * jumlah_error
                    PID = P + I  +D
                    PID = PID /2
                    
                  
                    setMotor(motor,50 ,-50,50 ,-50 )
                    
                    print("MAJU PID" + str(PID))
                    
                    
                    
                    if cenX_ball1 > 224 and cenX_ball1 < 244 and cenY_ball1 > 150 and cenY_ball1 < yAkhir:
                        setMotor(motor,0,0,0,0)
                        
                     
                        #dribbling(db,0)
                        
                        state = "FINISH"
                        
               
            
                break
                
      
         
      

        # displays

        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        cv2.moveWindow("Kamera Atas" ,0,0)
        cv2.imshow("Kamra Atas", frame2)
        
       
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def arahRobot():
  
 
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
  

    ballColor = getCyanInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)
    
    xAwal = 100
    xAkhir = 380
    yAwal = 105
    yAkhir = 210
        
    Kp = 1.0
    Kd = 5.0
    Ki = 0.0
    error = 0.0
    error_sebelumnya = 0.0
    jumlah_error = 0.0
    selisih_error = 0.0
    P = 0.0
    I = 0.0
    D = 0.0
    PID = 0.0
    
    dari = ""
    
    second = 0

    speed = 60

    state = "START"
   
    dribbling(db,1)
    
    db.flush()
    
    while(True):
        #print(state)
        second += 1
        #print(second)
       
        ## read frame
        _, frame1 = FRONT_CAP.read()
        _, frame2 = OMNI_CAP.read()
        
        # read ball color
        lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
        upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])
        lowerGoal = np.array([goalColor[0],goalColor[1],goalColor[2]])
        upperGoal = np.array([goalColor[3],goalColor[4],goalColor[5]])    

        # convert frame from BGR to HSV
        #hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        #blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        #BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        #_, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        #_, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        #BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        #ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        #goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        if state == "FINISH" : 
            setMotor(motor,0,0,0,0)
            break
        
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"        

       
            
        for ballContour1 in ballContours1:
            ball_area1 = cv2.contourArea(ballContour1)
            
            if ball_area1 > 5:
                (x_ball1, y_ball1, w_ball1, h_ball1) = cv2.boundingRect(ballContour1)
                cv2.putText(frame2, "X: "+str(x_ball1)+" Y: "+str(y_ball1), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball1 = (x_ball1+x_ball1+w_ball1)/2
                cenY_ball1 = (y_ball1+y_ball1+h_ball1)/2
                predicted1 = kfObj.Estimate(cenX_ball1, cenY_ball1)

                # draw actual coordinate from segmentation
                cv2.circle(frame2, (int(cenX_ball1), int(cenY_ball1)), 20, [0,255,0], 2, 8)
                cv2.line(frame2, (int(cenX_ball1), int(cenY_ball1 + 20)), (int(cenX_ball1 + 50), int(cenY_ball1 + 20)), [0,255,0], 2, 8)
                cv2.putText(frame2, "Actual", (int(cenX_ball1 + 50), int(cenY_ball1 + 20)), font, 0.5, [0,255,0], 2)
                
                print(cenX_ball1)
                
                
                if cenX_ball1 < 120  :
                    setMotor(motor,30,30,30,30)
                elif cenX_ball1 > 350  :
                    setMotor(motor,-30,-30,-30,-30)
                elif cenX_ball1 < 220  :
                    setMotor(motor,35,35,35,35)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kanan"
                    print("PUTAR KANAN")
                
                elif cenX_ball1 > 255 :
                
                    setMotor(motor,-35,-35,-35,-35)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kiri"
                    print("PUTAR KIRI")
                    
               
                break
                
      
         
      

        # displays

        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        cv2.moveWindow("Kamera Atas" ,0,0)
        cv2.imshow("Kamra Atas", frame2)
        
       
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break
            
def arahRobotKameraDpn():
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
  

    ballColor = getCyanInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)
    
    xAwal = 100
    xAkhir = 380
    yAwal = 105
    yAkhir = 210
        
    Kp = 1.0
    Kd = 5.0
    Ki = 0.0
    error = 0.0
    error_sebelumnya = 0.0
    jumlah_error = 0.0
    selisih_error = 0.0
    P = 0.0
    I = 0.0
    D = 0.0
    PID = 0.0
    
    dari = ""
    
    second = 0

    speed = 60

    state = "START"
    
    pas = 0
   
    dribbling(db,1)
    
    db.flush()
    
    while(True):
        #print(state)
        second += 1
        #print(second)
        for i in range(7):
            FRONT_CAP.grab()
            OMNI_CAP.grab()
        ## read frame
        _, frame1 = FRONT_CAP.read()
        _, frame2 = OMNI_CAP.read()
        
        # read ball color
        lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
        upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])
        lowerGoal = np.array([goalColor[0],goalColor[1],goalColor[2]])
        upperGoal = np.array([goalColor[3],goalColor[4],goalColor[5]])    

        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        #hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        #blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        #BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        #_, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        #_, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        #BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        #ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        #goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
       
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat"  :
                print("DAPAT BOLA")
                state = "FINISH"        
        
      
        
        
        ada = 0
        pas =0
       
        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            
            
            if ball_area > 300  :
                ada = 1
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2
                predicted = kfObj.Estimate(cenX_ball, cenY_ball)
                
                print(cenX_ball)

                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                
                
                if cenX_ball < 150  :
                    setMotor(motor,-30,-30,-30,-30)
                    
                elif cenX_ball > 420  :
                    setMotor(motor,30,30,30,30)
                    
                elif cenX_ball < 290  :
                    setMotor(motor,-35,-35,-35,-35)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kanan"
                    print("PUTAR KANAN")
                
                elif cenX_ball > 320 :
                
                    setMotor(motor,35,35,35,35)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kiri"
                    print("PUTAR KIRI")
                else :
                    pas = 1
                break
        
        if state == "FINISH"  and pas == 1: 
            setMotor(motor,0,0,0,0)
            break        
            
        if ada == 0 :
            if dari == "kanan" :
                setMotor(motor,35,35,35,35)
                sleep(0.1)
                setMotor(motor,20,20,20,20)
            else :
                setMotor(motor,-35,-35,-35,-35)
                sleep(0.1)
                setMotor(motor,-20,-20,-20,-20)

        # displays

        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        cv2.moveWindow("Kamera Atas" ,0,0)
        cv2.imshow("Kamra Atas", frame2)
        
       
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def terimaBola():
  
 
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
  

    ballColor = getBallInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)
    
    xAwal = 100
    xAkhir = 380
    yAwal = 105
    yAkhir = 210
        
    Kp = 1.0
    Kd = 5.0
    Ki = 0.0
    error = 0.0
    error_sebelumnya = 0.0
    jumlah_error = 0.0
    selisih_error = 0.0
    P = 0.0
    I = 0.0
    D = 0.0
    PID = 0.0
    
    dari = ""
    
    second = 0

    speed = 60

    state = "START"
   
    dribbling(db,1)
    
    db.flush()
    
    while(True):
        #print(state)
        second += 1
        #print(second)
       
        ## read frame
        _, frame1 = FRONT_CAP.read()
        _, frame2 = OMNI_CAP.read()
        
        # read ball color
        lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
        upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])
        lowerGoal = np.array([goalColor[0],goalColor[1],goalColor[2]])
        upperGoal = np.array([goalColor[3],goalColor[4],goalColor[5]])    

        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        #hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        #blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        #BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        #_, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        #_, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        #BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        #ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        #goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        if state == "FINISH" : 
            setMotor(motor,0,0,0,0)
            break
        
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"        

       
            
         
        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            
            
            if ball_area > 500  :
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2
                predicted = kfObj.Estimate(cenX_ball, cenY_ball)

                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if state == "START" :
                   
                    
                    if cenX_ball > outer_right :
                        dari = "kanan"
                        setMotor(motor,80,80,-80,-80)
                    
                    elif cenX_ball < outer_left :
                        dari = "kiri"
                        setMotor(motor,-80,-80,80,80)
                        
                    elif cenX_ball > inner_right :
                        dari = "kanan"
                        setMotor(motor,50,50,-50,-50)
               
                    elif cenX_ball < inner_left :
                        dari = "kiri"
                        setMotor(motor,-50,-50,50,50)
                    
                    else : 
                        if dari == "kanan" :
                            setMotor(motor,-50,-50,50,50)
                            sleep(0.1)    
                            setMotor(motor,0,0,0,0)
                            sleep(0.1)
                            dari = ""
                        if dari == "kiri" :
                            setMotor(motor,50,50,-50,-50)
                            sleep(0.1)    
                            setMotor(motor,0,0,0,0)
                            sleep(0.1)
                            dari = ""
                        else :
                            setMotor(motor,0,0,0,0)
               
            
                break
                
      
         
      

        # displays

        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        cv2.moveWindow("Kamera Atas" ,0,0)
        cv2.imshow("Kamra Atas", frame2)
        
       
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def putarDerajat(derajat_tujuan, dribble) :
    speed = 70
    state = "START"
    count = 1
    
    
    db.close()
    db.open()
    db.flush()
    
    
    maju = 0
    
    clb = 0
        
    while(True) :
        
        compassOn(db)
    
        
        if dribble == 1 :
            dribbling(db,1)
        else :
            dribbling(db,0)
            
        if state == "FINISH" :
            compassOff(db)
            break
            
            
        reading = db.readline().decode('utf-8','ignore')
        
        
        if len(reading) > 0 :
            
            #print(reading)
            head = reading[0:7]
            if  head == "Heading" :
                degree = float(reading[10:-9])
                
                print(degree)
                
                if degree - derajat_tujuan < -180 :
                    selisih = -360 + derajat_tujuan - degree
                elif degree - derajat_tujuan >= -180 and  degree - derajat_tujuan <= 180 :
                    selisih =  derajat_tujuan - degree
                elif degree - derajat_tujuan >  180 :
                    selisih = 360 + derajat_tujuan - degree   
                
                selisihabs = abs(selisih)
                
                speed = selisih
                
                if speed > 0 :
                    if speed > 50 :
                        speed = 50
                    if speed < 30 :
                        speed = 30
                else :
                    if speed < -50 :
                        speed = -50
                    if speed > -30 :
                        speed = -30

                print(speed)
                rentang = 1
                
                if selisihabs < rentang :
                    if speed > 0 :
                        speed = -35
                    else :
                        speed = 35
                        
                    setMotor(motor,speed,speed,speed,speed)
                    
                    sleep(0.1)
                    
                    setMotor(motor,0,0,0,0)
                 
                    if clb > 1 : 
                        state = "FINISH"
                        
                    clb += 1
                
                elif selisihabs < 15 :
                    if speed > 0 :
                        speed = 40
                    else :
                        speed = -40
                        
                    setMotor(motor,speed,speed,speed,speed)
                    sleep(0.1)
                    
                    setMotor(motor,0,0,0,0)
                   
                   
                elif selisihabs < 70 :
                    if speed > 0 :
                        speed = 40
                    else :
                        speed = -40
                        
                    setMotor(motor,speed,speed,speed,speed)
                   
                    
                else :
                    setMotor(motor,speed,speed,speed,speed)
                        
        
def cariHome():
  
 
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
  

    ballColor = getBallInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)
    
    xAwal = 100
    xAkhir = 380
    yAwal = 105
    yAkhir = 210
        
    Kp = 1.0
    Kd = 5.0
    Ki = 0.0
    error = 0.0
    error_sebelumnya = 0.0
    jumlah_error = 0.0
    selisih_error = 0.0
    P = 0.0
    I = 0.0
    D = 0.0
    PID = 0.0
    
    dari = ""
    
    second = 0

    speed = 60

    state = "START"
  
    
    while(True):
        #print(state)
        second += 1
        #print(second)
       
        ## read frame
        _, frame1 = FRONT_CAP.read()
        _, frame2 = OMNI_CAP.read()
        
        # read ball color
        lowerBall = np.array([ballColor[0],ballColor[1],ballColor[2]])
        upperBall = np.array([ballColor[3],ballColor[4],ballColor[5]])
        lowerGoal = np.array([goalColor[0],goalColor[1],goalColor[2]])
        upperGoal = np.array([goalColor[3],goalColor[4],goalColor[5]])    

        # convert frame from BGR to HSV
        #hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        #blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        #BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        GOAL_MASK = cv2.inRange(blur1, lowerGoal, upperGoal)
        
        #BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        #_, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        _, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        #_, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        #BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        #BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        #ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        #ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        #goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        homeContours = cv2.findContours(GOAL_MORPH,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        homeContours = homeContours[0] if len(homeContours) == 2 else homeContours[1]
        
        for c in homeContours :
                cv2. drawContours(frame2,[c],0,(0,0,0),2)
                rot_rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rot_rect)
                box = np.int0(box)
                cv2.drawContours(frame2,[box],0,(0,0,0),2)
            
        
            
                
                
      
         
      

        # displays

        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        cv2.moveWindow("Kamera Atas" ,0,0)
        cv2.imshow("Kamra Atas", frame2)
        
       
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break



if __name__ == "__main__":
    # execute main program
    compassOff(db)
    sleep(0.5)
    compassOff(db)
    
    mode = 2
    
    if mode == 1 :
        sr.open()
        sr.write(b"#450512")
        putarDerajat(45,0)
        
        kananLurusBola()
         
        LurusArahBola(240)
        
        dekatBola()
        putarDerajat(20,1)
      
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
        tendangOper(db)
        sleep(0.5)
        tendangOper(db)
        
        sleep(2)
        
        putarDerajat(30,0)
        setMotor(motor,-120,-120,120,120)
        sleep(2)
        
        setMotor(motor,50,50,-50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        
        terimaBola()
            
    if mode == 2 :
        #dummy 3 dan 8
        sr.open()
        sr.write(b"#450512")
        putarDerajat(53,0)
        
        
      
        SerongKananLurusBola()
        
        LurusArahBola(200)
        
        dekatBola()
        putarDerajat(10,1)
        
        arahRobotKameraDpn()
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
     
        tendangOper(db)
        sleep(2.5)
     
    
        
        putarDerajat(45,0)
        
        
        setMotor(motor,-120,-120,120,120)
        sleep(2)
        
        setMotor(motor,50,50,-50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        putarDerajat(80,1)
        
        #terimaBola()
        arahRobotKameraDpn()
        
         
        putarDerajat(30,1)
        
        arahRobotKameraDpn()
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(2.5)
    
        
        
        putarDerajat(55,0)
         
        setMotor(motor,-90,90,-90,90)
        sleep(3)
        setMotor(motor,50,-50,50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        
    if mode == 3 :
        sr.open()
        sr.write(b"#450512")
      
        #dekatBola()
        arahRobotKameraDpn()
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
      
        tendangOper(db)
        sleep(2.5)
        
    if mode == 4 :
    
        sr.open()
        sr.write(b"#450512")
        putarDerajat(75,0)
        
        
        setMotor(motor,120,-120,120,-120)
        sleep(1.5)
        
        setMotor(motor,0,0,0,0)
      
        terimaBola()
        
        
        putarDerajat(45,1)
      
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(2.5)
       
        
        setMotor(motor,-120,-120,120,120)
        sleep(1.1)
        
        setMotor(motor,120,-120,120,-120)
        sleep(0.6)
        
        setMotor(motor,0,0,0,0)
        sleep(1)  
        
        putarDerajat(70,1)
      
        terimaBola()
        
        
        putarDerajat(10,1)
        
      
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(3)
        
        
        putarDerajat(55,0)
         
        setMotor(motor,-90,90,-90,90)
        sleep(2.2)
        setMotor(motor,50,-50,50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)  
        
    if mode == 5 :
    
        sr.open()
        sr.write(b"#450512")
        putarDerajat(53,0)
        
        
      
        SerongKananLurusBola()
        
        LurusArahBola(200)
        
        dekatBola()
        putarDerajat(10,1)
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
     
        tendangKuat(db)
        sleep(2.5)
     
    
        
        putarDerajat(45,0)
        
        
        setMotor(motor,-120,-120,120,120)
        sleep(2)
        
        setMotor(motor,50,50,-50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        setMotor(motor,-90,90,-90,90)
        sleep(1)
        setMotor(motor,50,-50,50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        putarDerajat(70,1)
        
        terimaBola()
        
         
        putarDerajat(55,1)
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(2.5)
    
        
             
        setMotor(motor,-90,90,-90,90)
        sleep(2.2)
        setMotor(motor,50,-50,50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
            
       
    if mode == 6 :
    
        sr.open()
        sr.write(b"#450512")
        putarDerajat(75,0)
        
        
        setMotor(motor,120,-120,120,-120)
        sleep(1.5)
        
        setMotor(motor,0,0,0,0)
      
        terimaBola()
        
        
        putarDerajat(30,1)
      
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(2.5)
       
        
        putarDerajat(45,0)
        
        
        setMotor(motor,-120,-120,120,120)
        sleep(2)
        
        setMotor(motor,50,50,-50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        putarDerajat(80,1)
        
        terimaBola()
        
         
        putarDerajat(30,1)
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(2.5)
    
        
        
        putarDerajat(30,0)
         
        setMotor(motor,-90,90,-90,90)
        sleep(1)
        setMotor(motor,50,-50,50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
    if mode == 7 :
    
    
        sr.open()
        sr.write(b"#450512")
        putarDerajat(53,0)
        
        
      
        SerongKananLurusBola()
        
        LurusArahBola(200)
        
        dekatBola()
        putarDerajat(10,1)
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
     
        tendangKuat(db)
        sleep(2.5)
     
    
        
        putarDerajat(45,0)
        
        
        setMotor(motor,-120,-120,120,120)
        sleep(2)
        
        setMotor(motor,50,50,-50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        putarDerajat(80,1)
        
        terimaBola()
        
         
        putarDerajat(30,1)
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(2.5)
    
        
        
        putarDerajat(55,0)
         
        setMotor(motor,0,90,-90,0)
        sleep(2.1)
        setMotor(motor,0,-50,50,0)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
          
        setMotor(motor,-90,90,-90,90)
        sleep(1.8)
        setMotor(motor,50,-50,50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        setMotor(motor,-90,0,0,90)
        sleep(1.5)
        setMotor(motor,50,0,0,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
    if mode == 8 :
        #dummy 2 dan 8
        sr.open()
        sr.write(b"#450512")
        putarDerajat(75,0)
        
        
        setMotor(motor,120,-120,120,-120)
        sleep(1.5)
        
        setMotor(motor,0,0,0,0)
      
        terimaBola()
        
        
        putarDerajat(45,1)
      
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(2.7)
       
        
        putarDerajat(45,0)
        
        
        setMotor(motor,0,-120,120,0)
        sleep(1)
        
        setMotor(motor,-50,50,-50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        
        setMotor(motor,-120,-120,120,120)
        sleep(0.6)
        
        setMotor(motor,50,50,-50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        putarDerajat(80,1)
        
        terimaBola()
        
         
        putarDerajat(10,1)
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(2.5)
    
        
        
        putarDerajat(55,0)
        
        
        setMotor(motor,90,90,-90,-90)
        sleep(1)
        setMotor(motor,50,-50,50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        
        setMotor(motor,-90,90,-90,90)
        sleep(1.8)
        setMotor(motor,50,-50,50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
        setMotor(motor,-90,0,0,90)
        sleep(1.5)
        setMotor(motor,50,0,0,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
    if mode == 9 :
        # dummy 1 dan 5
        sr.open()
        sr.write(b"#450512")
        putarDerajat(50,0)
        
        kananLurusBola()
         
        LurusArahBola(240)
        
        dekatBola()
        
        putarDerajat(10,1)
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
     
        tendangKuat(db)
        sleep(2.5)
     
    
        
        putarDerajat(45,0)
        
        
        setMotor(motor,-120,-120,120,120)
        sleep(1)
        
        setMotor(motor,50,50,-50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
        
       
        putarDerajat(55,1)
        
        terimaBola()
        
         
        putarDerajat(30,1)
        
        dribbling(db,0)
        sleep(1)
        dribbling(db,0)
        sleep(0.5)
        dribbling(db,0)
        sleep(0.5)
      
        tendangKuat(db)
        sleep(2.5)
    
        
        
        putarDerajat(55,0)
        
        setMotor(motor,-120,-120,120,120)
        sleep(1)
        
        setMotor(motor,50,50,-50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
         
        setMotor(motor,-90,90,-90,90)
        sleep(2.2)
        setMotor(motor,50,-50,50,-50)
        sleep(0.1)
        setMotor(motor,0,0,0,0)
        sleep(0.1)
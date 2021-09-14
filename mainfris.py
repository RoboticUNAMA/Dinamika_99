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
import requests

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

ip_server = "192.168.10.244"


dummy1 = "" 
dummy2 =""  
kiper = "" 
mode = "" 
gameStatus  = ""

state = ""


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

def getYellowInfo():
    infoFile = open("yellowColor.txt","r")
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

def setJalan(ser,dki,dka,bki,bka,dly) :
    setMotor(ser,dki,dka,bki,bka)
    sleep(dly)
    if dki > 0 :
        invdki = 50
    elif dki < 0 :
        invdki = -50
    else :
        invdki = 0
    
    if dka > 0 :
        invdka = 50
    elif dka < 0 :
        invdka = -50
    else :
        invdka= 0
        
    if bki > 0 :
        invbki = 50
    elif bki < 0 :
        invbki = -50
    else :
        invbki= 0
        
        
    if bka > 0 :
        invbka = 50
    elif bka < 0 :
        invbka = -50
    else :
        invbka = 0
        
    setMotor(ser,invdki,invdka,invbki,invbka)
    sleep(0.1)
    setMotor(ser,0,0,0,0)
    sleep(0.1)
      
def getGameInfo():
    dummy1 = requests.post("http://"+ip_server+"/robot/getdummy1.php")
    dummy2 = requests.post("http://"+ip_server+"/robot/getdummy2.php")
    kiper = requests.post("http://"+ip_server+"/robot/getkiper.php")
    mode = requests.post("http://"+ip_server+"/robot/getmode.php")
    gameStatus = requests.post("http://"+ip_server+"/robot/getgame.php")
    return dummy1.text.strip(), dummy2.text.strip(), kiper.text.strip(), mode.text.strip(), gameStatus.text.strip()
 
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
        
    while True : 
       
        reading = ser.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:2]
            if  head == "OK"  :
                print("TENDANG")
                break
                
        ser.reset_input_buffer()
        sleep(0.1)
        
def tendangKuatNian(ser) :
    ser.write(b"TEND1\n")
    
    while True : 
       
        reading = ser.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:2]
            if  head == "OK"  :
                print("TENDANG")
                break
                
        ser.reset_input_buffer()
        sleep(0.1)

def tendangOper(ser) :
    ser.write(("TEND2\n").encode('utf-8'))
    
    while True : 
       
        reading = ser.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:2]
            if  head == "OK"  :
                print("TENDANG")
                break
                
        ser.reset_input_buffer()
        sleep(0.1)
    
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
    setMotor(motor,speed+20,speed+20,-speed-50,-speed -50)
    
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
        
        #retry()
        
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
                
                print(cenX_ball)
        
                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
         
                
                setMotor(motor,speed+20,speed+20,-speed-50,-speed -50)
                sleep(0.5)    
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

    speed = 220

    state = "START"
    
    
    #Ke Kanan
    setMotor(motor,speed,0,0,-speed)
    
    while(True):
        #print(state)
        
        if speed <= 240 :
            speed += 10
        
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
        
        #retry()
        
        if state == "FINISH"  or second > 14: 
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
                
                if cenX_ball < inner_right + 60 :
                
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
        
        #retry()
        
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
    
    global state
    

    state = "START"
   
    dribbling(db,1)
    
    db.reset_input_buffer()
    
    while(True):
        #print(state)
        second += 1
        #print(second)
      
        ## read frame
        #_, frame1 = FRONT_CAP.read()
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
        
        #retry()
        
        if state == "FINISH" : 
            setMotor(motor,0,0,0,0)
            break
        
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"        

        db.reset_input_buffer()
            
        for ballContour1 in ballContours1:
            ball_area1 = cv2.contourArea(ballContour1)
            
            if ball_area1 > 6:
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
                
                if cenX_ball1 > 220 and dari == "kanan" :
                    dari = ""
                    setMotor(motor,-30,-30,-30,-30)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)

              
                elif cenX_ball1 < 250 and dari == "kiri" :
                    dari = ""
                    setMotor(motor,30,30,30,30)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)
                    
          
                if cenX_ball1 < 220  :
                    
                    setMotor(motor,28,28,28,28)
                    dari = "kanan"
                    print("PUTAR KANAN")
                
                elif cenX_ball1 > 260 :
                
                    setMotor(motor,-28,-28,-28,-28)
                    dari = "kiri"
                    print("PUTAR KIRI")
                    
                else:
                
                    # error =  cenX_frame2 - 250 
                    # selisih_error = error_sebelumnya - error
                    # jumlah_error += (0.001 * error)
                    # error_sebelumnya = error
                    # P = Kp * error
                    # D = Kd * selisih_error
                    # I = Ki * jumlah_error
                    # PID = P + I  +D
                    # PID = PID /5
                    
                  
                    setMotor(motor,55  ,-50 ,50  ,-55  )
                   
                    
                    print("MAJU PID" + str(PID))
                    
                    
                    
                    # if cenX_ball1 > 224 and cenX_ball1 < 244 and cenY_ball1 > 151 and cenY_ball1 < yAkhir:
                        # setMotor(motor,0,0,0,0)
                        
                     
                        #dribbling(db,0)
                        
                        # state = "FINISH"
                        
               
            
                break
                
      
         
      

        # displays

        ## uncomment this to show center area of the frame 1
        #cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        #cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        #cv2.imshow("Kamera Depan", frame1)
        #cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        #cv2.moveWindow("Kamera Atas" ,0,0)
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
    
    db.reset_input_buffer()
    
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
        
        #retry()
        
     
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"        

        db.reset_input_buffer()
        
        ada = 0
        pas = 0
            
        for ballContour1 in ballContours1:
            ball_area1 = cv2.contourArea(ballContour1)
            
            if ball_area1 > 5:
                ada = 1
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
                
                
                if cenX_ball1 < 150  :
                    setMotor(motor,30,30,30,30)
                elif cenX_ball1 > 320  :
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
                else :
                   
                    pas = 1
                break
                
        if state == "FINISH"  and pas == 1: 
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

def arahRobot2():
  
 
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
    
    db.reset_input_buffer()
    
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
        
        #retry()
        
        if state == "FINISH" : 
            setMotor(motor,0,0,0,0)
            break
        
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"        

        db.reset_input_buffer()
            
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
                
                
                if cenX_ball1 < 150  :
                    setMotor(motor,30,30,30,30)
                elif cenX_ball1 > 320  :
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
                else :
                    setStatus("1","READY")
               
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
    
    db.reset_input_buffer()
    
    while(True):
        #print(state)
        second += 1
        print(second)
       
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
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        #_, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        #goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
       
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat"  :
                print("DAPAT BOLA")
                state = "FINISH"        
        
        db.reset_input_buffer()
        
        print(state)
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
                   
                    
                elif cenX_ball < 310  :
                    setMotor(motor,-40,-40,-40,-40)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kanan"
                    print("PUTAR KANAN")
                
                elif cenX_ball > 330 :
                
                    setMotor(motor,40,40,40,40)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kiri"
                    print("PUTAR KIRI")
                else :
                    pas = 1
                break
        
        #retry()
        
        if state == "FINISH"  and pas == 1: 
            setMotor(motor,0,0,0,0)
            break        
            
        if ada == 0 :
        
        
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
                    
                    
                    if cenX_ball1 < 150  :
                        setMotor(motor,30,30,30,30)
                    elif cenX_ball1 > 320  :
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
                    
      
            # if dari == "kanan" :
                # setMotor(motor,30,30,30,30)
             
            # else :
                # setMotor(motor,-30,-30,-30,-30)

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

def arahRobotKameraDpn2():
  
 
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
    
    db.reset_input_buffer()
   
    
    
  
    
    
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
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        #_, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        #goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        dribbling(db,1)
        
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat"  :
                print("DAPAT BOLA")
                state = "FINISH"        
        
        db.reset_input_buffer() 
        
        print(state)
    
     
        ada = 0
        pas =0
       
        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            
            
            reading = db.readline().decode('utf-8','ignore')
            if len(reading) > 0 :
                head = reading[0:5]
                if  head == "Dapat"  :
                    print("DAPAT BOLA")
                    state = "FINISH"        
            
            db.reset_input_buffer()
            
            print(state)
            
                  
            if state == "FINISH"  : 
               setMotor(motor,0,0,0,0)
               break    
               
            
            
            
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
                
                setStatus("1","RUNNING")
                
                if cenX_ball < 150  :
                    setMotor(motor,-50,-50,-50,-50)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                   
                    
                elif cenX_ball > 420  :
                    setMotor(motor,50,50,50,50)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                  
                    
                elif cenX_ball < 310  :
                    setMotor(motor,-40,-40,-40,-40)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kanan"
                    print("PUTAR KANAN")
                
                elif cenX_ball > 330 :
                
                    setMotor(motor,40,40,40,40)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kiri"
                    print("PUTAR KIRI")
                else :
                    pas = 1
                    setStatus("1","READY")
                    
                break
        
        #retry()
        
        if state == "FINISH"  : 
           setMotor(motor,0,0,0,0)
           break    
           
            
        if ada == 0 :
            
            # if dari == "kanan" :
                # setMotor(motor,30,30,30,30)
             
            # else :
                # setMotor(motor,-30,-30,-30,-30)
            
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
                    
                    
                    if cenX_ball1 < 150  :
                        setMotor(motor,30,30,30,30)
                    elif cenX_ball1 > 320  :
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
        #cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        #cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
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


def arahRobotKameraDpnCorner():
  
 
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
    
    db.reset_input_buffer()
   
    
    
  
    
    
    while(True):
        #print(state)
        second += 1
        
        print(second)
        
        if second > 12 :
            state = "FINISH"
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
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
        
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
        #_, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
        
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, ballColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContous = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        #goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        dribbling(db,1)
        
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat"  :
                print("DAPAT BOLA")
                state = "FINISH"        
        
        db.reset_input_buffer() 
        
        print(state)
    
     
        ada = 0
        pas =0
       
        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            
            
            reading = db.readline().decode('utf-8','ignore')
            if len(reading) > 0 :
                head = reading[0:5]
                if  head == "Dapat"  :
                    print("DAPAT BOLA")
                    state = "FINISH"        
            
            db.reset_input_buffer()
            
            print(state)
            
                  
            if state == "FINISH"  :
               setMotor(motor,0,0,0,0)
               break    
               
            
            
            
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
                
                setStatus("1","RUNNING")
                
                if cenX_ball < 150  :
                    setMotor(motor,-50,-50,-50,-50)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                   
                    
                elif cenX_ball > 420  :
                    setMotor(motor,50,50,50,50)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                  
                    
                elif cenX_ball < 310  :
                    setMotor(motor,-40,-40,-40,-40)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kanan"
                    print("PUTAR KANAN")
                
                elif cenX_ball > 330 :
                
                    setMotor(motor,40,40,40,40)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kiri"
                    print("PUTAR KIRI")
                else :
                    pas = 1
                    setStatus("1","READY")
                    
                break
        
        #retry()
        
        if state == "FINISH"  : 
           setMotor(motor,0,0,0,0)
           break    
           
            
        if ada == 0 :
            
            # if dari == "kanan" :
                # setMotor(motor,30,30,30,30)
             
            # else :
                # setMotor(motor,-30,-30,-30,-30)
            
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
                    
                    
                    if cenX_ball1 < 150  :
                        setMotor(motor,30,30,30,30)
                    elif cenX_ball1 > 320  :
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
        #cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        #cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
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
def arahRobotKompas2(derajat_tujuan):
  
 
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
    
    selisihabs =0 
    
    
    dribbling(db,1)
    
    db.reset_input_buffer()
   
    
    compassOn(db)
  
    
    
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
                        speed = 30
                    if speed < 30 :
                        speed = 30
                else :
                    if speed < -50 :
                        speed = -30
                    if speed > -30 :
                        speed = -30
                
              
        db.reset_input_buffer() 
        
        retry()
        
        print(state)
        
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
                    
                elif cenX_ball < 310  :
                    setMotor(motor,-35,-35,-35,-35)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kanan"
                    print("PUTAR KANAN")
                
                elif cenX_ball > 340 :
                
                    setMotor(motor,35,35,35,35)
                    sleep(0.1)
                    setMotor(motor,0,0,0,0)
                    dari = "kiri"
                    print("PUTAR KIRI")
                else :
                    pas = 1
                    setStatus("1","READY")
                break
        
        if state == "FINISH"  : 
            setMotor(motor,0,0,0,0)
            break        
            
        if ada == 0 :
            if selisihabs > 40 :
                setMotor(motor,speed,speed,speed,speed)
                
            else :
                if dari == "kanan" :
                    setMotor(motor,30,30,30,30)
             
                else :
                    setMotor(motor,-30,-30,-30,-30)
           
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

def arahGawangKameraAtas(tujuan):
    #tujuan kanankiper / kirikiper
  
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
  

    ballColor = getYellowInfo()
    goalColor = getYellowInfo()
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
    
    global state
    

    state = "START"
   
    dribbling(db,1)
    
    db.reset_input_buffer()
    
    while(True):
        #print(state)
        second += 1
        #print(second)
      
        ## read frame
        #_, frame1 = FRONT_CAP.read()
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
        
        #retry()
        
        #reading = db.readline().decode('utf-8','ignore')
        
        if state == "FINISH" : 
            setMotor(motor,0,0,0,0)
            break
        
       
            
        for ballContour1 in ballContours1:
            ball_area1 = cv2.contourArea(ballContour1)
            
            if ball_area1 > 6 :
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
                
                if cenY_ball1 < 90 :
                
                    target = 0
                    
                    if tujuan == "kirikiper" :
                        target = -40
                    elif tujuan == "kanankiper" :
                        target = 40
                    
                    if cenX_ball1 > 210 + target and dari == "kanan" :
                        dari = ""
                        setMotor(motor,-30,-30,-30,-30)
                        sleep(0.1)
                        setMotor(motor,0,0,0,0)
                        sleep(0.1)

                  
                    elif cenX_ball1 < 250  + target and dari == "kiri" :
                        dari = ""
                        setMotor(motor,30,30,30,30)
                        sleep(0.1)
                        setMotor(motor,0,0,0,0)
                        sleep(0.1)
                        
              
                    if cenX_ball1 < 210 + target  :
                        
                        setMotor(motor,30,30,30,30)
                        dari = "kanan"
                        print("PUTAR KANAN")
                    
                    elif cenX_ball1 > 250  + target:
                    
                        setMotor(motor,-30,-30,-30,-30)
                        dari = "kiri"
                        print("PUTAR KIRI")
                        
                    else:
                        state = "FINISH"
                        
                        
                      
                        
                        
                        break


        # displays

        ## uncomment this to show center area of the frame 1
        #cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        #cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
        #cv2.imshow("Kamera Depan", frame1)
        #cv2.moveWindow("Kamera Depan" ,20,20)
        
        
        #cv2.moveWindow("Kamera Atas" ,0,0)
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
    
    db.reset_input_buffer()
    
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
        
        retry()
        
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
    db.reset_input_buffer()
    
    
    maju = 0
    
    clb = 0
        
    while(True) :
        
        compassOn(db)
        
        #retry()
        
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
                rentang = 5
                
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
 
def putarDerajatBasing(derajat_tujuan, dribble) :
    speed = 70
    state = "START"
    count = 1
    
    
    db.close()
    db.open()
    db.reset_input_buffer()
    
    
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
                rentang = 10
                
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

def tungguStatus(idr,status) :
    while(True) :
        f = requests.get("http://192.168.10.244/robot/getstatus.php?id=" + idr)
        if f.text.strip() == status :
            break
            
def setStatus(idr,status) :
    f = requests.get("http://192.168.10.244/robot/setstatus.php?id=" + idr + "&status=" + status)

def setGame(status):
    requests.post("http://"+ip_server+"/robot/setgame.php?"+"status="+str(status))
    
def retry() :

    global dummy1, dummy2, kiper, mode, gameStatus
    global state
    
    
    dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
    #print(state)
    
    if gameStatus == "RETRY":
        setStatus("1","IDLE")
        state = "FINISH"
        

def backHome(posisi) :

    global gameStatus
    
    
    print(gameStatus)
    
    
    if gameStatus == "RETRY":
        cv2.destroyAllWindows()
        if posisi ==  "Bola Tengah" :
        
            putarDerajat(40,0)
            
            setMotor(motor,-110,110,-110,110)
            sleep(2.8)
            setMotor(motor,50,-50,50,-50)
            sleep(0.1)
            setMotor(motor,0,0,0,0)
            sleep(0.1)
            
            
            setMotor(motor,-110,-110,110,110)
            sleep(1.8)
            setMotor(motor,50,50,-50,-50)
            sleep(0.1)
            setMotor(motor,0,0,0,0)
            sleep(0.1)
            
            return True
        elif posisi ==  "Tengah Lapang" :
        
            putarDerajat(40,0)
            
            setMotor(motor,-110,110,-110,110)
            sleep(2.8)
            setMotor(motor,50,-50,50,-50)
            sleep(0.1)
            setMotor(motor,0,0,0,0)
            sleep(0.1)
            
            
            setMotor(motor,-110,-110,110,110)
            sleep(1)
            setMotor(motor,50,50,-50,-50)
            sleep(0.1)
            setMotor(motor,0,0,0,0)
            sleep(0.1)
            
            return True
        else : 
            return False
    else :
        return False

if __name__ == "__main__":
    # execute main program
    compassOff(db)
    sleep(0.5)
    compassOff(db)
    mode = "0"
    #
    sr.open()
    
    mode = input("Mode = ")
  
    if mode == "run" : 
        while True : 
        
            _, frame2 = OMNI_CAP.read()
            _, frame1 = FRONT_CAP.read()
            
            dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
            
            
            if mode == "CORNER":
            
                while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#530512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        speed = 110
                        
                        setJalan(motor,speed,-speed,speed,-speed,0.7)
                        
                        setJalan(motor,speed+20,speed+20,-speed-50,-speed -50,2.3)
                        
                        setJalan(motor,speed,-speed,speed,-speed,2.2)
                       
            
                        
                        #LurusArahBola(200)
                        
                        dekatBola()
                        
                        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        if backHome("Bola Tengah") == True:
                            break
                        
                        #putarDerajat(20,1)
                        
                        #setJalan(motor,-50,-50,-50,-50,0.6)
                     
           
                        arahRobotKameraDpn()
                        
                        
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        
                        dribbling(db,0)
                  
                        tendangKuat(db)
                     
                        setStatus("1","RUNNING")
                        
                     
                        setJalan(motor,50,50,50,50,0.2)
                        
                        setJalan(motor,-120,-120,120,120,1)
                        
                        arahRobotKameraDpn2()
                        
                        setJalan(motor,-50,-50,-50,-50,0.7)
                        
                        arahRobotKameraDpn()
                        
                       
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        dribbling(db,0)
                     
                        tendangKuat(db)
                            
                        setStatus("1","RUNNING")
                     
                   
                        putarDerajat(70,0)
                        
                        
                         
                        setJalan(motor,-90,90,-90,90,2.8)
                      
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
        
            if dummy1 == "1" and (dummy2 == "7" or  dummy2 == "8" )   :
                
                if mode == "KICKOFF KANAN":
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#530512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        speed = 110
                        
                        setJalan(motor,speed,-speed,speed,-speed,0.7)
                        
                        setJalan(motor,speed+20,speed+20,-speed-50,-speed -50,2.3)
                        
                        setJalan(motor,speed,-speed,speed,-speed,2.2)
                       
            
                        
                        #LurusArahBola(200)
                        
                        dekatBola()
                        
                        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        if backHome("Bola Tengah") == True:
                            break
                        
                        #putarDerajat(20,1)
                        
                        #setJalan(motor,-50,-50,-50,-50,0.6)
                     
           
                        arahRobotKameraDpn()
                        
                        
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        
                        dribbling(db,0)
                  
                        tendangKuat(db)
                     
                        setStatus("1","RUNNING")
                        
                     
                        setJalan(motor,50,50,50,50,0.2)
                        
                        setJalan(motor,-120,-120,120,120,1)
                        
                        arahRobotKameraDpn2()
                        
                        setJalan(motor,-50,-50,-50,-50,0.7)
                        
                        arahRobotKameraDpn()
                        
                       
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        dribbling(db,0)
                     
                        tendangKuat(db)
                            
                        setStatus("1","RUNNING")
                     
                   
                        putarDerajat(70,0)
                        
                        
                         
                        setJalan(motor,-90,90,-90,90,2.8)
                      
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
                        
                elif mode == "KICKOFF KIRI":
                    
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#520512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                      
                        speed = 110
                        setJalan(motor,speed,-speed,speed,-speed,2.3)
                        
                        
                        speed = 90
                        
                        setJalan(motor,speed,speed,-speed,-speed,1.6)
                        
                        setJalan(motor,50,50,50,50,0.4)
                       
                      
                        sleep(3)
                        
                        
                        arahRobotKameraDpn2()
                        
                      
                     
                        setStatus("1","RUNNING")
                        
                        #putarDerajat(40,0)
                        
                        #setJalan(motor,-50,-50,-50,-50,0.5)
                  
                        
                        arahRobotKameraDpn()
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                    
                        dribbling(db,0)
                        
                        tendangOper(db)
                        
                        setStatus("1","RUNNING")
                   
                        putarDerajat(45,0)
                        
                        setJalan(motor,-120,-120,120,120,1.2)
                     
                        
                        #setStatus("1","ONPOS")
                        
                        
                        
                        
                        #terimaBola()
                        
                        setJalan(motor,50,50,50,50,0.5)
                      
                        
                        
                        arahRobotKameraDpn2()
                        
                   
                        putarDerajatBasing(10,1)
                        
                        if kiper == "1" :
                        
                            arahGawangKameraAtas("kanankiper")
                        
                        else  :
                        
                            arahGawangKameraAtas("kirikiper")
                     
                        dribbling(db,0)
                        
                        tendangKuatNian(db)
                         
                        putarDerajat(60,0)
                        
                        
                         
                        setJalan(motor,-90,90,-90,90,1.8)
                        
                      
                        
                        setStatus("1","IDLE")
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
                                            
            if dummy1 == "2" and (dummy2 == "7" or  dummy2 == "8" )  :
                
                if mode == "KICKOFF KANAN":
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#512512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        SerongKananLurusBola()
        
                        #setJalan(motor,200,-200,200,-200,0.3)
                        
              
                        #LurusArahBola(200)
                        
                        dekatBola()
                        
                        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        if backHome("Bola Tengah") == True:
                            break
                        
                        #putarDerajat(20,1)
                        
                        
                        setJalan(motor,-50,-50,-50,-50,0.6)
                     
                        sr.write(b"#520512")
                        
                        arahRobotKameraDpn()
                        
                        
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        
                        dribbling(db,0)
                  
                        tendangKuat(db)
                     
                        setStatus("1","RUNNING")
                        
                     
                        setJalan(motor,50,50,50,50,0.2)
                        
                        setJalan(motor,-120,-120,120,120,1)
                      
                        
                        arahRobotKameraDpn2()
                        
                        #setJalan(motor,-50,-50,-50,-50,0.3)
                        
                        arahRobotKameraDpn()
                        
                       
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        dribbling(db,0)
                     
                        tendangKuatNian(db)
                            
                        setStatus("1","RUNNING")
                     
                   
                        putarDerajat(50,0)
                        
                        
                         
                        setJalan(motor,-90,90,-90,90,2.8)
                      
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
                        
                elif mode == "KICKOFF KIRI":
                    
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#520512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        
                        speed =  180
                        
                        setJalan(motor,speed,0,0,-speed,1.5)
                      
                        setJalan(motor,speed,-speed,speed,-speed,0.7)
                        
                        
                      
                        
                        setJalan(motor,50,50,50,50,0.3)
                       
                      
                        sleep(3)
                        
                        #dekatBola()
                        
                        #putarDerajat(20,1)
                        
                        
                        arahRobotKameraDpn2()
                        
                      
                     
                        setStatus("1","RUNNING")
                        
                        #putarDerajat(40,0)
                        
                        #setJalan(motor,-50,-50,-50,-50,0.5)
                  
                        
                        arahRobotKameraDpn()
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                    
                        dribbling(db,0)
                        
                        tendangOper(db)
                        
                        setStatus("1","RUNNING")
                   
                        putarDerajat(45,0)
                        
                        setJalan(motor,-120,-120,120,120,1.0)
                     
                        
                        #setStatus("1","ONPOS")
                        
                        
                        
                        
                        #terimaBola()
                        
                        setJalan(motor,50,50,50,50,0.5)
                      
                        
                        
                        arahRobotKameraDpn2()
                        
                   
                        putarDerajatBasing(10,1)
                        
                        if kiper == "1" :
                        
                            arahGawangKameraAtas("kanankiper")
                        
                        else  :
                        
                            arahGawangKameraAtas("kirikiper")
                     
                        dribbling(db,0)
                        
                        tendangKuatNian(db)
                         
                        putarDerajat(50,0)
                        
                        
                         
                        setJalan(motor,-90,-90,90,90,1)
                        
                        setJalan(motor,-90,90,-90,90,1.8)
                        
                      
                        
                        setStatus("1","IDLE")
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
                        
            if dummy1 == "3" and (dummy2 == "7" or  dummy2 == "8" )  :
                
                if mode == "KICKOFF KANAN":
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#490512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        SerongKananLurusBola()
        
                        #setJalan(motor,200,-200,200,-200,0.3)
                        
              
                        #LurusArahBola(200)
                        
                        dekatBola()
                        
                        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        if backHome("Bola Tengah") == True:
                            break
                        
                        #putarDerajat(20,1)
                        
                        
                        setJalan(motor,-50,-50,-50,-50,0.6)
                     
                        sr.write(b"#512512")
                        
                        arahRobotKameraDpn()
                        
                        
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        
                        dribbling(db,0)
                  
                        tendangKuat(db)
                     
                        setStatus("1","RUNNING")
                        
                     
                        setJalan(motor,50,50,50,50,0.2)
                        
                        setJalan(motor,-120,-120,120,120,1)
                      
                        
                        arahRobotKameraDpn2()
                        
                        #setJalan(motor,-50,-50,-50,-50,0.3)
                        
                        arahRobotKameraDpn()
                        
                       
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        dribbling(db,0)
                     
                        tendangKuatNian(db)
                            
                        setStatus("1","RUNNING")
                     
                   
                        putarDerajat(50,0)
                        
                        
                         
                        setJalan(motor,-90,90,-90,90,2.8)
                      
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
                        
                elif mode == "KICKOFF KIRI":
                    
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#512512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        
                        speed =  180
                        
                        setJalan(motor,speed,0,0,-speed,1.6)
                      
                        setJalan(motor,speed,-speed,speed,-speed,0.75)
                        
                        
                      
                        
                        setJalan(motor,50,50,50,50,0.3)
                       
                      
                        sleep(6)
                        
                        #dekatBola()
                        
                        #putarDerajat(20,1)
                        
                        
                        arahRobotKameraDpn2()
                        
                      
                     
                        setStatus("1","RUNNING")
                        
                        #putarDerajat(40,0)
                        
                        #setJalan(motor,-50,-50,-50,-50,0.5)
                  
                        
                        arahRobotKameraDpn()
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                    
                        dribbling(db,0)
                        
                        tendangOper(db)
                        
                        setStatus("1","RUNNING")
                   
                        putarDerajat(40,0)
                        
                        setJalan(motor,-120,-120,120,120,1.0)
                     
                        
                        #setStatus("1","ONPOS")
                        
                        
                        
                        
                        #terimaBola()
                        
                        setJalan(motor,50,50,50,50,0.5)
                      
                        
                        
                        arahRobotKameraDpn2()
                        
                   
                   
                   
                        putarDerajatBasing(10,1)
                        
                        
                        #setJalan(motor,120,-120,120,-120,0.6)
                        
                        if kiper == "1" :
                        
                            arahGawangKameraAtas("kanankiper")
                        
                        else  :
                        
                            arahGawangKameraAtas("kirikiper")
                     
                        dribbling(db,0)
                        
                        tendangKuatNian(db)
                         
                        putarDerajat(40,0)
                        
                        
                         
                        
                        setJalan(motor,-90,90,-90,90,1.8)
                        
                      
                        
                        setStatus("1","IDLE")
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
        
            if dummy1 == "4" and (dummy2 == "7" or  dummy2 == "8" )  :
                
                if mode == "KICKOFF KANAN":
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#420512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        SerongKananLurusBola()
        
                        #setJalan(motor,200,-200,200,-200,0.3)
                        
              
                        #LurusArahBola(200)
                        
                        dekatBola()
                        
                        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        if backHome("Bola Tengah") == True:
                            break
                        
                        #putarDerajat(20,1)
                        
                        
                        setJalan(motor,-50,-50,-50,-50,0.6)
                     
                        sr.write(b"#500512")
                        
                        arahRobotKameraDpn()
                        
                        
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        
                        dribbling(db,0)
                  
                        tendangKuat(db)
                     
                        setStatus("1","RUNNING")
                        
                     
                        setJalan(motor,50,50,50,50,0.2)
                        
                        setJalan(motor,-120,-120,120,120,1)
                        
                        
                        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        if backHome("Tengah Lapang") == True:
                            break
                      
                        
                        arahRobotKameraDpn2()
                        
                        #setJalan(motor,-50,-50,-50,-50,0.3)
                        
                        arahRobotKameraDpn()
                        
                       
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        dribbling(db,0)
                     
                        tendangKuatNian(db)
                            
                        setStatus("1","RUNNING")
                     
                   
                        putarDerajat(50,0)
                        
                        
                         
                        setJalan(motor,-90,90,-90,90,2.8)
                      
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
                        
                elif mode == "KICKOFF KIRI":
                    
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#500512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        
                        speed =  180
                        
                        setJalan(motor,speed,0,0,-speed,1.5)
                      
                        setJalan(motor,speed,-speed,speed,-speed,0.7)
                        
                        
                      
                        
                        #setJalan(motor,50,50,50,50,0.3)
                       
                      
                        sleep(6)
                        
                        #dekatBola()
                        
                        #putarDerajat(20,1)
                        
                        
                        arahRobotKameraDpn2()
                        
                      
                     
                        setStatus("1","RUNNING")
                        
                        #putarDerajat(40,0)
                        
                        setJalan(motor,-50,-50,-50,-50,0.5)
                  
                        
                        arahRobotKameraDpn()
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                    
                        dribbling(db,0)
                        
                        tendangOper(db)
                        
                        setStatus("1","RUNNING")
                   
                        putarDerajat(40,0)
                        
                        setJalan(motor,-120,-120,120,120,1.0)
                     
                        
                        #setStatus("1","ONPOS")
                        
                        
                        
                        
                        #terimaBola()
                        
                        setJalan(motor,50,50,50,50,0.5)
                      
                        
                        
                        arahRobotKameraDpn2()
                        
                   
                   
                   
                        putarDerajatBasing(10,1)
                        
                        
                        #setJalan(motor,120,-120,120,-120,0.6)
                        
                        if kiper == "1" :
                        
                            arahGawangKameraAtas("kanankiper")
                        
                        else  :
                        
                            arahGawangKameraAtas("kirikiper")
                     
                        dribbling(db,0)
                        
                        tendangKuatNian(db)
                         
                        putarDerajat(40,0)
                        
                        
                         
                        
                        setJalan(motor,-90,90,-90,90,1.8)
                        
                      
                        
                        setStatus("1","IDLE")
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
   
            if dummy1 == "5" and (dummy2 == "7" or  dummy2 == "8" )  :
                
                if mode == "KICKOFF KANAN":
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#470512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        SerongKananLurusBola()
        
                        #setJalan(motor,200,-200,200,-200,0.3)
                        
              
                        #LurusArahBola(200)
                        
                        dekatBola()
                       
                        
                        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        if backHome("Bola Tengah") == True:
                            break
                        
                        #putarDerajat(20,1)
                        
                        
                        setJalan(motor,-50,-50,-50,-50,0.6)
                     
                        sr.write(b"#500512")
                        
                        arahRobotKameraDpn()
                        
                        
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        
                        dribbling(db,0)
                  
                        tendangKuat(db)
                     
                        setStatus("1","RUNNING")
                        
                     
                        setJalan(motor,50,50,50,50,0.2)
                        
                        setJalan(motor,-120,-90,90,120,1)
                        
                       
                        
                        
                        #dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        #if backHome("Tengah Lapang") == True:
                        #    break
                      
                        
                        arahRobotKameraDpn2()
                        
                        #setJalan(motor,-50,-50,-50,-50,0.3)
                        
                        arahRobotKameraDpn()
                        
                       
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        dribbling(db,0)
                     
                        tendangKuat(db)
                            
                        setStatus("1","RUNNING")
                     
                   
                        putarDerajat(50,0)
                        
                        
                         
                        setJalan(motor,-90,90,-90,90,2.8)
                      
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
                        
                elif mode == "KICKOFF KIRI":
                    
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#500512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        
                        speed =  180
                        
                        setJalan(motor,speed,0,0,-speed,1.5)
                      
                        setJalan(motor,speed,-speed,speed,-speed,0.45)
                        
                        
                      
                        
                        setJalan(motor,50,50,50,50,0.3)
                       
                      
                        sleep(6)
                        
                        #dekatBola()
                        
                        #putarDerajat(20,1)
                        
                        
                        arahRobotKameraDpn2()
                        
                      
                     
                        setStatus("1","RUNNING")
                        
                        #putarDerajat(40,0)
                        
                        setJalan(motor,-50,-50,-50,-50,0.5)
                  
                        
                        arahRobotKameraDpn()
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                    
                        dribbling(db,0)
                        
                        tendangOper(db)
                        
                        setStatus("1","RUNNING")
                   
                        putarDerajat(40,0)
                        
                        setJalan(motor,-120,-120,120,120,1.0)
                     
                        
                        #setStatus("1","ONPOS")
                        
                        
                        
                        
                        #terimaBola()
                        
                        setJalan(motor,50,50,50,50,0.3)
                      
                        
                        
                        arahRobotKameraDpn2()
                        
                   
                   
                   
                        putarDerajatBasing(10,1)
                        
                        
                        #setJalan(motor,120,-120,120,-120,0.6)
                        
                        if kiper == "1" :
                        
                            arahGawangKameraAtas("kanankiper")
                        
                        else  :
                        
                            arahGawangKameraAtas("kirikiper")
                     
                        dribbling(db,0)
                        
                        tendangKuatNian(db)
                         
                        putarDerajat(40,0)
                        
                        
                         
                        
                        setJalan(motor,-90,90,-90,90,1.8)
                        
                      
                        
                        setStatus("1","IDLE")
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break            
            
            if dummy1 == "6" and (dummy2 == "7" or  dummy2 == "8" )  :
                
                if mode == "KICKOFF KANAN":
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#490512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        SerongKananLurusBola()
        
                        #setJalan(motor,200,-200,200,-200,0.3)
                        
              
                        #LurusArahBola(200)
                        
                        dekatBola()
                       
                        
                        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        if backHome("Bola Tengah") == True:
                            break
                        
                        #putarDerajat(20,1)
                        
                        
                        setJalan(motor,-50,-50,-50,-50,0.6)
                     
                        sr.write(b"#530512")
                        
                        arahRobotKameraDpn()
                        
                        
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        
                        dribbling(db,0)
                  
                        tendangKuat(db)
                     
                        setStatus("1","RUNNING")
                        
                     
                        setJalan(motor,50,50,50,50,0.2)
                        
                        setJalan(motor,-90,-120,120,90,1)
                        
                       
                        
                        
                        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
                        
                        if backHome("Tengah Lapang") == True:
                            break
                      
                        
                        arahRobotKameraDpn2()
                        
                        setJalan(motor,-50,-50,-50,-50,0.3)
                        
                        arahRobotKameraDpn()
                        
                       
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                        
                        dribbling(db,0)
                     
                        tendangKuatNian(db)
                            
                        setStatus("1","RUNNING")
                     
                   
                        putarDerajat(50,0)
                        
                        
                         
                        setJalan(motor,-90,90,-90,90,2.8)
                      
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break
                        
                elif mode == "KICKOFF KIRI":
                    
                    
                    while gameStatus == "START": 
                  
                        setStatus("1","IDLE")
                        
                        sr.write(b"#512512")
                        
                        #putarDerajat(52,0)
                        
                        setStatus("1","RUNNING")
                        
                        
                        speed =  180
                        
                        setJalan(motor,speed,0,0,-speed,1.5)
                      
                        setJalan(motor,speed,-speed,speed,-speed,0.45)
                        
                        
                      
                        
                        setJalan(motor,50,50,50,50,0.3)
                       
                      
                        sleep(6)
                        
                        #dekatBola()
                        
                        #putarDerajat(20,1)
                        
                        
                        arahRobotKameraDpn2()
                        
                      
                     
                        setStatus("1","RUNNING")
                        
                        #putarDerajat(40,0)
                        
                        setJalan(motor,-50,-50,-50,-50,0.5)
                  
                        
                        arahRobotKameraDpn()
                        
                        setStatus("1","READY")
                        tungguStatus("2","READY")
                    
                        dribbling(db,0)
                        
                        tendangOper(db)
                        
                        setStatus("1","RUNNING")
                   
                        putarDerajat(40,0)
                        
                        setJalan(motor,-120,-120,120,120,1.0)
                     
                        
                        #setStatus("1","ONPOS")
                        
                        
                        
                        
                        #terimaBola()
                        
                        setJalan(motor,50,50,50,50,0.3)
                      
                        
                        
                        arahRobotKameraDpn2()
                        
                   
                   
                   
                        putarDerajatBasing(10,1)
                        
                        
                        #setJalan(motor,120,-120,120,-120,0.6)
                        
                        if kiper == "1" :
                        
                            arahGawangKameraAtas("kanankiper")
                        
                        else  :
                        
                            arahGawangKameraAtas("kirikiper")
                        
                        dribbling(db,0)
                        
                        tendangKuatNian(db)
                         
                        putarDerajat(40,0)
                        
                        
                         
                        
                        setJalan(motor,-90,90,-90,90,1.8)
                        
                      
                        
                        setStatus("1","IDLE")
                        
                        setGame("STOP")
                        setStatus("1","IDLE")
                        break                 
                        
    if mode == 'tes':
        #sr.open()
        sr.write(b"#480512")
        
        speed = 180
        
        setJalan(motor,-60,-speed,speed,60,1.2)
        
        setJalan(motor,-speed,-speed,-speed,-speed ,0.1)
        
        
        
        arahRobotKameraDpnCorner()
        
        dekatBola()
        
        setJalan(motor,speed,speed,speed,speed ,0.2)
        
        
        arahRobot()
        
        setStatus("1","READY")
        tungguStatus("2","READY")
    
        dribbling(db,0)
                        
        tendangOper(db)
        
        setStatus("1","RUNNING")

        
        
   


        
        
        
        
   
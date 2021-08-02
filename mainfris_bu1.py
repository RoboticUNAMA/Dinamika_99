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

def tendangKuat(ser) :
    ser.write(b"TEND1\n")

def tendangOper(ser) :
    ser.write(b"TEND2\n")
        
def main():
    # serial motor driver
    motor = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)

    # serial dribble
    db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=0)
    que  = queue.Queue()
    #que = "tes"
    #thread = threading.Thread(target=read_from_port, args=(db,que,),daemon=True)
    #thread.start()

    # initialize
    font = cv2.FONT_HERSHEY_SIMPLEX
    date = str(datetime.datetime.now().strftime("%d-%m-%Y %H:%M:%S"))
    FRONT_CAM = 0   # front camera
    OMNI_CAM = 1    # omni camera

    # create opencv video capture object
    FRONT_CAP = cv2.VideoCapture(FRONT_CAM) 
    OMNI_CAP = cv2.VideoCapture(OMNI_CAM)
  

    # set frame size
    FRONT_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    FRONT_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

    OMNI_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
    OMNI_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)

    # get center of the frame
    _, frame1 = FRONT_CAP.read()
    rows, cols, _ = frame1.shape
    cenX_frame1 = int(cols/2)
    cenY_frame1 = int(rows/2)

    _, frame2 = OMNI_CAP.read()
    rows1, cols1, _ = frame2.shape
    cenX_frame2 = int(cols1/2)
    cenY_frame2 = int(rows1/2)

    # define center area of the frame 1
    inner_left = cenX_frame1 - 100
    outer_left = cenX_frame1 - 250 
    inner_right = cenX_frame1 + 100
    outer_right = cenX_frame1 + 250
    inner_top = cenY_frame1 - 100
    outer_top = cenY_frame1 - 150
    inner_bottom = cenY_frame1 + 100
    outer_bottom = cenY_frame1 + 150

    xAwal = 100
    xAkhir = 380
    yAwal = 105
    yAkhir = 210
    
    speed = 80
    
    ballColor = getBallInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)

    wait = 0

    speed_awal = 120.0
    
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
    
    degreeInit = 0
    count = 1
    derajat_tujuan = 20
                  
    state = "NERIMA_BOLA"
    

    while(True):
        #print(state)
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
        
        
        
        if state == "MUTAR_DERAJAT" :    
            reading = db.readline().decode()
            if len(reading) > 0 :
                degree = float(reading[10:-9])
                #print(degree)
                #degree =  float(que.get())
                selisih =  degree - derajat_tujuan
                if count == 1 :    
                    count = 0
                    if selisih > 180 :
                        speed = 70
                    elif selisih < -180 : 
                        speed = -70
                    else :
                        if selisih >= 0 :
                            speed = -70 
                        else : 
                            speed = 70
                        
                    
                selisih_error = abs(selisih)
                if selisih_error > 180 :
                    selisih_error = 360 - selisih_error
            
                print("Degree " + str(degree))
                if derajat_tujuan > 150 : 
                    rentang = 60
                else : 
                    rentang = 10
                if selisih_error < rentang :
                    if speed > 0 :
                        speed = -40
                    else :
                        speed = 40
                        
                    setMotor(motor,speed,speed,speed,speed)
                    
                    sleep(0.1)
                    
                    setMotor(motor,0,0,0,0)
                    
                    state = "OK"
                    
                else : 
                
                   
                    if derajat_tujuan > 150 : 
                        rentang2 = 200
                    else : 
                        rentang2 = 70
                    if selisih_error < rentang2 :
                        if speed > 0 :
                            speed  = 40
                        else :
                            speed = -40
                    
                
                    setMotor(motor,speed,speed,speed,speed)
                
                
            
        
        for goalContour in goalContours:
            goal_area = cv2.contourArea(goalContour)
            #print(goal_area)
            if goal_area > 5:
                (x_goal, y_goal, w_goal, h_goal) = cv2.boundingRect(goalContour)
                cenX_goal = (x_goal+x_goal+w_goal)/2
                cenY_goal = (y_goal+y_goal+h_goal)/2
                #predicted = kfObj.Estimate(cenX, cenY)

                # draw actual coordinate from segmentation
                cv2.rectangle(frame1, (x_goal, y_goal), ((x_goal+w_goal),(y_goal+h_goal)), [255,0,0], 2)
                cv2.putText(frame1, "Gawang", (x_goal, y_goal), font, 0.5, [0,255,0], 2)
                #print(cenX_goal)
                
                break
        
    
        
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
                
                #print(cenX_ball1)
                
                if cenX_ball1 > xAwal and cenX_ball1 < xAkhir and cenY_ball1 > yAwal and cenY_ball1 < yAkhir and state == "CARI BOLA":
                    
                    setMotor(motor,0,0,0,0)
                    
                    state = "LURUS BOLA"
                    
                if state == "LURUS BOLA"  :
                    if cenX_ball1 < 200  :
                    
                        setMotor(motor,80,80,-80,80)
                        
                        print("LURUS KANAN")
                        
                    elif cenX_ball1 > 245 :
                        setMotor(motor,-130,-80,90,-80)
                        print("LURUS KIRI")
                    else:
                    
                        setMotor(motor,0,0,0,0)
                        
                        dribbling(db,1)
                        
                        state = "MAJU DRIBBLING"
                        
                    
                if state == "MAJU DRIBBLING":
                    if cenX_ball1 < 220  :
                    
                        setMotor(motor,-55,55,55,55)
                        
                        print("PUTAR KANAN")
                    
                    elif cenX_ball1 > 320 :
                    
                        setMotor(motor,55,-55,-55,-55)
                        
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
                        
                      
                        setMotor(motor,80 + PID,-50,95 + PID ,80 )
                        
                        print("MAJU PID" + str(PID))
                        
                        if cenX_ball1 > 224 and cenX_ball1 < 244 and cenY_ball1 > 150 and cenY_ball1 < yAkhir:
                            setMotor(motor,0,0,0,0)
                            
                            cenX_goal = 0
                            
                            state = "DAPAT BOLA"
                            
                           
                
                if state == "DAPAT BOLA" :
                    if count_cr_gawang_kiri < 300:
                        count_cr_gawang_kiri += 1
                        print(count_cr_gawang_kiri)
                        if cenX_goal >0 :
                            
                            setMotor(motor,0,0,0,0)
                            
                            dribbling(db,0)
                            
                            sleep(1)
                            
                            tendangKuat(db)
                            
                            state = "SELESAI"
                                
                        else : 
                        
                            setMotor(motor,65,-65,-65,-65)
                            
                            print("PUTAR KIRI SHOOT")
                    elif count_cr_gawang_kanan < 300 :
                        count_cr_gawang_kanan += 1
                        print(count_cr_gawang_kiri)
                        if cenX_goal >0 :
                        
                            setMotor(motor,0,0,0,0)
                            
                            dribbling(db,0)
                            
                            sleep(1)
                            
                            tendangKuat(db)
                            
                            state = "SELESAI"
                            
                        else : 
                        
                            setMotor(motor,-65,65,65,65)
                            
                            print("PUTAR KANAN SHOOT")
                            
                    
                    else :
                        setMotor(motor,0,0,0,0)
                        
                        
                        dribbling(db,0)
                        
                        state = "SELESAI"
                        
               
                    
                    
               
                if state == "CARI BOLA":
                    if cenY_ball1 <200:
                        if cenX_ball1 < cenX_frame2:
                           error =  cenX_ball1 - cenX_frame2
                           selisih_error = error_sebelumnya - error
                           jumlah_error += (0.001 * error)
                           error_sebelumnya = error
                           P = Kp * error
                           D = Kd * selisih_error
                           I = Ki * jumlah_error
                           PID = P + I  +D
                           PID = PID /2
                           
                           dki = speed_awal 
                           dka = -PID
                           bki = PID 
                           bka = speed_awal 
                           
                           setMotor(motor,dki, dka, bki, bka)
                           
                           print(PID)
                                   
                        elif cenX_ball1 > cenX_frame2:
                           error = cols1 - cenX_ball1
                           selisih_error = error_sebelumnya - error
                           jumlah_error += (0.001 * error)
                           error_sebelumnya = error
                           P = Kp * error
                           D = Kd * selisih_error
                           I = Ki * jumlah_error
                           PID = P + I  +D
                           PID = PID /2
                           dki =  PID
                           dka = -speed_awal
                           bki = speed_awal 
                           bka = PID 
                           
                           setMotor(motor,PID, -speed_awal, speed_awal + 10, PID)

                         
                           print(PID)
                               
                    else :
                         #motor.write(b"#M|STP|0\n")
                        error =  cenX_frame2 - cenX_ball1
                        selisih_error = error_sebelumnya - error
                        jumlah_error += (0.001 * error)
                        error_sebelumnya = error
                        P = Kp * error
                        D = Kd * selisih_error
                        I = Ki * jumlah_error
                        PID = P + I  +D
                        PID /= 2
                        print(PID)
                        if PID > 40 :
                            PID = 40
                        if PID < -40 :
                            PID = -40
                            
                        if cenX_ball1 < 340 and cenX_ball1 > 250 : 
                            PID = -PID
                            
                 
                            setMotor(motor,PID, PID, PID, -PID)
                            
                            sleep(0.1)
                            
                            PID = 0
                            
                            setMotor(motor,PID, PID, PID, -PID)
                        
                        
                            
                        else :
                        
                            setMotor(motor,PID, PID, PID, -PID)
                    
                 
                    
               
                    
                
                break

        
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
                
                if state == "NERIMA_BOLA" :
                   
                    
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
        cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        
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
    main()

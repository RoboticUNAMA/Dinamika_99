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

def main():
    # serial motor driver
    motor = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

    # serial dribble
    db = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)

    # initialize
    font = cv2.FONT_HERSHEY_SIMPLEX
    date = str(datetime.datetime.now().strftime("%d-%m-%Y %H:%M:%S"))
    FRONT_CAM = 1   # front camera
    OMNI_CAM = 0    # omni camera

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
    cenX_frame2 = 235
    cenY_frame2 = int(rows1/2)

    # define center area of the frame 1
    inner_left = cenX_frame1 - 100
    outer_left = cenX_frame1 - 225 
    inner_right = cenX_frame1 + 100
    outer_right = cenX_frame1 + 225
    inner_top = cenY_frame1 - 100
    outer_top = cenY_frame1 - 150
    inner_bottom = cenY_frame1 + 100
    outer_bottom = cenY_frame1 + 150

    xAwal = 100
    xAkhir = 380
    yAwal = 95
    yAkhir = 210
    
    ballColor = getBallInfo()
    goalColor = getGoalInfo()
    kfObj = KalmanFilter()
    predictedCoords = np.zeros((2,1), np.float32)

    wait = 0

    speed_awal = 140.0
    Kp = 5.0
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
    
    dki =  80
    dka = -50
    bki = 95
    bka = 80
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("MAJU")
                  
    sleep(1)
    
        
    dki =  0
    dka = 0
    bki = 0
    bka = 0

    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                  
    state = "CARI BOLA"
    while(True):
        print(state)
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
        
        for ballContour1 in ballContours1:
            ball_area1 = cv2.contourArea(ballContour1)
            
            if ball_area1 > 10:
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
                    dki =  0
                    dka = 0
                    bki = 0
                    bka = 0
                    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                    state = "LURUS BOLA"
                    
                if state == "LURUS BOLA"  :
                    if cenX_ball1 < 200  :
                        dki =  130
                        dka = 80
                        bki = -90
                        bka = 80
                        motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8')) 
                        print("LURUS KANAN")
                        
                    elif cenX_ball1 > 230 :
                        dki =  -130
                        dka = -80
                        bki = 90
                        bka = -80
                        motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8')) 
                        print("LURUS KIRI")
                    else:
                        dki =  0
                        dka = 0
                        bki = 0
                        bka = 0
                        motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                        state = "MAJU DRIBBLING"
                        db.write(b"DB ON\n")
                    
                if state == "MAJU DRIBBLING":
                    if cenX_ball1 < 220  :
                        dki =  -55
                        dka = 55
                        bki = 55
                        bka = 55
                        motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8')) 
                        print("PUTAR KANAN")
                    
                    elif cenX_ball1 > 250 :
                        dki =  55
                        dka = -55
                        bki = -55
                        bka = -55
                        motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8')) 
                        print("PUTAR KIRI")
                    else:
                    
                        dki =  80
                        dka = -50
                        bki = 95
                        bka = 80
                        motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                        print("MAJU")
                        
                        if cenX_ball1 > 224 and cenX_ball1 < 244 and cenY_ball1 > 164 and cenY_ball1 < yAkhir:
                            dki =  0
                            dka = 0
                            bki = 0
                            bka = 0
                            motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                            state = "DAPAT BOLA"
                    
                            dki =  80
                            dka = -80
                            bki = -80
                            bka = -80
                            motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8')) 
                            print("PUTAR KIRI")
                            
                            sleep(2)
                            dki =  0
                            dka = 0
                            bki = 0
                            bka = 0
                            motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                            
                
              
               
               
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
                           PID = PID /20
                           dki = speed_awal + 40
                           dka = -PID
                           bki = PID + 10
                           bka = speed_awal 
                           
                           motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                          
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
                           PID = PID /20
                           dki =  PID +40
                           dka = -speed_awal
                           bki = speed_awal + 10
                           bka = PID 

                           motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                      
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
                        #print(PID)
                        if PID > 40 :
                            PID = 40
                        if PID < -40 :
                            PID = -40
                            
                        if cenX_ball1 < 340 and cenX_ball1 > 250 : 
                            PID = -PID
                            
                            dki =  PID
                            dka = PID
                            bki = PID 
                            bka = -PID
                            
                            motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                            
                            sleep(0.1)
                            
                            PID = 0
                        
                            dki =  PID
                            dka = PID
                            bki = PID 
                            bka = -PID
                            
                            motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                            
                        else :
                        
                            dki =  PID
                            dka = PID
                            bki = PID 
                            bka = -PID
                            
                            motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                    
                 
                    
               
                    
                
                break

        # for ballContour in ballContours:
            # ball_area = cv2.contourArea(ballContour)
            # if ball_area > 500:
                # (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                # cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                # cenX_ball = (x_ball+x_ball+w_ball)/2
                # cenY_ball = (y_ball+y_ball+h_ball)/2
                # predicted = kfObj.Estimate(cenX_ball, cenY_ball)

                # # draw actual coordinate from segmentation
                # cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                # cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                # cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)

                # # Draw Kalman Filter Predicted output
                # #cv2.circle(frame1, (predictedCoords[0], predictedCoords[1]), 20, [255,0,0], 2, 8)
                # #cv2.line(frame1, 
                        # #(predictedCoords[0] + 16, predictedCoords[1] - 15), 
                        # #(predictedCoords[0] + 50, predictedCoords[1] - 30),
                        # #[255, 0, 0], 2, 8)
                # #cv2.putText(frame1,
                        # #"Predicted", 
                        # #(int(predictedCoords[0] + 50),
                        # #int(predictedCoords[1] - 30)), 
                        # #font, 0.5, [255, 0, 0], 2)

               

                # # kirim serial aktifkan dribble
                # #db.write(b"DB ON\n")
                # if cenY_ball < rows - 50 :
                    # if cenX_ball > cenX_frame1:
                           # error =  cols - cenX_ball
                           # selisih_error = error_sebelumnya - error
                           # jumlah_error += (0.001 * error)
                           # error_sebelumnya = error
                           # P = Kp * error
                           # D = Kd * selisih_error
                           # I = Ki * jumlah_error
                           # PID = P + I  +D
                           # PID = PID /2
                           # dki = speed_awal 
                           # dka = -PID
                           # bki = PID + 10
                           # bka = speed_awal

                           # motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                          
                           # print(PID)
                           
                    # elif cenX_ball < cenX_frame1:
                           # error =  cenX_ball 
                           # selisih_error = error_sebelumnya - error
                           # jumlah_error += (0.001 * error)
                           # error_sebelumnya = error
                           # P = Kp * error
                           # D = Kd * selisih_error
                           # I = Ki * jumlah_error
                           # PID = P + I  +D
                           # PID = PID /2
                           # dki =  PID
                           # dka = -speed_awal
                           # bki = speed_awal + 10
                           # bka = PID

                           # motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
                      
                           # print(PID)
                           
                # else :
                     # motor.write(b"#M|STP|0\n")
                       

                # break
            # else: 
                # motor.write(b"#M|STP|0\n")

        for goalContour in goalContours:
            goal_area = cv2.contourArea(goalContour)
            if goal_area > 500:
                (x_goal, y_goal, w_goal, h_goal) = cv2.boundingRect(goalContour)
                cenX_goal = (x_goal+x_goal+w_goal)/2
                cenY_goal = (y_goal+y_goal+h_goal)/2
                #predicted = kfObj.Estimate(cenX, cenY)

                # draw actual coordinate from segmentation
                cv2.rectangle(frame1, (x_goal, y_goal), ((x_goal+w_goal),(y_goal+h_goal)), [255,0,0], 2)
                cv2.putText(frame1, "Gawang", (x_goal, y_goal), font, 0.5, [0,255,0], 2)
                break
 
       

        # displays

        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        cv2.imshow("Kamra Atas", frame2)
        #cv2.imshow("Kamera Depan", frame1)
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

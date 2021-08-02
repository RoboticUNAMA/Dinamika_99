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

# serial dribble
db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

#serial OpenCM
#cm = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=1)
#cm.close()

# initialize
font = cv2.FONT_HERSHEY_SIMPLEX
FRONT_CAM = 1   # front camera
OMNI_CAM = 0   # omni camera

# create opencv video capture object
FRONT_CAP = cv2.VideoCapture(FRONT_CAM) 
OMNI_CAP = cv2.VideoCapture(OMNI_CAM)

# set frame size
FRONT_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
FRONT_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)

OMNI_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
OMNI_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

def setMotor(ser,dki,dka,bki,bka) :
    if ser.isOpen() == False:
        ser.open()
    dki = dki + (dki * 0.3)
    dka = dka + (dka * 0)
    bki = bki + (bki * 0)
    bka = bka + (bka * 0.3) 
    ser.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))

def dribbling(ser,val) :
    if ser.isOpen() == False:
        ser.open()
    if val == 1 :
        ser.write(b"DB ON\n")
    else : 
        ser.write(b"DB OFF\n")

def compass(ser, val) :
    if ser.isOpen() == False:
        ser.open()
    if val == 1:
        ser.write(b"COMPASS ON\n")
    else:
        ser.write(b"COMPASS OFF\n")

def bacaCompass(ser):
    if ser.isOpen() == False:
        ser.open()
    read = ser.readline().decode('utf-8','ignore')
    ser.close()
    return read

def tendang(ser):
    if ser.isOpen() == False:
        ser.open()
    ser.write(b"TEND1\n")

def oper(ser):
    if ser.isOpen() == False:
        ser.open()
    ser.write(b"TEND2\n")

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

def putarDerajat(derajat_tujuan, dribble) :
    if db.isOpen() == False:
        db.open()

    speed = 70
    state = "START"
    clb = 0
        
    while(True) :
        compass(db,1)
    
        if dribble == 1 :
            dribbling(db,1)
        else :
            dribbling(db,0)
            
        if state == "FINISH" :
            compass(db, 0)
            motor.close()
            break
 
        reading = bacaCompass(db)
        
        if len(reading) > 0 :
            print(reading)
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

def arahBolaDepan():
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

    # read magenta color
    objColor = getBallInfo()
    lowerBall = np.array([objColor[0],objColor[1],objColor[2]])
    upperBall = np.array([objColor[3],objColor[4],objColor[5]])

    dari = ""
    second = 0
    startCount = 10
    count = startCount
    speed = 60
    state = "START"

    dribbling(db,1)
    db.flush()

    while(True):
        if count <= 0:
            motor.close()
            count = startCount
        count -= 1
        #print(state)
        second += 1
        #print(second)
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
        _, BALL_THRESH = cv2.threshold(BALL_MASK, objColor[6], 255, 0)
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, objColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        if state == "FINISH":
            setMotor(motor,0,0,0,0)
            motor.close()
            cv2.destroyAllWindows()
            break

        if db.isOpen() == False:
            db.open()
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"    
         
        pas = 0

        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            if ball_area > 500:
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2   
                print(cenX_ball)
                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenX_ball > 0 and cenX_ball < 150  :
                    # kiri robot
                    #print("KIRI JAUH")
                    dari = "kiri"
                    setMotor(motor,28,28,28,28)
                elif cenX_ball > 250:
                #     #print("KANAN JAUH")
                    dari = "kanan"
                    setMotor(motor,-28,-28,-28,-28)
                else:
                    if dari == "kanan" :
                        setMotor(motor,30,30,30,30)
                        sleep(0.1)
                        setMotor(motor,0,0,0,0)
                        dari = "tengah"
                    else :
                        setMotor(motor,-30,-30,-30,-30)
                        sleep(0.1)
                        setMotor(motor,0,0,0,0)
                        dari = "tengah"
                    
                    
                # elif cenX_ball > 300  :
                #     # kanan robot
                #     #print("KANAN JAUH")
                #     setMotor(motor,30,30,30,30)
                    
                # elif cenX_ball > 50 and cenX_ball < 150  :
                #     #print("KIRI DEKAT")
                #     setMotor(motor,-35,-35,-35,-35)
                #     sleep(0.1)
                #     setMotor(motor,0,0,0,0)
                #     dari = "kanan"
                #     #print("PUTAR KANAN")
                
                # elif cenX_ball > 250 :
                #     #print("KANAN DEKAT")
                #     setMotor(motor,35,35,35,35)
                #     sleep(0.1)
                #     setMotor(motor,0,0,0,0)
                #     dari = "kiri"
                #     #print("PUTAR KIRI")
                # elif cenX_ball > 150 and cenX_ball < 250:
                #     pas = 1
                # elif cenX_ball <= 0:
                #     setMotor(motor, 0,0,0,0)
                break
        
        if state == "FINISH" and pas == 1: 
            setMotor(motor,0,0,0,0)
            motor.close()
            cv2.destroyAllWindows()
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
            if motor.isOpen == False:
                motor.open()
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def arahRobotDepan():
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

    # read magenta color
    objColor = getMagentaInfo()
    lowerBall = np.array([objColor[0],objColor[1],objColor[2]])
    upperBall = np.array([objColor[3],objColor[4],objColor[5]])

    dari = ""
    second = 0
    startCount = 10
    count = startCount
    speed = 60
    state = "START"

    dribbling(db,1)
    db.flush()

    while(True):
        if count <= 0:
            motor.close()
            count = startCount
        count -= 1
        #print(state)
        second += 1
        #print(second)
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
        _, BALL_THRESH = cv2.threshold(BALL_MASK, objColor[6], 255, 0)
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, objColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        if state == "FINISH":
            setMotor(motor,0,0,0,0)
            motor.close()
            cv2.destroyAllWindows()
            break

        if db.isOpen() == False:
            db.open()
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Tidak" :
                print("TIDAK DAPAT BOLA")
                arahBolaDepan()
                break    
         
        pas = 0

        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            if ball_area > 500:
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2   
                print(cenX_ball)
                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenX_ball > 0 and cenX_ball < 170  :
                    # kiri robot
                    #print("KIRI JAUH")
                    dari = "kiri"
                    setMotor(motor,28,28,28,28)
                elif cenX_ball > 240:
                #     #print("KANAN JAUH")
                    dari = "kanan"
                    setMotor(motor,-28,-28,-28,-28)
                else:
                    if dari == "kanan" :
                        setMotor(motor,30,30,30,30)
                        sleep(0.1)
                        setMotor(motor,0,0,0,0)
                        dari = "tengah"
                    else :
                        setMotor(motor,-30,-30,-30,-30)
                        sleep(0.1)
                        setMotor(motor,0,0,0,0)
                        dari = "tengah"


                if dari == "tengah":
                    state = "FINISH"
                break
        
        if state == "FINISH" and pas == 1: 
            setMotor(motor,0,0,0,0)
            motor.close()
            cv2.destroyAllWindows()
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
            if motor.isOpen == False:
                motor.open()
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def mulaiSerongKiri():
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

    # read magenta color
    objColor = getBallInfo()
    lowerBall = np.array([objColor[0],objColor[1],objColor[2]])
    upperBall = np.array([objColor[3],objColor[4],objColor[5]])

    dari = ""
    second = 0
    startCount = 10
    count = startCount
    speed = 80
    state = "START"

    dribbling(db,1)
    db.flush()

    setMotor(motor, 0,speed,-speed,0) # serong kiri

    while(True):
        if count <= 0:
            motor.close()
            count = startCount
        count -= 1
        #print(state)
        second += 1
        #print(second)
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
        _, BALL_THRESH = cv2.threshold(BALL_MASK, objColor[6], 255, 0)
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, objColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        if state == "FINISH":
            setMotor(motor,0,0,0,0)
            motor.close()
            cv2.destroyAllWindows()
            break

        if db.isOpen() == False:
            db.open()
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Tidak" :
                print("TIDAK DAPAT BOLA")
                arahBolaDepan()
                break    
         
        pas = 0

        if state == "FINISH"  or second > 25: 
            setMotor(motor,0,0,0,0)
            break

        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            if ball_area > 500:
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2   
                print(cenX_ball)
                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenX_ball > inner_left :
                    setMotor(motor,0,-50,50,0)
                    sleep(0.1)    
                    setMotor(motor,0,0,0,0)
                    state = "FINISH"   

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
            if motor.isOpen == False:
                motor.open()
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def main():
    mode = int(input("Mode = "))

    # serial motor driver
    motor = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)

    # serial dribble
    db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

    #serial OpenCM
    #cm = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=1)
    #cm.close()

    # initialize camera
    _, frame1 = FRONT_CAP.read()
    _, frame2 = OMNI_CAP.read()

    #dummy 3 dan 8

    #cm.write(b"#450512")

    if mode == 1:
        # lurusin
        putarDerajat(87,0)
        putarDerajat(99,1)

        setMotor(motor, -80,80,-80,80) # motor maju
        sleep(1.9)
        setMotor(motor, 50,-50,50,-50) # rem maju
        sleep(0.1)
        setMotor(motor, 0,0,0,0)

        arahBolaDepan()

        putarDerajat(86.5, 1)

        arahRobotDepan()

        # === init tendang
        dribbling(db, 0)
        sleep(1)
        dribbling(db, 0)
        sleep(0.5)
        oper(db)
        # ================
        sleep(2)

        setMotor(motor, -80,-80,80,80) # motor geser kanan
        sleep(1.5)
        setMotor(motor, 50,50,-50,-50) # motor geser kanan
        sleep(0.1)
        setMotor(motor, 0,0,0,0)
        sleep(0.5)
        putarDerajat(98,1)
        arahBolaDepan()

        setMotor(motor, -80,-80,-80,-80) # motor putar kanan
        sleep(0.64) 
        setMotor(motor, 50,50,50,50) # rem putar kanan
        sleep(0.1)
        setMotor(motor, 0,0,0,0) # motor stop

        # === init tendang
        dribbling(db, 0)
        sleep(1)
        dribbling(db, 0)
        sleep(0.5)
        tendang(db)
        # ================
        sleep(2)

        putarDerajat(87,0)
        sleep(0.2)

        setMotor(motor, 80,-80,80,-80) # motor mundur
        sleep(1.5)
        setMotor(motor, -50,50,-50,50) # rem mundur
        sleep(0.1)
        setMotor(motor, 0,0,0,0)

    elif mode == 2:
        # lurusin
        putarDerajat(87,0)
        mulaiSerongKiri()


if __name__ == '__main__':
    # execute main program
    main()
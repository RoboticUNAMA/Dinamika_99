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
import requests
from time import sleep

# webserver
ip_server = "192.168.10.244"

# serial motor driver
motor = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)

# serial dribble
db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

# STM
# stm = serial.Serial(port='/dev/ttyACM2', baudrate=9600, timeout=1)

# # serial OpenCM
# cm = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=1)
# cm.close()

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

def setStatus(id, status):
    requests.post("http://"+ip_server+"/robot/setstatus.php?"+"id="+str(id)+"&status="+str(status))
def setGame(status):
    requests.post("http://"+ip_server+"/robot/setgame.php?"+"status="+str(status))
def getStatus(id):
    get = requests.post("http://"+ip_server+"/robot/getstatus.php?"+"id="+str(id))
    return get.text.strip()
def getGameInfo():
    dummy1 = requests.post("http://"+ip_server+"/robot/getdummy1.php")
    dummy2 = requests.post("http://"+ip_server+"/robot/getdummy2.php")
    kiper = requests.post("http://"+ip_server+"/robot/getkiper.php")
    mode = requests.post("http://"+ip_server+"/robot/getmode.php")
    gameStatus = requests.post("http://"+ip_server+"/robot/getgame.php")
    return dummy1.text.strip(), dummy2.text.strip(), kiper.text.strip(), mode.text.strip(), gameStatus.text.strip()

def reset(ser):
    if ser.isOpen() == False:
            ser.open()
    ser.reset_input_buffer()

def setMotor(ser,dki,dka,bki,bka) :
    if ser.isOpen() == False:
        ser.open()
    dki = dki + (dki * 0.3)
    dka = dka + (dka * 0)
    bki = bki + (bki * 0)
    bka = bka + (bka * 0.3) 
    ser.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))

def stop():
    setMotor(motor, 0,0,0,0)

def maju(speed, delay):
    setMotor(motor, -speed,speed,-speed,speed)
    sleep(delay)
    setMotor(motor, 50,-50,50,-50)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def mundur(speed, delay):
    setMotor(motor, speed,-speed,speed,-speed)
    sleep(delay)
    setMotor(motor, -50,50,-50,50)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def geserKiri(speed, delay):
    adj = speed*0.2
    setMotor(motor, speed,speed,-speed-adj,-speed)
    sleep(delay)
    setMotor(motor, -50,-50,50,50)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def geserKanan(speed, delay):
    setMotor(motor, -speed,-speed,speed,speed)
    sleep(delay)
    setMotor(motor, 50,50,-50,-50)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def putarKiri(speed, delay):
    setMotor(motor, speed,speed,speed,speed)
    sleep(delay) 
    setMotor(motor, -50,-50,-50,-50)
    sleep(0.05)
    setMotor(motor, 0,0,0,0)

def putarKanan(speed, delay):
    setMotor(motor, -speed,-speed,-speed,-speed)
    sleep(delay) 
    setMotor(motor, 50,50,50,50)
    sleep(0.05)
    setMotor(motor, 0,0,0,0)

def serongKiri(speed, delay):
    setMotor(motor, 0,speed,-speed,0)
    sleep(delay)
    setMotor(motor, 0,-50,50,0)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def serongKanan(speed, delay):
    setMotor(motor, -speed,0,0,speed)
    sleep(delay)
    setMotor(motor, 50,0,0,-50)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def mundurSerongKanan(speed, delay):
    setMotor(motor, 0,-speed,speed,0)
    sleep(delay)
    setMotor(motor, 0,50,-50,0)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def mundurSerongKiri(speed, delay):
    setMotor(motor, speed,0,0,-speed)
    sleep(delay)
    setMotor(motor, -50,0,0,50)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

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
    # === init tendang
    ser.reset_input_buffer()
    dribbling(ser, 0)
    sleep(0.5)
    dribbling(ser, 0)
    sleep(0.1)
    ser.write(b"TEND1\n")
    sleep(1)

def oper(ser):
    if ser.isOpen() == False:
        ser.open()
    # === init tendang
    ser.reset_input_buffer()
    dribbling(ser, 0)
    sleep(0.5)
    dribbling(ser, 0)
    sleep(0.1)
    ser.write(b"TEND2\n")
    sleep(1)

def getBallInfo():
    infoFile = open("ballColor.txt","r")
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

def getBallInfo2():
    infoFile = open("ballColor1.txt","r")
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

def getDummyInfo():
    infoFile = open("dummyColor.txt","r")
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
    print("==>> PUTAR DERAJAT")
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
                        speed = 35
                    else :
                        speed = -35
                        
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
    print("==>> ARAH BOLA DEPAN")
    # get center of the frame
    _, frame1 = FRONT_CAP.read()
    rows, cols, _ = frame1.shape
    cenX_frame1 = int(cols/2)
    cenY_frame1 = int(rows/2)

    # _, frame2 = OMNI_CAP.read()
    # rows1, cols1, _ = frame2.shape
    # cenX_frame2 = int(cols1/2)
    # cenY_frame2 = int(rows1/2)
    
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

    counter = 0
    dari = "kiri"
    second = 0
    startCount = 10
    count = startCount
    speed = 60
    state = "START"

    dribbling(db,1)
    db.flush()

    while(True):
        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
        if gameStatus == "RETRY":
            cv2.destroyAllWindows()
            break

        # print(gameStatus)

        if count <= 0:
            motor.close()
            count = startCount
        count -= 1
        #print(state)
        second += 1
        #print(second)
        for i in range(3):
            FRONT_CAP.grab()
            #OMNI_CAP.grab()
        ## read frame
        _, frame1 = FRONT_CAP.read()
        #_, frame2 = OMNI_CAP.read()
        
        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        #hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        #blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, objColor[6], 255, 0)
        #_, BALL_THRESH1 = cv2.threshold(BALL_MASK1, objColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        #ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)  

        pas = 0
        ada = 0  

        if db.isOpen() == False:
            db.open()
        db.reset_input_buffer()
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH" 

        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            if ball_area > 500:
                ada = 1
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2   
                print(cenX_ball)
                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenX_ball < 100  :
                    putarKiri(40, 0.2)
                    dari = "kanan"
                    print("PUTAR KANAN")
                    
                elif cenX_ball > 300  :
                    putarKanan(40, 0.2)
                    dari = "kiri"
                    print("PUTAR KIRI")
                    
                    
                elif cenX_ball < 180  :
                    putarKiri(40, 0.1)
                    dari = "kanan"
                    print("PUTAR KANAN")
                
                elif cenX_ball > 220 :
                    putarKiri(40, 0.1)
                    dari = "kiri"
                    print("PUTAR KIRI")
                else :
                    #pas = 1
                    setStatus(2, "READY")
                break

        if state == "FINISH": 
            setMotor(motor,0,0,0,0)
            motor.close()
            cv2.destroyAllWindows()
            break 

        # if ada == 0:
        #     setMotor(motor,0,0,0,0)     
        #     dari = ""
        #     counter += 1
        #     if counter > 10 and dari == "kanan":
        #         counter = 0
        #         dari = "kiri"
        #     elif counter > 10 and dari == "kiri":
        #         counter = 0
        #         dari = "kanan"

        #     if dari == "kiri":
        #         setMotor(motor,28,28,28,28)
        #         # sleep(0.1)
        #         # setMotor(motor,0,0,0,0)     
        #         # dari = ""
        #     else:
        #         setMotor(motor,-28,-28,-28,-28)
        #         # sleep(0.1)
        #         # setMotor(motor,0,0,0,0)
        #         # dari = "" 

        # displays
        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        #cv2.imshow("Kamra Atas", frame2)
        #cv2.moveWindow("Kamera Atas" ,0,0)
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            if motor.isOpen == False:
                motor.open()
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            #OMNI_CAP.release()
            cv2.destroyAllWindows()
            break
    return gameStatus

def arahRobotDepan():
    print("==>> ARAH ROBOT DEPAN")
    setStatus(2, "RUNNING")
    # get center of the frame
    _, frame1 = FRONT_CAP.read()
    rows, cols, _ = frame1.shape
    cenX_frame1 = int(cols/2)
    cenY_frame1 = int(rows/2)

    # _, frame2 = OMNI_CAP.read()
    # rows1, cols1, _ = frame2.shape
    # cenX_frame2 = int(cols1/2)
    # cenY_frame2 = int(rows1/2)
    
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

    dari = "kanan"
    second = 0
    startCount = 10
    count = startCount
    counter = 0
    speed = 60
    state = "START"

    dribbling(db,1)
    db.flush()

    setMotor(motor,0,0,0,0)

    while(True):
        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
        if gameStatus == "RETRY":
            cv2.destroyAllWindows()
            break

        if count <= 0:
            motor.close()
            count = startCount
        count -= 1
        #print(state)
        second += 1
        #print(second)
        for i in range(3):
            FRONT_CAP.grab()
            #OMNI_CAP.grab()
        ## read frame
        _, frame1 = FRONT_CAP.read()
        #_, frame2 = OMNI_CAP.read()
        
        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        #hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        #blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, objColor[6], 255, 0)
        #_, BALL_THRESH1 = cv2.threshold(BALL_MASK1, objColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        pas = 0
        ada = 0

        for ballContour in ballContours:
            ball_area = cv2.contourArea(ballContour)
            if ball_area > 500:
                ada = 1
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2   
                print(cenX_ball)
                # draw actual coordinate from segmentation
                cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenX_ball > 300  :
                    setMotor(motor,-28,-28,-28,-28)
                    dari = "kiri"
                    count = startCount

                elif cenX_ball > 0 and cenX_ball < 100  :
                    setMotor(motor,28,28,28,28)
                    dari = "kanan"
                    count = startCount
                    
                elif cenX_ball > 0 and cenX_ball < 200  :
                    setMotor(motor,35,35,35,35)
                    sleep(0.125)
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)
                    dari = "kanan"
                    count = startCount
                    print("PUTAR KANAN")
                
                elif cenX_ball > 210 :
                    setMotor(motor,-35,-35,-35,-35)
                    sleep(0.125)
                    setMotor(motor,0,0,0,0)
                    sleep(0.1)
                    dari = "kiri"
                    count = startCount
                    print("PUTAR KIRI")
                else :
                    pas = 1
                    state = "FINISH"
                    setStatus(2, "READY")
                break

        if state == "FINISH" and getStatus(1) == "READY": 
            setMotor(motor,0,0,0,0)
            motor.close()
            compass(db,0)
            cv2.destroyAllWindows()
            break   

        if ada == 0:
            counter += 1
            if counter > 15 and dari == "kanan":
                counter = 0
                dari = "kiri"
            elif counter > 15 and dari == "kiri":
                counter = 0
                dari = "kanan"

            if dari == "kiri":
                setMotor(motor,28,28,28,28)
                # sleep(0.1)
                # setMotor(motor,0,0,0,0)     
                # dari = ""
            else:
                setMotor(motor,-28,-28,-28,-28)
                # sleep(0.1)
                # setMotor(motor,0,0,0,0)
                # dari = ""
            #print(dari)

        # displays
        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame1, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        cv2.rectangle(frame1, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        cv2.imshow("Kamera Depan", frame1)
        cv2.moveWindow("Kamera Depan" ,20,20)
        
        # cv2.imshow("Kamra Atas", frame2)
        # cv2.moveWindow("Kamera Atas" ,0,0)
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            if motor.isOpen == False:
                motor.open()
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            #OMNI_CAP.release()
            cv2.destroyAllWindows()
            break
    return gameStatus

def arahKiper():
    # get center of the frame
    _, frame1 = FRONT_CAP.read()
    rows, cols, _ = frame1.shape
    cenX_frame1 = int(cols/2)
    cenY_frame1 = int(rows/2)

    # _, frame2 = OMNI_CAP.read()
    # rows1, cols1, _ = frame2.shape
    # cenX_frame2 = int(cols1/2)
    # cenY_frame2 = int(rows1/2)
    
    inner_left = cenX_frame1 - 100
    outer_left = cenX_frame1 - 250 
    inner_right = cenX_frame1 + 100
    outer_right = cenX_frame1 + 250
    inner_top = cenY_frame1 - 100
    outer_top = cenY_frame1 - 150
    inner_bottom = cenY_frame1 + 100
    outer_bottom = cenY_frame1 + 150

    # read magenta color
    objColor = getDummyInfo()
    lowerBall = np.array([objColor[0],objColor[1],objColor[2]])
    upperBall = np.array([objColor[3],objColor[4],objColor[5]])

    dari = ""
    second = 0
    startCount = 10
    count = startCount
    speed = 60
    state = "START"

    db.reset_input_buffer()
    dribbling(db,1)

    while(True):
        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
        
        if gameStatus == "RETRY":
            break

        if count <= 0:
            motor.close()
            count = startCount
        count -= 1
        #print(state)
        second += 1
        #print(second)
        for i in range(3):
            FRONT_CAP.grab()
            #OMNI_CAP.grab()
        ## read frame
        _, frame1 = FRONT_CAP.read()
        #_, frame2 = OMNI_CAP.read()
        
        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        #hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        #blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        #BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        _, BALL_THRESH = cv2.threshold(BALL_MASK, objColor[6], 255, 0)
        #_, BALL_THRESH1 = cv2.threshold(BALL_MASK1, objColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        #BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        # ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)
        #

        # if state == "FINISH":
        #     setMotor(motor,0,0,0,0)
        #     motor.close()
        #     cv2.destroyAllWindows()
        #     break    

        pas = 0
        ada = 0

        dummyContour = []
        dummyObj = None

        for ballContour in ballContours:
            dummyContour.append(ballContour)

        #print(len(dummyCountour))
        if len(dummyContour) > 1:
            dummyObj = dummyContour[1]
        elif len(dummyContour) > 0 and len(dummyContour) <= 1:
            dummyObj = dummyContour[0]

        if dummyObj is not None:  
            ada = 1
            (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(dummyObj)
            cv2.putText(frame1, "X: "+str(x_ball)+" Y: "+str(y_ball), (520, 20), font, 0.5, (0,0,255),2)
            cenX_ball = (x_ball+x_ball+w_ball)/2
            cenY_ball = (y_ball+y_ball+h_ball)/2   
            print(cenX_ball)
            # draw actual coordinate from segmentation
            cv2.circle(frame1, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
            cv2.line(frame1, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
            cv2.putText(frame1, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
            
            if cenX_ball < 100  :
                setMotor(motor,30,30,30,30)
                
            elif cenX_ball > 300  :
                setMotor(motor,-30,-30,-30,-30)
                
            elif cenX_ball < 180  :
                setMotor(motor,35,35,35,35)
                sleep(0.1)
                setMotor(motor,0,0,0,0)
                sleep(0.1)
                dari = "kanan"
                print("PUTAR KANAN")
            
            elif cenX_ball > 220 :
                setMotor(motor,-35,-35,-35,-35)
                sleep(0.1)
                setMotor(motor,0,0,0,0)
                sleep(0.1)
                dari = "kiri"
                print("PUTAR KIRI")
            else:
                setMotor(motor,0,0,0,0)
                state = "FINISH"

        if state == "FINISH": 
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
        
        # cv2.imshow("Kamra Atas", frame2)
        # cv2.moveWindow("Kamera Atas" ,0,0)
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            if motor.isOpen == False:
                motor.open()
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            FRONT_CAP.release()
            #OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def mulaiSerongKiri():
    print("==>> MULAI SERONG KIRI")
    # get center of the frame
    # _, frame1 = FRONT_CAP.read()
    # rows, cols, _ = frame1.shape
    # cenX_frame1 = int(cols/2)
    # cenY_frame1 = int(rows/2)

    _, frame2 = OMNI_CAP.read()
    rows1, cols1, _ = frame2.shape
    cenX_frame2 = int(cols1/2)
    cenY_frame2 = int(rows1/2)
    
    inner_left = cenX_frame2 - 120
    outer_left = cenX_frame2 - 170 
    inner_right = cenX_frame2 + 55
    outer_right = cenX_frame2 + 105
    inner_top = cenY_frame2 - 100
    outer_top = cenY_frame2 - 120
    inner_bottom = cenY_frame2 + 55
    outer_bottom = cenY_frame2 + 80

    # read magenta color
    objColor = getBallInfo2()
    lowerBall = np.array([objColor[0],objColor[1],objColor[2]])
    upperBall = np.array([objColor[3],objColor[4],objColor[5]])

    dari = ""

    startCount = 10
    count = startCount
    speed = 35
    state = "START"

    dribbling(db,1)
    #db.flush()
    dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()

    while(True):
        if gameStatus == "RETRY":
            cv2.destroyAllWindows()
            break
        if count <= 0:
            motor.close()
            dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
            count = startCount
        count -= 1
        #print(state)
        for i in range(3):
            #FRONT_CAP.grab()
            OMNI_CAP.grab()
        ## read frame
        #_, frame1 = FRONT_CAP.read()
        _, frame2 = OMNI_CAP.read()
        
        # convert frame from BGR to HSV
        #hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        #blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        #BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        #_, BALL_THRESH = cv2.threshold(BALL_MASK, objColor[6], 255, 0)
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, objColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        #BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        #ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        if db.isOpen() == False:
            db.open()
        reading = db.readline().decode('utf-8','ignore')
        # db.reset_input_buffer()
        if len(reading) > 0 :
            head = reading[0:5]
            print(head)
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"    

        # if stm.isOpen() == False:
        #     stm.open()
        # reading = stm.readline().decode('utf-8','ignore')
        # # db.reset_input_buffer()
        # if len(reading) > 0 :
        #     head = reading[0:5]
        #     print(head)
        #     if  head == "Dapat" :
        #         print("DAPAT BOLA")
        #         state = "FINISH"    

        pas = 0

        for ballContour in ballContours1:
            ball_area = cv2.contourArea(ballContour)
            if ball_area > 10:
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame2, "X: "+str(x_ball)+" Y: "+str(y_ball), (20, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2   
                #print("X: "+str(cenX_ball)+" Y: "+str(cenY_ball))
                # draw actual coordinate from segmentation
                cv2.circle(frame2, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame2, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame2, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenX_ball > 200:
                    setMotor(motor, 0,0,0,0)
                    #pas = 1
                    #state = "FINISH"
                    setStatus(2, "READY")
                break
        
        if state == "FINISH": 
            setMotor(motor,0,0,0,0)
            motor.close()
            cv2.destroyAllWindows()
            break        

        # displays
        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame2, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        #cv2.rectangle(frame2, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        #cv2.imshow("Kamera Depan", frame1)
        #cv2.moveWindow("Kamera Depan" ,20,20)
        
        cv2.imshow("Kamera Atas", frame2)
        cv2.moveWindow("Kamera Atas" ,0,0)
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            if motor.isOpen == False:
                motor.open()
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            #FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break
    return gameStatus

def tesLimit():
    print("==>> Tes Limit")
    # get center of the frame
    # _, frame1 = FRONT_CAP.read()
    # rows, cols, _ = frame1.shape
    # cenX_frame1 = int(cols/2)
    # cenY_frame1 = int(rows/2)

    _, frame2 = OMNI_CAP.read()
    rows1, cols1, _ = frame2.shape
    cenX_frame2 = int(cols1/2)
    cenY_frame2 = int(rows1/2)
    
    inner_left = cenX_frame2 - 120
    outer_left = cenX_frame2 - 170 
    inner_right = cenX_frame2 + 55
    outer_right = cenX_frame2 + 105
    inner_top = cenY_frame2 - 100
    outer_top = cenY_frame2 - 120
    inner_bottom = cenY_frame2 + 55
    outer_bottom = cenY_frame2 + 80

    # read magenta color
    objColor = getBallInfo2()
    lowerBall = np.array([objColor[0],objColor[1],objColor[2]])
    upperBall = np.array([objColor[3],objColor[4],objColor[5]])

    dari = ""
    second = 0
    startCount = 10
    count = startCount
    speed = 35
    state = "START"

    dribbling(db,1)
    #db.flush()

    while(True):
        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
        if gameStatus == "RETRY":
            cv2.destroyAllWindows()
            break

        if count <= 0:
            motor.close()
            count = startCount
        count -= 1
        #print(state)
        second += 1
        #print(second)
        for i in range(3):
            #FRONT_CAP.grab()
            OMNI_CAP.grab()
        ## read frame
        #_, frame1 = FRONT_CAP.read()
        _, frame2 = OMNI_CAP.read()
        
        # convert frame from BGR to HSV
        #hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        #blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        #BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        #_, BALL_THRESH = cv2.threshold(BALL_MASK, objColor[6], 255, 0)
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, objColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        #BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        #ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        if db.isOpen() == False:
            db.open()
        # db.reset_input_buffer()
        reading = db.readline().decode('utf-8','ignore')
        if len(reading) > 0 :
            head = reading[0:5]
            print(head)
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"    

        pas = 0

        for ballContour in ballContours1:
            ball_area = cv2.contourArea(ballContour)
            if ball_area > 10:
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame2, "X: "+str(x_ball)+" Y: "+str(y_ball), (20, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2   
                #print("X: "+str(cenX_ball)+" Y: "+str(cenY_ball))
                # draw actual coordinate from segmentation
                cv2.circle(frame2, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame2, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame2, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                # if cenX_ball < 170 and cenY_ball < 100 :
                #     setMotor(motor,-speed,-speed,-speed,-speed)
                    
                # elif cenX_ball > 280 and cenY_ball < 100 :
                #     setMotor(motor,speed,speed,speed,speed)
                    
                # elif cenX_ball < 195 and cenY_ball < 100 :
                #     putarKanan(speed, 0.3)
                
                # elif cenX_ball > 220 and cenY_ball < 100 :
                #     putarKiri(speed, 0.3)
                # else :
                #     if cenY_ball < 90:
                #         setMotor(motor,-70,70,-70,70)
                #     else:
                #         setMotor(motor,50,-50,50,-50)
                #         sleep(0.1)
                #         setMotor(motor, 0,0,0,0)
                #         #pas = 1
                #         #state = "FINISH"
                #         setStatus(2, "READY")

                # if cenX_ball > 230 and cenX_ball < 235 and cenY_ball > 80:
                #     setMotor(motor, 0,0,0,0)
                #     #pas = 1
                #     #state = "FINISH"
                #     setStatus(2, "READY")
                break
        
        if state == "FINISH": 
            setMotor(motor,0,0,0,0)
            motor.close()
            cv2.destroyAllWindows()
            break        

        # displays
        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame2, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        #cv2.rectangle(frame2, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        #cv2.imshow("Kamera Depan", frame1)
        #cv2.moveWindow("Kamera Depan" ,20,20)
        
        cv2.imshow("Kamera Atas", frame2)
        cv2.moveWindow("Kamera Atas" ,0,0)
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            if motor.isOpen == False:
                motor.open()
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            #FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break

def lurusBolaAtas():
    print("==>> LURUS BOLA ATAS")
    # get center of the frame
    # _, frame1 = FRONT_CAP.read()
    # rows, cols, _ = frame1.shape
    # cenX_frame1 = int(cols/2)
    # cenY_frame1 = int(rows/2)

    _, frame2 = OMNI_CAP.read()
    rows1, cols1, _ = frame2.shape
    cenX_frame2 = int(cols1/2)
    cenY_frame2 = int(rows1/2)
    
    inner_left = cenX_frame2 - 120
    outer_left = cenX_frame2 - 170 
    inner_right = cenX_frame2 + 55
    outer_right = cenX_frame2 + 105
    inner_top = cenY_frame2 - 100
    outer_top = cenY_frame2 - 120
    inner_bottom = cenY_frame2 + 55
    outer_bottom = cenY_frame2 + 80

    # read magenta color
    objColor = getBallInfo2()
    lowerBall = np.array([objColor[0],objColor[1],objColor[2]])
    upperBall = np.array([objColor[3],objColor[4],objColor[5]])

    dari = ""

    startCount = 10
    count = startCount
    speed = 35
    state = "START"

    dribbling(db,1)
    #db.flush()
    dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()

    while(True):
        if gameStatus == "RETRY":
            cv2.destroyAllWindows()
            break
        if count <= 0:
            motor.close()
            dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()
            count = startCount
        count -= 1
        #print(state)
        for i in range(3):
            #FRONT_CAP.grab()
            OMNI_CAP.grab()
        ## read frame
        #_, frame1 = FRONT_CAP.read()
        _, frame2 = OMNI_CAP.read()
        
        # convert frame from BGR to HSV
        #hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        #blur = cv2.medianBlur(hsv, 5)
        blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        #BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)
        BALL_MASK1 = cv2.inRange(blur1, lowerBall, upperBall)

        # convert to black and white image
        #_, BALL_THRESH = cv2.threshold(BALL_MASK, objColor[6], 255, 0)
        _, BALL_THRESH1 = cv2.threshold(BALL_MASK1, objColor[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        #BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)
        BALL_MORPH1 = cv2.morphologyEx(BALL_THRESH1, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        #ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)
        
        ballContours1, _ = cv2.findContours(BALL_MORPH1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ballContours1 = sorted(ballContours1, key=lambda x:cv2.contourArea(x), reverse=True)

        if db.isOpen() == False:
            db.open()
        reading = db.readline().decode('utf-8','ignore')
        # db.reset_input_buffer()
        if len(reading) > 0 :
            head = reading[0:5]
            print(head)
            if  head == "Dapat" :
                print("DAPAT BOLA")
                state = "FINISH"    

        # if stm.isOpen() == False:
        #     stm.open()
        # reading = stm.readline().decode('utf-8','ignore')
        # # db.reset_input_buffer()
        # if len(reading) > 0 :
        #     head = reading[0:5]
        #     print(head)
        #     if  head == "Dapat" :
        #         print("DAPAT BOLA")
        #         state = "FINISH"    

        pas = 0

        for ballContour in ballContours1:
            ball_area = cv2.contourArea(ballContour)
            if ball_area > 10:
                (x_ball, y_ball, w_ball, h_ball) = cv2.boundingRect(ballContour)
                cv2.putText(frame2, "X: "+str(x_ball)+" Y: "+str(y_ball), (20, 20), font, 0.5, (0,0,255),2)
                cenX_ball = (x_ball+x_ball+w_ball)/2
                cenY_ball = (y_ball+y_ball+h_ball)/2   
                #print("X: "+str(cenX_ball)+" Y: "+str(cenY_ball))
                # draw actual coordinate from segmentation
                cv2.circle(frame2, (int(cenX_ball), int(cenY_ball)), 20, [0,255,0], 2, 8)
                cv2.line(frame2, (int(cenX_ball), int(cenY_ball + 20)), (int(cenX_ball + 50), int(cenY_ball + 20)), [0,255,0], 2, 8)
                cv2.putText(frame2, "Actual", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
                
                if cenX_ball < 170 and cenY_ball < 100 :
                    setMotor(motor,-speed,-speed,-speed,-speed)
                    
                elif cenX_ball > 280 and cenY_ball < 100 :
                    setMotor(motor,speed,speed,speed,speed)
                    
                elif cenX_ball < 195 and cenY_ball < 100 :
                    putarKanan(speed, 0.2)
                
                elif cenX_ball > 220 and cenY_ball < 100 :
                    putarKiri(speed, 0.2)
                else :
                    if cenY_ball < 115:
                        setMotor(motor,-50,50,-50,50)
                    # elif cenY_ball < 90 and cenX_ball < 230 or cenX_ball > 235:
                    #     setMotor(motor,50,-50,50,-50)

                if cenX_ball > 230 and cenX_ball < 235 and cenY_ball > 80:
                    setMotor(motor, 0,0,0,0)
                    #pas = 1
                    #state = "FINISH"
                    setStatus(2, "READY")
                break
        
        if state == "FINISH": 
            setMotor(motor,0,0,0,0)
            motor.close()
            cv2.destroyAllWindows()
            break        

        # displays
        ## uncomment this to show center area of the frame 1
        cv2.rectangle(frame2, (inner_left, inner_top), (inner_right, inner_bottom), (0,255,0), 2)
        #cv2.rectangle(frame2, (outer_left, outer_top), (outer_right, outer_bottom), (0,255,255), 2)
        #cv2.rectangle(frame2, (xAwal, yAwal), (xAkhir, yAkhir), (0,255,0), 2)

        #cv2.imshow("Kamera Depan", frame1)
        #cv2.moveWindow("Kamera Depan" ,20,20)
        
        cv2.imshow("Kamera Atas", frame2)
        cv2.moveWindow("Kamera Atas" ,0,0)
        
        #print(ballColor)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            if motor.isOpen == False:
                motor.open()
            motor.write(b"#M|STP|0\n")
            db.write(b"DB OFF")
            #FRONT_CAP.release()
            OMNI_CAP.release()
            cv2.destroyAllWindows()
            break
    return gameStatus

def main():
    # serial motor driver
    motor = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)

    # serial dribble
    db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

    # stm = serial.Serial(port='/dev/ttyACM2', baudrate=9600, timeout=1)
    # # serial OpenCM
    # cm = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=1)
    # cm.close()

    # initialize camera
    _, frame1 = FRONT_CAP.read()
    _, frame2 = OMNI_CAP.read()

    phase = 0

    while True:
        dummy1, dummy2, kiper, mode, gameStatus = getGameInfo()

        reset(db)
        reset(motor)

        if dummy1 == "1" and dummy2 == "7":
            if mode == "KICKOFF KANAN":
                while gameStatus == "START":
                    setStatus(2, "RUNNING")
                    maju(90, 0.8)
                    serongKiri(180, 1.5)
                    putarKiri(90, 0.2)
                    gameStatus = arahBolaDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarKanan(90, 0.3)
                        sleep(0.5)
                        mundurSerongKanan(180, 1.5)
                        mundur(90, 0.8)
                        #setGame("STOP")
                        break
                    setStatus(2, "RUNNING")
                    putarKanan(90, 0.3)
                    gameStatus = arahRobotDepan()
                    if gameStatus == "RETRY":
                        putarDerajat(87,1)
                        mundurSerongKanan(180, 1.5)
                        mundur(90, 0.5)
                        dribbling(db,0)
                        #setGame("STOP")
                        break
                    while getStatus(1) != "READY":
                        setMotor(motor, 0,0,0,0)
                        if getStatus(1) == "READY":
                            break
                    oper(db)
                    setStatus(2, "RUNNING")
                    geserKanan(90, 1.5)
                    putarKiri(90, 0.2)
                    arahBolaDepan()
                    if gameStatus == "RETRY":
                        putarDerajat(87,1)
                        mundurSerongKanan(180, 1.5)
                        mundur(90, 0.5)
                        dribbling(db,0)
                        #setGame("STOP")
                        break
                    setGame("STOP")
                    setStatus(2, "RUNNING")
                    putarKanan(80, 0.8)
                    arahKiper()
                    if kiper == "3":
                        putarKiri(60, 0.2)
                    else:
                        putarKanan(60, 0.2)
                    tendang(db)
                    putarKiri(90, 0.3)
                    mundur(180, 1)
                    setStatus(2, "IDLE")
                    break

            elif mode == "KICKOFF KIRI":
                while gameStatus == "START":
                    serongKiri(170, 3)
                    gameStatus = lurusBolaAtas()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarDerajat(86,0)
                        mundur(90, 1)
                        mundurSerongKanan(180, 2)
                        #setGame("STOP")
                        break
                    mundur(50, 0.5)
                    putarKanan(90, 0.2)
                    gameStatus = arahRobotDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarDerajat(86,0)
                        mundur(90, 1)
                        mundurSerongKanan(180, 2)
                        #setGame("STOP")
                        break
                    while getStatus(1) != "READY":
                        setMotor(motor, 0,0,0,0)
                        if getStatus(1) == "READY":
                            break
                    oper(db)
                    setStatus(2, "RUNNING")
                    serongKanan(180, 1.2)
                    putarKiri(90, 0.2)
                    gameStatus = arahBolaDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarDerajat(86,0)
                        mundurSerongKanan(180, 1.5)
                        mundur(90, 1.5)
                        #setGame("STOP")
                        break
                    sleep(2)
                    putarKanan(90, 0.2)
                    gameStatus = arahRobotDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarKiri(90, 0.3)
                        mundurSerongKanan(180, 1)
                        mundur(90, 3)
                        #setGame("STOP")
                        break
                    while getStatus(1) != "READY":
                        setMotor(motor, 0,0,0,0)
                        if getStatus(1) == "READY":
                            break
                    oper(db)
                    dribbling(db,0)
                    putarKiri(90, 0.3)
                    mundur(90, 1.5)
                    mundurSerongKanan(180, 1.5)
                    setGame("STOP")
                    break
                
            elif mode == "KICKOFF CORNER":
                dribbling(db,1)
                lurusBolaAtas()
                dribbling(db,0)
                break
            
        elif dummy1 == "2" and dummy2 == "7":
            if mode == "KICKOFF KANAN":
                while gameStatus == "START":
                    setStatus(2, "RUNNING")
                    maju(90, 0.8)
                    serongKiri(180, 1.5)
                    putarKiri(90, 0.2)
                    gameStatus = arahBolaDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        sleep(0.1)
                        dribbling(db,0)
                        putarDerajat(87,1)
                        mundurSerongKanan(150, 1.5)
                        mundur(90, 0.8)
                        setStatus(2, "RETRY")
                        dribbling(db,0)
                        sleep(0.1)
                        dribbling(db,0)
                        #setGame("STOP")
                        break
                    setStatus(2, "RUNNING")
                    putarKanan(90, 0.3)
                    gameStatus = arahRobotDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        sleep(0.1)
                        dribbling(db,0)
                        putarDerajat(87,1)
                        mundurSerongKanan(150, 1.5)
                        mundur(90, 0.5)
                        setStatus(2, "RETRY")
                        #setGame("STOP")
                        break
                    while getStatus(1) != "READY":
                        setMotor(motor, 0,0,0,0)
                        if getStatus(1) == "READY":
                            break
                    oper(db)
                    setStatus(2, "RUNNING")
                    geserKanan(90, 1.2)
                    putarKiri(90, 0.2)
                    arahBolaDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        sleep(0.1)
                        dribbling(db,0)
                        putarDerajat(87,1)
                        mundurSerongKanan(180, 1.5)
                        mundur(90, 0.5)
                        setStatus(2, "RETRY")
                        #setGame("STOP")
                        break
                    setGame("STOP")
                    setStatus(2, "RUNNING")
                    putarKanan(80, 0.65)
                    arahKiper()
                    if kiper == "3":
                        putarKiri(60, 0.2)
                    else:
                        putarKanan(60, 0.2)
                    tendang(db)
                    putarKiri(90, 0.3)
                    mundur(140, 1.5)
                    setStatus(2, "IDLE")
                    break

            elif mode == "KICKOFF KIRI":
                while gameStatus == "START":
                    # serongKiri(170, 3)
                    gameStatus = mulaiSerongKiri()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarDerajat(86,0)
                        mundur(90, 1)
                        mundurSerongKanan(180, 2)
                        #setGame("STOP")
                        break
                    gameStatus = lurusBolaAtas()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarDerajat(86,0)
                        mundur(90, 1)
                        mundurSerongKanan(180, 2)
                        #setGame("STOP")
                        break
                    mundur(50, 0.5)
                    putarKanan(90, 0.2)
                    gameStatus = arahRobotDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarDerajat(86,0)
                        mundur(90, 1)
                        mundurSerongKanan(180, 2)
                        #setGame("STOP")
                        break
                    while getStatus(1) != "READY":
                        setMotor(motor, 0,0,0,0)
                        if getStatus(1) == "READY":
                            break
                    oper(db)
                    setStatus(2, "RUNNING")
                    serongKanan(180, 1.2)
                    putarKiri(90, 0.2)
                    gameStatus = arahBolaDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarDerajat(86,0)
                        mundurSerongKanan(180, 1.5)
                        mundur(90, 1.2)
                        #setGame("STOP")
                        break
                    sleep(2)
                    putarKanan(90, 0.2)
                    gameStatus = arahRobotDepan()
                    if gameStatus == "RETRY":
                        dribbling(db,0)
                        putarKiri(90, 0.3)
                        mundurSerongKanan(180, 1)
                        mundur(90, 3)
                        #setGame("STOP")
                        break
                    while getStatus(1) != "READY":
                        setMotor(motor, 0,0,0,0)
                        if getStatus(1) == "READY":
                            break
                    oper(db)
                    dribbling(db,0)
                    putarKiri(90, 0.3)
                    mundur(90, 1.5)
                    mundurSerongKanan(180, 1.5)
                    setGame("STOP")
                    break

        elif dummy1 == "3" and dummy2 == "7":
            pass
        elif dummy1 == "4" and dummy2 == "7":
            pass
        elif dummy1 == "5" and dummy2 == "7":
            pass
        elif dummy1 == "6" and dummy2 == "7":
            pass
        elif dummy1 == "1" and dummy2 == "8":
            pass
        elif dummy1 == "2" and dummy2 == "8":
            pass
        elif dummy1 == "3" and dummy2 == "8":
            pass
        elif dummy1 == "4" and dummy2 == "8":
            pass
        elif dummy1 == "5" and dummy2 == "8":
            pass
        elif dummy1 == "6" and dummy2 == "8":
            pass

if __name__ == '__main__':
    # execute main program
    main()
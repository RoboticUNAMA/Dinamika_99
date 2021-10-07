import cv2, numpy as np

from server import *
from navigasi import *

# set imshow
DISPLAY         = True

# konfigurasi
FRONT_CAM       = 1
OMNI_CAM        = 0
RESOLUTION      = [300,200]
FONT            = cv2.FONT_HERSHEY_SIMPLEX

# capture object
FRONT_CAP = cv2.VideoCapture(FRONT_CAM)
FRONT_CAP.set(cv2.CAP_PROP_EXPOSURE, -5)
OMNI_CAP = cv2.VideoCapture(OMNI_CAM)
OMNI_CAP.set(cv2.CAP_PROP_EXPOSURE, -5)

# set frame size
FRONT_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
FRONT_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
OMNI_CAP.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
OMNI_CAP.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])

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

def arahBolaKameraAtas():
    _, frame = OMNI_CAP.read()
    rows, cols, _ = frame.shape
    cenX_frame = int(cols/2)
    cenY_frame = int(rows/2)

    info = getBallInfo()
    lower = np.array([info[0],info[1],info[2]])
    upper = np.array([info[3],info[4],info[5]])

    ada = False
    count = 0

    while(True):
        # skip frame
        for i in range(0):
            OMNI_CAP.grab()

        # read frame
        _, frame = OMNI_CAP.read()
        
        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        #blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        mask = cv2.inRange(blur, lower, upper)

        # convert to black and white image
        _, thresh = cv2.threshold(mask, info[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        contours, _ = cv2.findContours(morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)

        for c in contours:
            ada = True
            area = cv2.contourArea(c)
            if area > 10:
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.putText(frame, "X: "+str(x)+" Y: "+str(y), (10,20), FONT, 0.5, (0,0,255),2)
                cenX = (x+x+w)/2
                cenY = (y+y+h)/2
                # cv2.circle(frame, (int(cenX), int(cenY)), 10, [0,255,0], 2, 8)
                cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), [0,255,0], 2)
                cv2.line(frame, (int(x+w), int(y+h)), (int((x+w) + 25), int(cenY)), [0,255,0], 2, 8)
                cv2.putText(frame, "Bola", (int(x+w + 25), int(cenY)), FONT, 0.5, [0,255,0], 2)

                if cenX < cenX_frame-100 and cenY > cenY_frame+50:
                    print("KANAN BANYAK")
                    putarKanan(motor, 120)
                elif cenX < cenX_frame-20 and cenY > cenY_frame-50:
                    print("KANAN DIKIT")
                    putarKanan(motor, 40)
                elif cenX > cenX_frame+100 and cenY > cenY_frame+50:
                    print("KIRI BANYAK")
                    putarKiri(motor, 120)
                elif cenX > cenX_frame+20 and cenY > cenY_frame-50:
                    print("KIRI DIKIT")
                    putarKiri(motor, 40)
                elif cenX >= cenX_frame-20 and cenX <= cenX_frame+20 and cenY <= cenY_frame-50:
                    print("MAJU")
                    stop(motor)
                    cv2.destroyAllWindows()
                    arahBolaKameraDepan()
                break

        ada = False

        if DISPLAY == True:
            cv2.imshow("Arah Bola Kamera Atas", frame)
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            FRONT_CAP.release()
            cv2.destroyAllWindows()
            break

def arahBolaKameraDepan():
    _, frame = FRONT_CAP.read()
    rows, cols, _ = frame.shape
    cenX_frame = int(cols/2)
    cenY_frame = int(rows/2)

    info = getBallInfo()
    lower = np.array([info[0],info[1],info[2]])
    upper = np.array([info[3],info[4],info[5]])

    ada = False
    count = 0
    state = ""

    while(True):
        if ada == False:
            count += 1
            print("Pindah kamera atas dalam:",count)
            if count >= 30:
                count = 0
                cv2.destroyAllWindows()
                arahBolaKameraAtas()

        # skip frame
        for i in range(0):
            FRONT_CAP.grab()

        # read frame
        _, frame = FRONT_CAP.read()
        
        # convert frame from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #hsv1 = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)

        # blur the frame
        blur = cv2.medianBlur(hsv, 5)
        #blur1 = cv2.medianBlur(hsv1, 5)

        # create a mask from blurred frame
        mask = cv2.inRange(blur, lower, upper)

        # convert to black and white image
        _, thresh = cv2.threshold(mask, info[6], 255, 0)

        # refine the image using morphological transformation
        kernal = np.ones((5,5), np.uint8)
        morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernal, iterations = 2)

        # find contours
        contours, _ = cv2.findContours(morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)

        for c in contours:
            area = cv2.contourArea(c)
            if area > 500:
                ada = True
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.putText(frame, "X: "+str(x)+" Y: "+str(y), (10,20), FONT, 0.5, (0,0,255),2)
                cenX = (x+x+w)/2
                cenY = (y+y+h)/2
                # cv2.circle(frame, (int(cenX), int(cenY)), 10, [0,255,0], 2, 8)
                cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), [0,255,0], 2)
                cv2.line(frame, (int(x+w), int(y+h)), (int((x+w) + 25), int(cenY)), [0,255,0], 2, 8)
                cv2.putText(frame, "Bola", (int(x+w + 25), int(cenY)), FONT, 0.5, [0,255,0], 2)

                # if db.isOpen() == False:
                #     db.open()
                # db.reset_input_buffer()
                reading = db.readline().decode('utf-8','ignore')
                if len(reading) > 0 :
                    head = reading[0:5]
                    print(head)
                    if head == "Dapat" :
                        print("DAPAT BOLA")
                        state = "FINISH" 
                        stop(motor)

                if cenX < cenX_frame-100:
                    print("KIRI BANYAK")
                    putarKiri(motor, 120)
                elif cenX < cenX_frame-20:
                    print("KIRI DIKIT")
                    putarKiri(motor, 40)
                elif cenX > cenX_frame+100:
                    print("KANAN BANYAK")
                    putarKanan(motor, 120)
                elif cenX > cenX_frame+20:
                    print("KANAN DIKIT")
                    putarKanan(motor, 40)
                elif cenX > cenX_frame-20 and cenX < cenX_frame+20:
                    print("MAJU")
                    maju(motor, 120)
                else:
                    print("STOP")
                    stop(motor)
                break

        if ada == True:
            count = 0
        ada = False

        if DISPLAY == True:
            cv2.imshow("Arah Bola Kamera Depan", frame)

        if state == "FINISH":
            break
        
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            FRONT_CAP.release()
            cv2.destroyAllWindows()
            break
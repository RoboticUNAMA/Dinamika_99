import cv2
import numpy as np

def nothing(x):
    #print(x)
    pass

def getBallInfo():
    infoFile = open("ballColor.txt","r")
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

def setBallInfo(LOW_H, LOW_S, LOW_V, UPP_H, UPP_S, UPP_V, TH):
    value = str(LOW_H)+"\n"+str(LOW_S)+"\n"+str(LOW_V)+"\n"+str(UPP_H)+"\n"+str(UPP_S)+"\n"+str(UPP_V)+"\n"+str(TH)
    userInput = input("Simpan data warna bola? (y/n): ")
    if userInput == "y" or userInput == "Y":
        infoFile = open("ballColor.txt", "w")
        infoFile.write(value)
        print("Data warna bola berhasil di simpan kedalam file ballColor.txt")
        infoFile.close()
    else:
        print("Data warna bola tidak di simpan")

FRONT_CAM = 0
OMNI_CAM = 1

window_name = 'Ball Calibration'
cv2.namedWindow(window_name)
cv2.createTrackbar('L_H', window_name, 0, 255, nothing)
cv2.createTrackbar('L_S', window_name, 0, 255, nothing)
cv2.createTrackbar('L_V', window_name, 0, 255, nothing)
cv2.createTrackbar('U_H', window_name, 0, 255, nothing)
cv2.createTrackbar('U_S', window_name, 0, 255, nothing)
cv2.createTrackbar('U_V', window_name, 0, 255, nothing)
cv2.createTrackbar('Threshold', window_name, 0, 255, nothing)

ballColor = getBallInfo()

cv2.setTrackbarPos('L_H', window_name, ballColor[0])
cv2.setTrackbarPos('L_S', window_name, ballColor[1])
cv2.setTrackbarPos('L_V', window_name, ballColor[2])
cv2.setTrackbarPos('U_H', window_name, ballColor[3])
cv2.setTrackbarPos('U_S', window_name, ballColor[4])
cv2.setTrackbarPos('U_V', window_name, ballColor[5])
cv2.setTrackbarPos('Threshold', window_name, ballColor[6])

# Define font
font = cv2.FONT_HERSHEY_SIMPLEX

# Create opencv video capture object
cap = cv2.VideoCapture(FRONT_CAM)

# Set frame size
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

# Get center of the frame
_, frame = cap.read()
rows, cols, _ = frame.shape
cenX_frame = int(cols/2)
cenY_frame = int(rows/2)

while True:
    # Read frame
    _, frame = cap.read()

    LOW_H = cv2.getTrackbarPos('L_H', window_name)
    LOW_S = cv2.getTrackbarPos('L_S', window_name)
    LOW_V = cv2.getTrackbarPos('L_V', window_name)
    UPP_H = cv2.getTrackbarPos('U_H', window_name)
    UPP_S = cv2.getTrackbarPos('U_S', window_name)
    UPP_V = cv2.getTrackbarPos('U_V', window_name)
    TH = cv2.getTrackbarPos('Threshold', window_name)

    lowerBall = np.array([LOW_H, LOW_S, LOW_V])
    upperBall = np.array([UPP_H, UPP_S, UPP_V])

    # Convert frame from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Blur the frame
    blur = cv2.medianBlur(hsv, 5)

    # Create a mask from blurred frame
    mask = cv2.inRange(blur, lowerBall, upperBall)

    # Convert to black and white image
    _, thresh = cv2.threshold(mask, TH, 255, 0)

    # Refine the image using morphological transformation
    kernal = np.ones((5,5), np.uint8)
    morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernal, iterations=2)

    # Find contours
    contours, _ = cv2.findContours(morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)


    cv2.imshow(window_name, morph)
    #cv2.imshow("Frame", frame)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        cap.release()
        cv2.destroyAllWindows()
        setBallInfo(LOW_H, LOW_S, LOW_V, UPP_H, UPP_S, UPP_V, TH)
        break
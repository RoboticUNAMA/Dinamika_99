import cv2
import numpy as np

img = cv2.imread('gawang.jpg', cv2.IMREAD_COLOR)
img_show = cv2.resize(img, (500, 400))

font = cv2.FONT_HERSHEY_SIMPLEX

def getGoalInfo():
    infoFile = open('goalColor.txt','r')
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

def getBallInfo():
    infoFile = open('ballColor.txt','r')
    info = []
    for i in infoFile:
        info.append(int(i))
    return info

goalColor = getGoalInfo()
ballColor = getBallInfo()

while(True):
    frame = img_show

    lowerGoal = np.array([
        goalColor[0],
        goalColor[1],
        goalColor[2]
        ])
    
    upperGoal = np.array([
        goalColor[3],
        goalColor[4],
        goalColor[5]
        ])

    lowerBall = np.array([
        ballColor[0],
        ballColor[1],
        ballColor[2]
        ])
    
    upperBall = np.array([
        ballColor[3],
        ballColor[4],
        ballColor[5]
        ])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    blur = cv2.medianBlur(hsv, 5)

    GOAL_MASK = cv2.inRange(blur, lowerGoal, upperGoal)
    BALL_MASK = cv2.inRange(blur, lowerBall, upperBall)

    _, GOAL_THRESH = cv2.threshold(GOAL_MASK, goalColor[6], 255, 0)
    _, BALL_THRESH = cv2.threshold(BALL_MASK, ballColor[6], 255, 0)
    kernal = np.ones((5,5), np.uint8)

    GOAL_MORPH = cv2.morphologyEx(GOAL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)

    BALL_MORPH = cv2.morphologyEx(BALL_THRESH, cv2.MORPH_CLOSE, kernal, iterations = 2)

    goalContours, _ = cv2.findContours(GOAL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    goalContours = sorted(goalContours, key=lambda x:cv2.contourArea(x), reverse=True)
    
    ballContours, _ = cv2.findContours(BALL_MORPH, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    ballContours = sorted(ballContours, key=lambda x:cv2.contourArea(x), reverse=True)

    for goalContour in goalContours:
        goal_area = cv2.contourArea(goalContour)
        if goal_area > 5000:
            x_goal, y_goal, w_goal, h_goal = cv2.boundingRect(goalContour)
            cv2.putText(frame, "Gawang", (x_goal, y_goal), font, 0.5, [255,0,0], 2)
            cv2.rectangle(frame,(x_goal, y_goal), (x_goal+w_goal, y_goal+h_goal), (255,0,0),2)
            break
   
    for ballContour in ballContours:
        ball_area = cv2.contourArea(ballContour)
        if ball_area > 500:
            x_ball, y_ball, w_ball, h_ball = cv2.boundingRect(ballContour)
            cenX_ball = (x_ball+x_ball+w_ball)/2
            cenY_ball = (y_ball+y_ball+h_ball)/2
            cv2.circle(frame, (int(cenX_ball), int(cenY_ball)), 30, (0,255,0), 2)
            cv2.putText(frame, "Bola", (int(cenX_ball + 50), int(cenY_ball + 20)), font, 0.5, [0,255,0], 2)
            break

    cv2.imshow('Frame', frame)        

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        cv2.destroyAllWindows()
        break

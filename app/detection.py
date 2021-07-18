import cv2
import numpy as np

class Camera:
    def __init__(self, src, fileColor):
        self.color = fileColor
        self.cap = cv2.VideoCapture(src)  # Prepare the camera...
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)
        print("Camera warming up ...")
        self.ret, self.frame = self.cap.read()
        self.height, self.width, _ = self.frame.shape
        if self.ret:  # frame captures without errors...
            pass
    
    def display(self, title, text, color, draw):
        self.ret, self.frame = self.cap.read()
        self.height, self.width, _ = self.frame.shape
        self.title = title
        if self.ret:  # frame captures without errors...
            pass        
        # draw 0 = circle, draw 1 = rectangle
        if draw == 0 and self.cenX > 0 and self.cenY > 0:
            cv2.circle(self.frame, (int(self.cenX), int(self.cenY)), 20, color, 2, 8)
            cv2.line(self.frame, (int(self.cenX), int(self.cenY + 20)), (int(self.cenX + 50), int(self.cenY + 20)), color, 2, 8)
            cv2.putText(self.frame, text, (int(self.cenX + 50), int(self.cenY + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        elif draw == 1 and self.cenX > 0 and self.cenY > 0:
            cv2.rectangle(self.frame, (self.x, self.y), (self.w, self.h), color, 2)
            cv2.putText(self.frame, text, (int(self.cenX + 50), int(self.cenY + 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow(self.title, self.frame)
        cv2.waitKey(1)

    def get_center_frame(self):
        self.centerX = int(self.width/2)
        self.centerY = int(self.height/2)
        return self.centerX, self.centerY

    def release(self):
        self.cap.release()

    def get_object(self, min_area):
        self.s = open(self.color, 'r')

        self.n = []
        for i in self.s:
            self.n.append(int(i))
        self.lower = np.array([self.n[0],self.n[1],self.n[2]])
        self.upper = np.array([self.n[3],self.n[4],self.n[5]])
        self.threshold = self.n[6]

        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.blur = cv2.medianBlur(self.hsv, 5)
        self.mask = cv2.inRange(self.blur, self.lower, self.upper)
        _, self.thresh = cv2.threshold(self.mask, self.threshold, 255, 0)
        self.kernal = np.ones((5,5), np.uint8)
        self.morph = cv2.morphologyEx(self.thresh, cv2.MORPH_CLOSE, self.kernal, iterations = 2)

        self.contours, _ = cv2.findContours(self.morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.contours = sorted(self.contours, key=lambda x:cv2.contourArea(x), reverse=True)

        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0
        self.cenX = 0
        self.cenY = 0
        self.area = 0

        for c in self.contours:
            self.area = cv2.contourArea(c)
            if self.area > min_area:
                self.x, self.y, self.w, self.h = cv2.boundingRect(c)
                self.cenX = (self.x*2+self.w)/2
                self.cenY = (self.y*2+self.h)/2
                break
        return self.area, self.x, self.y, self.w, self.h, self.cenX, self.cenY

def main():
    cam = Camera(2, 'ballColor.txt')
    while True:
        area, x, y, w, h, cenX, cenY = cam.get_object(500)
        #print(area, x, y, w, h, cenX, cenY)
        cam.display('Frame', 'Bola', [0,255,0], 0)

if __name__ == "__main__":
    # execute main program
    main()

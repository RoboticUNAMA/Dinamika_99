from detection import Camera
from navigation import Serial

def main():
    green = [0,255,0]
    cam1 = Camera(1, 'ballColor.txt')
    cam2 = Camera(0, 'ballColor1.txt')
    s = Serial('/dev/ttyACM0', '/dev/ttyUSB0')
    s.close(1)
    s.close(2)
    while True:
        area1, x1, y1, w1, h1, cenX1, cenY1 = cam1.get_object(500)
        area2, x2, y2, w2, h2, cenX2, cenY2 = cam2.get_object(10)
        cam1.display("Front Cam", "Bola", green, 0)
        cam2.display("Omni Cam", "Bola", green, 0)
        if cenX1 > 0:
            s.open(1)
            s.maju(100)
            s.close(1)

if __name__ == '__main__':
    # execute main program
    main()

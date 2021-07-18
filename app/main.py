from detection import *

green = [0,255,0]
camera = Detection(2, 'ballColor.txt')
while True:
    area, x, y, w, h, cenX, cenY = camera.get_object(500)
    camera.display("Frame", "Bola", green, 0)

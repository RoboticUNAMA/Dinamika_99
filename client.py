import socket, cv2, pickle, struct, numpy as np, serial, requests
from time import sleep

HEADER = 64
FORMAT = 'utf-8'
PORT = 9090
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

# serial motor driver
motor = serial.Serial(
    port='/dev/ttyUSB1', 
    baudrate=9600, 
    timeout=1
    )
# serial dribble
db = serial.Serial(
    port='/dev/ttyUSB0', 
    baudrate=9600, 
    timeout=1
    )


#serial OpeCM
sr = serial.Serial(
    port='/dev/ttyACM0', 
    baudrate=9600, 
    timeout=1
    )
sr.close()

def send(msg):
    message = msg.encode(FORMAT)
    msg_length = len(message)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b' ' * (HEADER - len(send_length))
    client.send(send_length)
    client.send(message)

def setMotor(ser,dki,dka,bki,bka) :
    dki = dki + (dki * 0.3)
    dka = dka + (dka * 0)
    bki = bki + (bki * 0)
    bka = bka + (bka * 0.3)
    
        
    ser.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))

def setJalan(ser,dki,dka,bki,bka,dly) :
    setMotor(ser,dki,dka,bki,bka)
    sleep(dly)
    if dki > 0 :
        invdki = 50
    elif dki < 0 :
        invdki = -50
    else :
        invdki = 0
    
    if dka > 0 :
        invdka = 50
    elif dka < 0 :
        invdka = -50
    else :
        invdka= 0
        
    if bki > 0 :
        invbki = 50
    elif bki < 0 :
        invbki = -50
    else :
        invbki= 0
        
        
    if bka > 0 :
        invbka = 50
    elif bka < 0 :
        invbka = -50
    else :
        invbka = 0
        
    setMotor(ser,invdki,invdka,invbki,invbka)
    sleep(0.1)
    setMotor(ser,0,0,0,0)
    sleep(0.1)

def dribbling(ser,val) :
    if val == 1 :
        ser.write(b"DB ON\n")
    else : 
        ser.write(b"DB OFF\n")

while True:
    vid = cv2.VideoCapture(0)
    while(vid.isOpened()):
        img, frame = vid.read()
        a = pickle.dumps(frame)
        msg = struct.pack("Q",len(a))+a
        client.sendall(msg)
        cv2.imshow('TRANSMITTING VIDEO', frame)

        # get message from server
        getMsg = client.recv(2048).decode(FORMAT)
        if getMsg == "kanan":
            setMotor(motor, 50,50,50,50)
        elif getMsg == "kiri":
            setMotor(motor, -50,-50,-50,-50)
        elif getMsg == "maju":
            setMotor(motor, 50,-50,50,-50)
        else:
            setMotor(motor, 0,0,0,0)

        print(getMsg)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            vid.release()
            cv2.destroyAllWindows()
            break
    break
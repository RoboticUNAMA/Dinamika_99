# METHOD NAVIGASI ROBOT
import serial
from time import sleep

PORT_MOTOR      = '/dev/ttyACM0'
PORT_DRIBBLE    = '/dev/ttyUSB0'
BAUDRATE        = 115200

motor = serial.Serial(PORT_MOTOR, BAUDRATE, timeout=1)
db = serial.Serial(PORT_DRIBBLE, 115200, timeout=1)
motor.close()
db.close()

def setMotor(ser,dki,dka,bki,bka) :
    if ser.isOpen() == False:
        ser.open()
    dki = dki + (dki * 0.3)
    dka = dka + (dka * 0)
    bki = bki + (bki * 0)
    bka = bka + (bka * 0.3) 
    ser.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    ser.close()

def stop(motor):
    setMotor(motor, 0,0,0,0)

def maju(motor, speed):
    setMotor(motor, -speed,speed,-speed,speed)

def maju2(motor, speed, delay):
    setMotor(motor, -speed,speed,-speed,speed)
    sleep(delay)
    setMotor(motor, speed,-speed,speed,-speed)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def mundur(motor, speed):
    setMotor(motor, speed,-speed,speed,-speed)

def mundur2(motor, speed, delay):
    setMotor(motor, speed,-speed,speed,-speed)
    sleep(delay)
    setMotor(motor, -speed,speed,-speed,speed)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def geserKiri(motor, speed):
    adj = speed*0.2
    setMotor(motor, speed,speed,-speed-adj,-speed)

def geserKiri2(motor, speed, delay):
    adj = speed*0.2
    setMotor(motor, speed,speed,-speed-adj,-speed)
    sleep(delay)
    setMotor(motor, -speed,-speed,speed+adj,speed)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def geserKanan(motor, speed):
    setMotor(motor, -speed,-speed,speed,speed)

def geserKanan2(motor, speed, delay):
    setMotor(motor, -speed,-speed,speed,speed)
    sleep(delay)
    setMotor(motor, speed,speed,-speed,-speed)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def putarKiri(motor, speed):
    setMotor(motor, speed,speed,speed,speed)

def putarKiri2(motor, speed, delay):
    setMotor(motor, speed,speed,speed,speed)
    sleep(delay) 
    setMotor(motor, -speed,-speed,-speed,-speed)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def putarKanan(motor, speed):
    setMotor(motor, -speed,-speed,-speed,-speed)

def putarKanan2(motor, speed, delay):
    setMotor(motor, -speed,-speed,-speed,-speed)
    sleep(delay) 
    setMotor(motor, speed,speed,speed,speed)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def serongKiri(motor, speed):
    setMotor(motor, 0,speed,-speed,0)

def serongKiri2(motor, speed, delay):
    setMotor(motor, 0,speed,-speed,0)
    sleep(delay)
    setMotor(motor, 0,-speed,speed,0)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def serongKanan(motor, speed):
    setMotor(motor, -speed,0,0,speed)

def serongKanan2(motor, speed, delay):
    setMotor(motor, -speed,0,0,speed)
    sleep(delay)
    setMotor(motor, speed,0,0,-speed)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def mundurSerongKanan(motor, speed):
    setMotor(motor, 0,-speed,speed,0)

def mundurSerongKanan2(motor, speed, delay):
    setMotor(motor, 0,-speed,speed,0)
    sleep(delay)
    setMotor(motor, 0,speed,-speed,0)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

def mundurSerongKiri(motor, speed):
    setMotor(motor, speed,0,0,-speed)

def mundurSerongKiri2(motor, speed, delay):
    setMotor(motor, speed,0,0,-speed)
    sleep(delay)
    setMotor(motor, -speed,0,0,speed)
    sleep(0.1)
    setMotor(motor, 0,0,0,0)

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

def dribbling(ser,val) :
    if ser.isOpen() == False:
        ser.open()
    if val == 1 :
        ser.write(b"DB ON\n")
    else : 
        ser.write(b"DB OFF\n")

def tendang(ser):
    if ser.isOpen() == False:
        ser.open()
    # === init tendang
    # ser.reset_input_buffer()
    dribbling(ser, 0)
    sleep(0.2)
    dribbling(ser, 0)
    sleep(0.2)
    dribbling(ser, 0)
    sleep(0.2)
    ser.write(b"TEND0\n")
    sleep(2)
    ser.close()

def oper_pelan(ser):
    if ser.isOpen() == False:
        ser.open()
    # === init tendang
    # ser.reset_input_buffer()
    dribbling(ser, 0)
    sleep(0.2)
    dribbling(ser, 0)
    sleep(0.2)
    dribbling(ser, 0)
    sleep(0.2)
    ser.write(b"TEND1\n")
    sleep(1)
    ser.close()

def oper(ser):
    if ser.isOpen() == False:
        ser.open()
    # === init tendang
    # ser.reset_input_buffer()
    putarKiri(30,0.1)
    dribbling(ser, 0)
    sleep(0.2)
    dribbling(ser, 0)
    sleep(0.2)  
    dribbling(ser, 0)
    sleep(0.2)
    ser.write(b"TEND2\n")
    sleep(1)
    ser.close()

def db_on(db):
    if db.isOpen() == False:
        db.open()
    db.write(b"DB ON\n")
    db.close()

def db_off(db):
    if db.isOpen() == False:
        db.open()
    db.write(b"DB OFF\n")
    db.close()

def passing(db):
    if db.isOpen() == False:
        db.open()
    db.write(b"PASSING\n")
    db.close()

def reset(db):
    if db.isOpen() == False:
        db.open()
    db.write(b"RESET\n")
    db.close()

if __name__ == '__main__':
    motor = serial.Serial(PORT_MOTOR, BAUDRATE, timeout=1)
    db = serial.Serial(PORT_DRIBBLE, 115200, timeout=1)
    motor.close()
    db.close()
    spd = 140
    while True:
        key = input("Input = ")
        if key == 'w':
            maju(motor, spd)
        elif key == 'a':
            geserKiri(motor, -spd)
        elif key == 'd':
            geserKanan(motor, spd)
        elif key == 's':
            mundur(motor, spd)
        elif key == 'q':
            serongKiri(motor, spd)
        elif key == 'e':
            serongKanan(motor, spd)
        elif key == 'z':
            putarKiri(motor, spd/2)
        elif key == 'x':
            putarKanan(motor, -spd/2)
        elif key == '9':
            db_on(db)
        elif key == '0':
            db_off(db)
        elif key == 't':
            tendang(db)
        elif key == 'y':
            passing(db)
        elif key == 'r':
            reset(db)
        else:
            stop(motor)
            db_off(db)
            reset(db)
import serial

motor = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
speed_awal = 100
adj = speed_awal*0.2

def maju():
    dki = -speed_awal - adj
    dka = speed_awal
    bki = -speed_awal
    bka = speed_awal + adj
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("maju")
    motor.flush()

def mundur():
    dki = speed_awal + adj
    dka = -speed_awal
    bki = speed_awal
    bka = -speed_awal - adj
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("mundur")

def berhenti():
    dki = 0
    dka = 0
    bki = 0
    bka = 0
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("berhenti")

def serongKanan():
    dki = -speed_awal
    dka = 0#-speed_awal
    bki = 0#speed_awal
    bka = speed_awal
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("serong kanan")

def serongKiri():
    dki = 0#speed_awal
    dka = speed_awal
    bki = -speed_awal
    bka = 0#speed_awal
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("serong kiri")

def geserKanan():
    dki = -speed_awal - adj
    dka = -speed_awal
    bki = speed_awal
    bka = speed_awal + adj
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("geser kanan")

def geserKiri():
    dki = speed_awal + adj
    dka = speed_awal
    bki = -speed_awal
    bka = -speed_awal - adj
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("geser kiri")

def putarKanan():
    dki = -speed_awal
    dka = -speed_awal
    bki = -speed_awal
    bka = -speed_awal
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("putar kanan")

def putarKiri():
    dki = speed_awal
    dka = speed_awal
    bki = speed_awal
    bka = speed_awal
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    print("putar kiri")

while(True):
    arah = input("Arah = ")

    if arah == 'w':
        maju()
    elif arah == 'a':
        geserKiri()
    elif arah == 's':
        mundur()
    elif arah == 'd':
        geserKanan()
    elif arah == 'q':
        serongKiri()
    elif arah == 'e':
        serongKanan()
    elif arah == 'z':
        putarKiri()
    elif arah == 'x':
        putarKanan()
    elif arah == '1':
        berhenti()
    elif arah == '0':
        db.write(b"DB OFF\n")
    elif arah == '9':
        db.write(b"DB ON\n")
    elif arah == '8':
        db.write(b"TENDANG\n")
    elif arah == '7':
        db.write(b"PASSING\n")
    else:
        berhenti()

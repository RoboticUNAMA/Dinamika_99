import serial

motor = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)
db = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

speed_awal = 90


while(True):
    arah = input("Arah = ")

    if arah == 'w':
       dki = speed_awal + (speed_awal * 0.3)
       dka = -speed_awal
       bki = speed_awal
       bka = -speed_awal - (speed_awal * 0.3)
       motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    elif arah == 'a':
       dki =  -speed_awal - (speed_awal * 0.3)
       dka = -speed_awal
       bki = -speed_awal
       bka = -speed_awal  - (speed_awal * 0.3)
       motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    elif arah == 's':
       dki = -speed_awal -  (speed_awal * 0.3)
       dka = speed_awal 
       bki = -speed_awal
       bka = speed_awal +  (speed_awal * 0.3)
       motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    elif arah == 'd':
       dki = speed_awal + (speed_awal * 0.3)
       dka = speed_awal
       bki = speed_awal
       bka = speed_awal + (speed_awal * 0.3)
       motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    elif arah == 'q':
       dki = -speed_awal -  (speed_awal * 0.3)
       dka = -speed_awal
       bki = speed_awal
       bka = speed_awal  +  (speed_awal * 0.3)
       motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    elif arah == 'e':
       dki = speed_awal   + (speed_awal * 0.3)
       dka = speed_awal
       bki = -speed_awal 
       bka = -speed_awal - (speed_awal * 0.3)
       motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    elif arah == 'g':
       db.write(("TEND2\n").encode('utf-8'))
    elif arah == 'x':
        motor.write(b"PUTAR KIRI\n")
    elif arah == 'r':
        motor.write(b"SERONG KIRI\n")
    elif arah == 't':
        motor.write(b"SERONG KANAN\n")
    elif arah == '1':
       dki = 0
       dka = 0
       bki = 0
       bka = 0
       motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|"+ str(bka) + "|"  + str(bki) + "\n").encode('utf-8'))
    elif arah == '0':
        db.write(b"DB OFF\n")
    elif arah == '9':
        db.write(b"DB ON\n")

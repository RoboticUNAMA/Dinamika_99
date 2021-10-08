import serial

br = 115200
motor = serial.Serial('/dev/ttyACM0', br, timeout=1)
db = serial.Serial('/dev/ttyUSB0', br, timeout=1)

def maju(motor, pwm):
    dki = -pwm-(pwm*0.5)
    dka = pwm
    bki = -pwm
    bka = pwm+(pwm*0.5)
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def mundur(motor, pwm):
    dki = pwm+(pwm*0.5)
    dka = -pwm
    bki = pwm
    bka = -pwm-(pwm*0.5)
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def geser(motor, pwm):
    dki = -pwm-(pwm*0.5)
    dka = -pwm
    bki = pwm
    bka = pwm+(pwm*0.5)
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def putar(motor, pwm):
    dki = pwm+(pwm*0.5)
    dka = pwm
    bki = pwm
    bka = pwm+(pwm*0.5)
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def serongKiri(motor, pwm):
    dki = 0
    dka = pwm*2
    bki = -pwm*2
    bka = 0
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def serongKanan(motor, pwm):
    dki = -pwm*2-(pwm*0.5)
    dka = 0
    bki = 0
    bka = pwm*2+(pwm*0.5)
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def berhenti(motor, pwm=0):
    dki = -pwm
    dka = pwm
    bki = -pwm
    bka = pwm
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def db_on(db):
    db.open()
    db.write(b"DB ON\n")
    db.close()

def db_off(db):
    db.open()
    db.write(b"DB OFF\n")
    db.close()

def tendang(db):
    db.open()
    db.write(b"TENDANG\n")
    db.close()

def passing(db):
    db.open()
    db.write(b"PASSING\n")
    db.close()

def reset(db):
    db.open()
    db.write(b"RESET\n")
    db.close()

def PID(centerObj, centerFrame):
    Kp = 5.0
    Kd = 5.0
    Ki = 0.0
    error = 0.0
    error_sebelumnya = 0.0
    selisih_error = 0.0
    jumlah_error = 0.0
    error = centerObj - centerFrame
    error_sebelumnya = error
    selisi_error = error_sebelumnya - error
    jumlah_error += (0.001 * error)
    P = Kp * error
    I = Ki * selisih_error
    D = Kd * jumlah_error
    PID = P + I + D
    PID = PID / 6
    return PID

def main():
    motor = serial.Serial('/dev/ttyACM0', br, timeout=1)
    db = serial.Serial('/dev/ttyUSB0', br, timeout=1)
    motor.close()
    db.close()
    spd = 140
    while True:
        key = input("Input = ")
        if key == 'w':
            maju(motor, spd)
        elif key == 'a':
            geser(motor, -spd)
        elif key == 'd':
            geser(motor, spd)
        elif key == 's':
            mundur(motor, spd)
        elif key == 'q':
            serongKiri(motor, spd)
        elif key == 'e':
            serongKanan(motor, spd)
        elif key == 'z':
            putar(motor, spd/2)
        elif key == 'x':
            putar(motor, -(spd/2))
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
            berhenti(motor)
            db_off(db)
            reset(db)

if __name__ == "__main__":
    # execute main program
    main()

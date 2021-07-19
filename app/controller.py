import serial

def maju(motor, pwm):
    dki = -pwm
    dka = pwm
    bki = -pwm
    bka = pwm
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def mundur(motor, pwm):
    dki = pwm
    dka = -pwm
    bki = pwm
    bka = -pwm
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def geser(motor, pwm):
    dki = pwm
    dka = pwm
    bki = -pwm
    bka = -pwm
    motor.open()
    motor.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
    motor.close()

def putar(motor, pwm):
    dki = pwm
    dka = pwm
    bki = pwm
    bka = pwm
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
    dki = -pwm*2
    dka = 0
    bki = 0
    bka = pwm*2
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

def main():
    motor = serial.Serial('/dev/ttyAMC0', 9600, timeout=1)
    db = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    motor.close()
    db.close()
    spd = 100
    while True:
        key = input("Input = ")
        if key == 'w':
            maju(motor, spd)
        elif key == 'a':
            geser(motor, spd)
        elif key == 'd':
            geser(-spd)
        elif key == 's':
            mundur(motor, spd)
        elif key == 'q':
            serongKiri(motor, spd)
        elif key == 'e':
            serongKanan(motor, spd)
        elif key == 'z':
            putar(motor, spd)
        elif key == 'x':
            putar(-spd)
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
            berhenti(motor, spd)
            db_off(db)
            reset(db)

if __name__ == "__main__":
    # execute main program
    main()

import serial

class Serial:
    def __init__(self, port1, port2):
        self.port1 = port1
        self.port2 = port2
        self.serial1 = serial.Serial(self.port1, baudrate=9600, timeout=1)
        self.serial2 = serial.Serial(self.port2, baudrate=9600, timeout=1)
        self.serial1.close()
        self.serial2.close()

    def maju(self, pwm):
        self.dki = -pwm
        self.dka = pwm
        self.bki = -pwm
        self.bka = pwm
        self.serial1.open()
        self.serial1.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
        self.serial1.close()
    
    def mundur(self, pwm):
        self.dki = pwm
        self.dka = -pwm
        self.bki = pwm
        self.bka = -pwm
        self.serial1.open()
        self.serial1.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
        self.serial1.close()

    def geser(self, pwm):
        self.dki = -pwm
        self.dka = pwm
        self.bki = -pwm
        self.bka = pwm
        self.serial1.open()
        self.serial1.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
        self.serial1.close()

    def putar(self, pwm):
        self.dki = pwm
        self.dka = pwm
        self.bki = pwm
        self.bka = pwm
        self.serial1.open()
        self.serial1.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
        self.serial1.close()


    def serongKiri(self, pwm):
        self.dki = 0
        self.dka = pwm*2
        self.bki = -pwm*2
        self.bka = 0
        self.serial1.open()
        self.serial1.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
        self.serial1.close()

    def serongKanan(self, pwm):
        self.dki = -pwm*2
        self.dka = 0
        self.bki = 0
        self.bka = pwm*2
        self.serial1.open()
        self.serial1.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
        self.serial1.close()

    def berhenti(self, pwm=0):
        self.dki = -pwm
        self.dka = pwm
        self.bki = -pwm
        self.bka = pwm
        self.serial1.open()
        self.serial1.write(("#M|RUN|" + str(dki) + "|" + str(dka) + "|" + str(bka) + "|" + str(bki) + "\n").encode("utf-8"))
        self.serial1.close()

    def db_on(self):
        self.serial2.open()
        self.serial2.write(b"DB ON\n")
        self.serial2.close()

    def db_off(self):
        self.serial2.open()
        self.serial2.write(b"DB OFF\n")
        self.serial2.close()

    def tendang(self):
        self.serial2.open()
        self.serial2.write(b"TENDANG\n")
        self.serial2.close()

    def passing(self):
        self.serial2.open()
        self.serial2.write(b"PASSING\n")
        self.serial2.close()

    def reset(self):
        self.serial2.open()
        self.serial2.write(b"RESET\n")
        self.serial2.close()

def main():
    s = Serial('/dev/ttyACM0', '/dev/ttyUSB0')
    speed = 100
    while True:
        key = input("Input = ")
        if key == 'w':
            s.maju(spd)
        elif key == 'a':
            s.geser(spd)
        elif key == 'd':
            s.geser(-spd)
        elif key == 's':
            s.mundur(spd)
        elif key == 'q':
            s.serongKiri(spd)
        elif key == 'e':
            s.serongKanan(spd)
        elif key == 'z':
            s.putar(spd)
        elif key == 'x':
            s.putar(-spd)
        elif key == '9':
            s.db_on()
        elif key == '0':
            s.db_off()
        elif key == 't':
            s.tendang()
        elif key == 'y':
            s.passing()
        elif key == 'r':
            s.reset()

if __name__ == "__main__":
    # execute main program
    main()

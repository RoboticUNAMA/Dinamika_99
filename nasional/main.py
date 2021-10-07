from deteksi import *
from server import getGame

def main():
    while True:
        game = getGame()
        print(game)
        if game == "START":
            arahBolaKameraAtas()
            putarKiri2(motor, 90, 1)
            db_off(db)
            tendang(db)
        elif game == "STOP":
            break  

if __name__ == '__main__':
    main()
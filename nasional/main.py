from deteksi import *
from server import getGame

def main():
    while True:
        game = getGame()
        print(game)
        if game == "START":
            db_on()
            break
        sleep(1)
        db_off()

if __name__ == '__main__':
    main()
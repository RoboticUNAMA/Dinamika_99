from deteksi import *
from server import getGame

def main():
    while True:
        game = getGame()
        print(game)
        if game == "START":
            db_on(db)
            sleep(3)
            db_off(db)
            break

if __name__ == '__main__':
    main()
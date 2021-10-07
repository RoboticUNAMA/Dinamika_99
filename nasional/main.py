from deteksi import *
from server import getGame

def main():
    while True:
        game = getGame()
        print(game)
        if game == "START":
            db_on(db)
            break
        sleep(1)
        db_off(db)

if __name__ == '__main__':
    main()
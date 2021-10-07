from deteksi import *
from server import getGame

def main():
    while True:
        game = getGame()
        print(game)
        if game == "START":
            dribbling(db,1)
            sleep(0.5)
            dribbling(db,1)
            sleep(0.5)
            dribbling(db,1)
            break

if __name__ == '__main__':
    main()
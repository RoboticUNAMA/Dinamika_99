from deteksi import *
from server import getGame

def main():
    while True:
        game = getGame()
        print(game)
        if game == "START":
            tendang(db)
            break

if __name__ == '__main__':
    main()
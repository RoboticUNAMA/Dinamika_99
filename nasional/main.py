from deteksi import *

def main():
    game = getGame()
    while True:
        if game == "START":
            arahBolaKameraDepan()
            break

if __name__ == '__main__':
    main()
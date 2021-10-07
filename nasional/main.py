from deteksi import *

def main():
    while True:
        game = getGame()
        if game == "START":
            arahBolaKameraDepan()
            break

if __name__ == '__main__':
    main()
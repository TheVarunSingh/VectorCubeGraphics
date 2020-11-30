import serial
from sys import platform

if "win32" in platform:
    import msvcrt
    the_port='COM7'
else:
    import getch
    the_port='/dev/ttyACM1'

VALID_CHARS = "wasdqeijkluovn"
QUIT_CHARS = "b"

def openSerialPort():
    serialPort = serial.Serial(
        port=the_port,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        write_timeout=1
    )
    print("Opening serial port", serialPort.name)
    return serialPort

def windowsLoop(serialPort):
    while True:
        if msvcrt.kbhit():
            char = msvcrt.getch().decode('utf-8')

            if char in VALID_CHARS:
                print("You pressed:", char)
                serialPort.write(char.encode('utf-8'))

            if char in QUIT_CHARS:
                print("Bye bye! See you next time!")
                print("Quitting and closing serial port", serialPort.name)
                serialPort.close()
                break

def linuxLoop(serialPort):
    while True:
        char = getch.getch()

        if char in VALID_CHARS:
            print("You pressed:", char)
            serialPort.write(char.encode('utf-8'))

        if char in QUIT_CHARS:
            print("Bye bye! See you next time!")
            print("Quitting and closing serial port", serialPort.name)
            serialPort.close()
            break

def main():
    serialPort = openSerialPort()

    if "win32" in platform:
        windowsLoop(serialPort)
    else:
        linuxLoop(serialPort)

if __name__ == "__main__":
    main()

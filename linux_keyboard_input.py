import getch # you can install this with pip
import serial

VALID_CHARS = "wasdqeijkluovn"
QUIT_CHARS = "b"

ser = serial.Serial(
    port='/dev/ttyACM1',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    write_timeout=1
)

print("Opening serial port", ser.name)

while True:
    char = getch.getch()

    if char in VALID_CHARS:
        print("You pressed:", char)
        ser.write(char.encode('utf-8'))

    if char in QUIT_CHARS:
        print("Bye bye! See you next time!")
        print("Quitting and closing serial port", ser.name)
        ser.close()
        break
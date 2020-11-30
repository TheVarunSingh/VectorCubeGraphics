import msvcrt
import serial

VALID_CHARS = "wasdqeijkluovn"
QUIT_CHARS = "b"

ser = serial.Serial(
    port='COM7',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    write_timeout=1
)

print("Opening serial port", ser.name)

while True:
    if msvcrt.kbhit():
        char = msvcrt.getch().decode('utf-8')

        if char in VALID_CHARS:
            print("You pressed:", char)
            ser.write(char.encode('utf-8'))

        else if char in QUIT_CHARS:
            print("Bye bye! See you next time!")
            print("Quitting and closing serial port", ser.name)
            ser.close()
            break

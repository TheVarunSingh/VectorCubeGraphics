import getch # you can install this with pip
import serial

VALID_CHARS = ['w', 'a', 's', 'd', 'i', 'k', 'j']

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
        print("Sending ",char)
        ser.write(char.encode())

    if char == 'q':
        print('Quitting and closing serial port')
        ser.close()
        break
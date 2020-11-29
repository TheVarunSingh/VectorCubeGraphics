import msvcrt
import serial

VALID_CHARS = [b'w', b'a', b's', b'd', b'i', b'k', b'j']

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
        char = msvcrt.getch()

        if char in VALID_CHARS:
            ser.write(char)

        if char == b'q':
            print('Quitting and closing serial port')
            ser.close()
            break

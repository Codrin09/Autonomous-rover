import datetime
import math
import pprint
import serial
def ana(y):
    global x
    x *= 2
    y *= 2
    print(y)
    print(x)

if __name__ == "__main__":
    """Opening of the serial port"""
    x = "ana"
    y = 10
    ana(y)
    print(y)

    """Opening of the serial port"""
    try:
        # arduino = serial.Serial("/dev/tty.usbmodem14101", 115200)
        arduino = serial.Serial("/dev/tty.HC-05-DevB", 115200, timeout = 2)
        arduino.flushInput() #This gives the bluetooth a little kick
    except:
        print('Please check the port')
        sys.exit(0)

    print("Connected")
    while True:
        line = input()
        arduino.write(line.encode())
        msg = arduino.readline()
        print(msg)
    # x1 = 900
    # y1 = 1100
    # x2 = 800
    # y2 = 1200 
    # baseTh = 180


    # x1 = 1000
    # y1 = 1000
    # x2 = 800
    # y2 = 1200
    # baseTh = 180
    # rotation = baseTh - math.degrees(math.atan2(y2 - y1, x2 - x1))
    # print('Rotation', rotation)

    # baseTh = 0
    # rotation_angle = rotation
    # if rotation_angle > 180:
    #     rotation_angle -= 360
    # elif rotation_angle <= -180:
    #     rotation_angle += 360
    # print("rotation_angle", rotation_angle)
    # if rotation > 180 or rotation < -180:
    #     print("1", rotation % 360)
    # else:
    #     print("2",rotation)

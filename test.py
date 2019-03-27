import datetime
import math
import pprint
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

    x1 = y1 = 1000
    x2 = 1050
    y2 = 950
    rotation = math.degrees(math.atan2(x2 - x1, y2 - y1)) - 180

    
    print('Rotation', rotation)

    baseTh = 0
    if rotation > 180 or rotation < -180:
        print("Sal")
        print(rotation % 360)
    else:
        print("sal2")
        print(rotation)

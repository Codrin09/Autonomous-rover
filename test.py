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

    rotation = math.degrees(math.atan2(250, 0))
    baseTh = 180
    print(rotation - baseTh)

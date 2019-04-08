import datetime
import math
import pprint
import serial
import lee
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

    baseX = 889
    baseY = 996
    endX = 789
    endY = 1096

    matrix = [[0 for col in range(2001)] for row in range(2001)]

    minX, maxX = min(baseX, endX), max(baseX, endX)
    minY, maxY = min(baseY, endY), max(baseY, endY)
    a,b,c  = lee.line_equation(baseX, baseY, endX, endY)

    if maxX - minX >= maxY - minY:
        for x in range (minX, maxX, 2):
            y = int((-1) * (a * x + c) / b)
            print(x, y)
            if lee.check_crash(x, y, matrix):
                crashed = True
                break;
    else:
        for y in range (minY, maxY, 2):
            x = int((-1) * (b * y + c) / a)
            print(x, y)
            if lee.check_crash(x, y, matrix):
                crashed = True
                break;
    
    baseTh = 180
    rotation = baseTh - math.degrees(math.atan2(endY - baseY, endX - baseX))
    print('Rotation', rotation)

    baseTh = 0
    rotation_angle = rotation
    if rotation_angle > 180:
        rotation_angle -= 360
    elif rotation_angle <= -180:
        rotation_angle += 360
    print("rotation_angle", rotation_angle)
    if rotation > 180 or rotation < -180:
        print("1", rotation % 360)
    else:
        print("2",rotation)

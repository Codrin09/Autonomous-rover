import sys
import signal
import serial
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import datetime

"""Opening of the serial port"""
try:
    # arduino = serial.Serial("/dev/tty.usbmodem14101", 115200)
    arduino = serial.Serial("/dev/tty.NICE-BT-DevB", 115200)
    arduino.flushInput() #This gives the bluetooth a little kick
except:
    print('Please check the port')
    sys.exit(0)

matrix = [[0 for col in range(1001)] for row in range(1001)]
changed = [0 for i in range(361)]

baseX = baseY = 500
matrix[baseX][baseY] = 1
scans = 0


cmap = ListedColormap(['k', 'w', 'r'])
# create the figure
fig, ax = plt.subplots()
matrice = ax.matshow(matrix, cmap = cmap)
plt.colorbar(matrice)

def init():
    global distances, rawdata, wasSet, matrix
    """Initialising variables""" 
    distances = [0 for _ in range(361)]
    rawdata= [0 for _ in range(361)]
    wasSet = [0 for _ in range(361)]
    
    matrice.set_array(matrix)
    
    line = "0"
    while int(line) != 1:
        print("Press 1 to start mapping")
        line = sys.stdin.readline()
    arduino.write(line.encode())

    return matrice,

def edit_point(x, y, action):
    global matrix

    left = -1
    right = 2
    if action == "create":
        action = 1
    else:
        action = 0

    for i in range(left, right):
        for j in range(left, right):
            if x + i >= 0 and x + i <= 1000 and y + j >= 0 and y + j <= 1000:
                try:
                    matrix[x+i][y+j] = action
                except Exception as e:
                    print(e)
                    print ("Edit point: " + str(x+i) + " " + str(y+j))

def update(index):
    global distances, rawdata, wasSet, matrix, scans, baseX, baseY
    """Receiving data and storing it in a list"""
    print("reading")
    scans += 1
    # manual_move(scans)

    print(datetime.datetime.now())
    for angle in range(360):
        rawdata[angle] = str(arduino.readline())[:-3]
        print(rawdata[angle])
        #print(rawdata[angle] + "(in update)") 
        split = rawdata[angle].split(":")
        try:
            new_distance = float(split[1][1:].replace("\\r", ""))
            new_set = int(split[0][-2])

            if new_set == 1:
                distances[angle] = int(new_distance)
            wasSet[angle] = new_set

        except Exception as e:
            print(e)
            print (rawdata[angle])
    #print(rawdata)
    #print("------------------------------------------------------------------------")

    print(datetime.datetime.now())

    count = 0 
    print("finish read")
    for angle in range(361):
        if wasSet[angle] == 1:
            changed[angle] = 1

            deltaX = distances[angle] * math.sin(math.radians(angle)) / 4
            deltaY = distances[angle] * math.cos(math.radians(angle)) / 4

            pointX = baseX + int(deltaX)
            pointY = baseY + int(deltaY)

            #print (str(i) + ": " + str(xCoord + int(deltaX)) + " " + str(yCoord + int(deltaY)) + " " + str(distances[i]))
            edit_point(pointX, pointY, "create")
            # matrix[pointX][pointY] = 1
            count+=1

            a = baseY - pointY
            b = pointX - baseX
            c = baseX * (pointY - baseY) + baseY * (baseX - pointX)

            x = y = 0

            minX = min(baseX, pointX)
            maxX = max(baseX, pointX)
            minY = min(baseY, pointY)
            maxY = max(baseY, pointY)

            if baseX == pointX:
                try:
                    for y in range(minY + 1, maxY):
                        edit_point(pointX, y, "delete")
                        # matrix[pointX][y] = 0
                except Exception as e:
                    print(e)
                    print("pointX, Y: " + str(pointX) + " " + str(y))
                
            else:
                x = y = 0
                try:
                    if maxX - minX >=  maxY - minY:
                        for x in range (minX + 1, maxX):
                            y = int((-1) * (a * x + c) / b)
                            edit_point(x, y, "delete")
                    else:
                        for y in range (minY + 1, maxY):
                            x = int((-1) * (b * y + c) / a)
                            edit_point(x, y, "delete")

                    # matrix[x][y] = 0
                except Exception as e:
                    print(e)
                    print("X,Y: " + str(x) + " " + str(y))

        elif changed[angle] == 1:
            changed[angle] = 0
            for distance in range(2001):
                deltaX = distance * math.sin(math.radians(angle)) / 4
                deltaY = distance * math.cos(math.radians(angle)) / 4

                pointX = baseX + int(deltaX)
                pointY = baseY + int(deltaY)

                try:
                    if pointX >= 0 and pointX <= 1000 and pointY >= 0 and pointY <= 1000:
                        edit_point(pointX, pointY, "delete")
                        # matrix[pointX][pointY] = 0
                    else:
                        break;
                except Exception as e:
                    print(e)
                    print("pointX, pointY: " + str(pointX) + " " + str(pointY))

    print("******* count: " + str(count))
    matrice.set_array(matrix)
    # matrix = [[0 for col in range(1001)] for row in range(1001)]

    return matrice,

def signal_handler(sig, frame):
    print('Shutting down')
    arduino.write('2'.encode())
    arduino.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
    sys.exit(0)

def manual_move(scans):
    global baseY
    if (scans/4)%2 == 1:
        arduino.write('2'.encode());
        print("Waiting for user input")
        line = sys.stdin.readline()
        arduino.write('1'.encode())
        baseY = 587

    if (scans/4)%2 == 0:
        arduino.write('2'.encode());
        print("Waiting for user input")
        line = sys.stdin.readline()
        arduino.write('1'.encode())
        baseY = 500


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    # ani = animation.FuncAnimation(fig, update, frames=200, init_func = init, blit = True)
    ani = animation.FuncAnimation(fig, update, frames=200, interval=200, init_func = init, blit = True)
    plt.show()
    main()
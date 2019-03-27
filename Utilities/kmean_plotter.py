import sys
import signal
import serial
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import datetime
from kmeans import *
import random

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

def edit_point(x, y, action, recently_set):
    global xyCoords, matrix, xyHash
    left = -1
    right = 2
    if action == "create":
        action = 1
    else:
        action = 0
        # left += 1
        # right -= 1

    for i in range(left, right):
        for j in range(left, right):
            if x + i >= 0 and x + i <= 1000 and y + j >= 0 and y + j <= 1000:
                try:
                    if action == 1:
                        recently_set[x+i][y+j] = 1
                        if (x+i, y+j) not in xyHash or xyHash[(x+i, y+j)] != 1:    
                            xyCoords.append((x+i, y+j))
                            xyHash[(x+i, y+j)] = 1
                    elif recently_set[x+i][y+j] == 1:
                        continue;
                    elif matrix[x+i][y+j] != 0 and xyHash[(x+i, y+j)] == 1:
                        xyCoords.remove((x+i, y+j))
                        xyHash[(x+i, y+j)] = 0
                    matrix[x+i][y+j] = action
                except Exception as e:
                    print(e)
                    print ("Edit point: " + str(x+i) + " " + str(y+j))

def update(index):
    global distances, rawdata, wasSet, matrix, scans, baseX, baseY, xyHash
    """Receiving data and storing it in a list"""
    print("reading")
    scans += 1
    # manual_move(scans)

    print(datetime.datetime.now())
    for angle in range(360):
        rawdata[angle] = str(arduino.readline())[:-3]
        # print(rawdata[angle])
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

    recently_set = [[0 for col in range(1001)] for row in range(1001)]

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
            edit_point(pointX, pointY, "create", recently_set)
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
                    #maxY - 2 because of 2 * padding 
                    #minY + 3 bacaue of 2 * padding + 1
                    for y in range(minY + 3, maxY - 2):
                        edit_point(pointX, y, "delete", recently_set)
                        # matrix[pointX][y] = 0
                except Exception as e:
                    print(e)
                    print("pointX, Y: " + str(pointX) + " " + str(y))
                
            else:
                x = y = 0
                try:
                    if maxX - minX >=  maxY - minY:
                        for x in range (minX + 3, maxX - 2):
                            y = int((-1) * (a * x + c) / b)
                            edit_point(x, y, "delete", recently_set)
                    else:
                        for y in range (minY + 3, maxY - 2):
                            x = int((-1) * (b * y + c) / a)
                            edit_point(x, y, "delete", recently_set)

                    # matrix[x][y] = 0
                except Exception as e:
                    print(e)
                    print("X,Y: " + str(x) + " " + str(y))

        elif changed[angle] == 1:
            changed[angle] = 0
            #from 3*sqrt(2) to 2000
            for distance in range(5, 2001):
                deltaX = distance * math.sin(math.radians(angle)) / 4
                deltaY = distance * math.cos(math.radians(angle)) / 4

                pointX = baseX + int(deltaX)
                pointY = baseY + int(deltaY)

                try:
                    if pointX >= 0 and pointX <= 1000 and pointY >= 0 and pointY <= 1000:
                        edit_point(pointX, pointY, "delete", recently_set)
                        # matrix[pointX][pointY] = 0
                    else:
                        break;
                except Exception as e:
                    print(e)
                    print("pointX, pointY: " + str(pointX) + " " + str(pointY))

    index = 0
    x = []
    y = []
    for item in xyCoords:
        (xVal, yVal) = item
        x.append(xVal)
        y.append(yVal)
    
    #K-means algorithm
    df = pd.DataFrame()
    df['x'] = x
    df['y'] = y

    global centroids, col_ind, col_map, clusters_no, cmap
    old_cl_no = 0
    while clusters_no != old_cl_no:
        print("back")
        old_cl_no = clusters_no
        df = assignment(df, centroids, col_map)
        df, centroids = k_means(df, centroids, col_map)

        index = 0
        for x in df['x']:
            y = df['y'][index]
            color = df['color'][index]
            matrix[x][y] = col_ind[color]

            closest = df['closest'][index]
            left = centroids[closest][0]
            right = centroids[closest][1]
            #to be continued 
            if math.sqrt(math.pow(x - left, 2) + math.pow(y - right, 2)) > 200:
                clusters_no += 1
            index += 1
        
        print(clusters_no)

        for i in range (old_cl_no, clusters_no + 1):
            rand_col = random_color()
            col.append(rand_col)
            col_ind[rand_col] = i + 1
            col_map[i+1] = rand_col
            centroids[i+1] = [np.random.randint(0, 1000), np.random.randint(0, 1000)]

        print("out")
        cmap = ListedColormap(col)

    # fig = plt.figure(figsize=(5, 5))
    # plt.scatter(df['x'], df['y'], color=df['color'], alpha=0.5, edgecolor='k')
    # for i in centroids.keys():
    #     plt.scatter(*centroids[i], color=colmap[i])
    # plt.xlim(1000, 0)
    # plt.ylim(0, 1000)
    # plt.show()

    print("******* count: " + str(count))
    matrice.set_array(matrix)
    # matrix = [[0 for col in range(1001)] for row in range(1001)]

    return matrice,

def random_color():
    return "#{:06x}".format(random.randint(0, 0xFFFFFF))

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
    xyCoords = [(baseX, baseY)]
    xyHash = {}
    xyHash[(500, 500)] = 1

    matrix[baseX][baseY] = 20
    scans = 0

    col_ind = {}
    col_map = {}
    clusters_no = 10
    centroids = {
        i+1: [np.random.randint(0, 1000), np.random.randint(0, 1000)]
        for i in range(clusters_no)
    }

    col = ['k']
    for i in range(clusters_no):
        rand_col = random_color()
        col.append(rand_col)
        col_ind[rand_col] = i + 1
        col_map[i+1] = rand_col

    cmap = ListedColormap(col)
    # cmap = ListedColormap(['k', 'r', 'g', 'b', 'w'])
    # create the figure
    fig, ax = plt.subplots(figsize = (7,7))
    matrice = ax.matshow(matrix, cmap = cmap)
    plt.colorbar(matrice)
    signal.signal(signal.SIGINT, signal_handler)
    # ani = animation.FuncAnimation(fig, update, frames=200, init_func = init, blit = True)
    ani = animation.FuncAnimation(fig, update, frames=200, interval=200, init_func = init, blit = True)
    plt.show()
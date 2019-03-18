import sys
import signal
import serial
import math
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import datetime
from linear_regression import *
import pprint

def init_map(testing = None):
    global distances, rawdata, wasSet, matrix, travelDistance, arduino, sin, cos
    """Initialising variables""" 
    distances = [0 for _ in range(360)]
    rawdata= [0 for _ in range(360)]
    wasSet = [0 for _ in range(360)]
    travelDistance = [0 for _ in range(360)]
    sin = []
    cos = []
    for angle in range(360):
        sin.append(math.sin(math.radians(angle)))
        cos.append(math.cos(math.radians(angle)))
    
    matrice.set_array(matrix)

    if testing == True:
        line = "0"
        while int(line) != 1:
            print("Press 1 to start mapping")
            line = sys.stdin.readline()
        arduino.write(line.encode())

    return matrice,
def handle_scans():
    global rawdata, wasSet, distances, travelDistance, arduino, baseTh
    for angle in range(360):
        real_angle = (angle + baseTh) % 360
        rawdata[real_angle] = str(arduino.readline())[:-3].replace("\\r", "")
        # print(rawdata[angle])
        split = rawdata[real_angle].split(":")
        try:
            new_distance = float(split[1][1:])
            new_set = int(split[0][-2])
            new_travel_dist = float(split[2])

            if new_set == 1:
                distances[real_angle] = new_distance
            
            #! Travel distance here
            # travelDistance[angle] = new_travel_dist
            wasSet[real_angle] = new_set

        except Exception as e:
            print(e)
            print (rawdata[real_angle])
def update_map(tag):
    global distances, rawdata, wasSet, matrix, scans, baseX, baseY, baseTh, changed, travelDistance, arduino
    """Receiving data and storing it in a list"""
    print("reading")
    scans += 1
    # manual_move(scans)

    print(datetime.datetime.now())
    handle_scans()
    print(datetime.datetime.now())

    new_points = []
    k = m = -1
    cluster = [0 for i in range(360)]
    clusterX = []
    clusterY = []
    cluster_ind = 1

    count = 0 
    print("finish read")

    for angle in range(360):
        real_angle = (angle + baseTh) % 360
        
        newBaseX = baseX + round(math.sin(math.radians(baseTh)) * travelDistance[real_angle] / 4);
        newBaseY = baseY + round(math.cos(math.radians(baseTh)) * travelDistance[real_angle] / 4);

        if wasSet[real_angle] == 1:
            changed[real_angle] = 1

            deltaX = distances[real_angle] * math.sin(math.radians(real_angle)) / 4
            deltaY = distances[real_angle] * math.cos(math.radians(real_angle)) / 4

            pointX = newBaseX + round(deltaX)
            pointY = newBaseY + round(deltaY)

            k, m, clusterX, clusterY, cluster, cluster_ind, new_points = feature_extraction(k, m, clusterX, clusterY, pointX, pointY, real_angle, cluster, cluster_ind, new_points)
            count+=1
            # edit_point(pointX, pointY, "create")
            draw_line(newBaseX, newBaseY, pointX, pointY, "delete")

        #! Assuming we have no moving obstacles we don't need this else check for changed[angle] is 1
        elif changed[real_angle] == -1:
            changed[real_angle] = 0
            #from 5 as 3*sqrt(2) ~ 4.2
            #old 5 2001
            for distance in range(5, 2001):
                deltaX = distance * math.sin(math.radians(real_angle)) / 4
                deltaY = distance * math.cos(math.radians(real_angle)) / 4

                pointX = newBaseX + round(deltaX)
                pointY = newBaseY + round(deltaY)

                try:
                    if pointX >= 0 and pointX <= 1000 and pointY >= 0 and pointY <= 1000:
                        edit_point(pointX, pointY, "delete")
                    else:
                        break;
                except Exception as e:
                    print(e)
                    print("pointX, pointY: " + str(pointX) + " " + str(pointY))

    index = 0
    oldVal = 0
    startX = startY = 0
    #!Try to use linear regression to map the start and end of the line instead of using first and last point from cluster 
    for angle in range(360):
        real_angle = (angle + baseTh) % 360
        if cluster[real_angle] != 0:
            newX, newY = new_points[index]
            if oldVal == 0:
                oldVal = cluster[real_angle]
                startX, startY = newX, newY
            elif oldVal != cluster[real_angle]:
                draw_line(startX, startY, lastX, lastY, "create", cluster[real_angle])
                startX, startY = newX, newY
                oldVal = cluster[real_angle]
            elif real_angle == 359:
                draw_line(startX, startY, newX, newY, "create", cluster[real_angle])

            # print(i, ":", new_points[index], distances[i], cluster[i])
            # edit_point(newX, newY, "create", cluster[i])
            lastX, lastY = newX, newY
            index += 1

    print("******* count: " + str(count))
    matrice.set_array(matrix)

    return matrice,

def valid_point(x, y):
    if x >= 0 and x <= 1000 and y >= 0 and y <= 1000:
        return True
    return False

def get_position():
    global wasSet, distances, baseX, baseY, baseTh
    handle_scans()

    # x = baseX
    # y = baseY
    x = 250
    y = 500

    current_observations = []
    for angle in range(360):
        real_angle = (angle + baseTh) % 360
        if wasSet[real_angle] == 1:
            deltaX = distances[real_angle] * sin[real_angle] / 4
            deltaY = distances[real_angle] * cos[real_angle] / 4

            pointX = x + round(deltaX)
            pointY = y + round(deltaY)

            current_observations.append((pointX, pointY, real_angle))
    
    left = -24
    right = 25

    # z = simulate_observations(500, 500)
    # z.sort(key = lambda x: (x[0], x[1]))
    # current_observations.sort(key = lambda x: (x[0], x[1]))

    # pp = pprint.PrettyPrinter(indent=4)
    # print("Current observations")
    # pp.pprint(current_observations)
    # print("Simulated observations")
    # pp.pprint(z)

    print("No of observations:",len(current_observations))

    maxMatches = 0
    gX = gY = gTh = 0
    for i in range(left, right, 4):
        for j in range(left, right, 4):
            if valid_point(x + i, y + j):
                z = simulate_observations(x + i, y + j)
                matches = 0
                start_angle = -1
                for h in range(len(current_observations)):
                    for k in range(len(z)):
                        if abs(z[k][0] - current_observations[h][0]) < 2 and abs(z[k][1] - current_observations[h][1]) < 2:
                            if start_angle == -1:
                                angle_obs = current_observations[h][2]
                                angle_sim = z[k][2]
                                if angle_obs > angle_sim:
                                    # start_angle = 359 - (angle_obs - angle_sim)
                                    start_angle = 0
                                else:
                                    # start_angle = angle_sim - angle_obs
                                    start_angle = 0
                            matches += 1
                            break
                if matches >  maxMatches:
                    maxMatches = matches
                    gX, gY, gTh = x+i, y+j, (baseTh + start_angle) % 360
    print(gX, gY, gTh, maxMatches)
    return (gX, gY, gTh)
                
def simulate_observations(x, y):
    global matrix, sin, cos, baseTh
    z = []
    for angle in range(360):
        real_angle = (angle + baseTh) % 360
        pointX = pointY = -1
        for distance in range(2, 501):
            deltaX = distance * sin[real_angle]
            deltaY = distance * cos[real_angle]

            pointX = x + round(deltaX)
            pointY = y + round(deltaY)

            if valid_point(pointX, pointY):
                if matrix[pointX][pointY] == 0:
                    continue
                else:
                    z.append((pointX, pointY, real_angle))
            break
    return z

def edit_point(x, y, action, value = None):
    global matrix
    left = -1
    right = 2
    if action == "create":
        action = value
    else:
        action = 0
        # left += 1
        # right -= 1

    for i in range(left, right):
        for j in range(left, right):
            if x + i >= 0 and x + i <= 1000 and y + j >= 0 and y + j <= 1000:
                try:
                    matrix[x+i][y+j] = action
                except Exception as e:
                    print(e)
                    print ("Edit point: " + str(x+i) + " " + str(y+j))

def draw_line(startX, startY, endX, endY, action, value = None):
    minX, maxX = min(startX, endX), max(startX, endX)
    minY, maxY = min(startY, endY), max(startY, endY)

    if startX == endX:
        #max - 2 because of 2 * padding / min + 3 because 2*padding + 1
        for y in range(minY + 3, maxY - 2):
            edit_point(startX, y, action, value)
    elif startY == endY:
        for x in range(minX + 3, minX - 2):
            edit_point(x, startY, action, value)
    else:
        a = startY - endY
        b = endX - startX
        c = startX * (endY - startY) + startY * (startX - endX)
        if maxX - minX >= maxY - minY:
            for x in range (minX + 3, maxX - 2):
                y = int((-1) * (a * x + c) / b)
                edit_point(x, y, action, value)
        else:
            for y in range (minY + 3, maxY - 2):
                x = int((-1) * (b * y + c) / a)
                edit_point(x, y, action, value)

    # k,m = linear_fit([startX, endX], [startY, endY])
    # try:
    #     if abs(startX - endX) > abs(startY - endY):
    #         for x in range(startX, endX):
    #             y = round(k * x + m)
    #             edit_point(x, y, "create", value)
    #     else:
    #         for y in range(startY, endY):
    #             x = round((y - m) / k)
    #             edit_point(x, y, "create", value)
    # except Exception as e:
    #     print(e)
    #     print(startX, startY, endX, endY)
    
def feature_extraction(k, m, clusterX, clusterY, pointX, pointY, angle, cluster, cluster_ind, new_points):
    clusterX.append(pointX)
    clusterY.append(pointY)
    cluster[angle] = cluster_ind

    if len(clusterX) > 1:
        oldX, oldY = new_points[len(new_points) - 1]
        newK, newM = linear_fit(clusterX, clusterY)

        if k == m == -1:
            k, m = newK, newM
        expectedY = k * pointX + m
        dist = points_distance(pointX, pointY, oldX, oldY)

        if (dist < 40 and abs(pointY - expectedY) < 20) or dist == 0:
            k, m = newK, newM
        else:
            # print(angle, pointX, pointY, oldX, oldY, "expected:", expectedY)
            cluster_ind += 1
            cluster[angle] = cluster_ind
            clusterX = [pointX]
            clusterY = [pointY]
            k = m = -1
            
    if angle == 359 and cluster[0] != 0:
        firstX, firstY = new_points[0]
        expectedY = k * firstX + m
        dist = points_distance(pointX, pointY, firstX, firstY)
        if (dist < 40 and abs(pointY - expectedY) < 40) or dist == 0:
            oldCluster = cluster[angle]
            for i in reversed(range(len(cluster))):
                if cluster[i] == oldCluster:
                    cluster[i] = 1
                else:
                    break;

    new_points.append((pointX, pointY))

    return k, m, clusterX, clusterY, cluster, cluster_ind, new_points

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

def points_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def signal_handler(sig, frame):
    print('Shutting down')
    arduino.write('2'.encode())
    arduino.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
    sys.exit(0) 

def init_variables(btConnection = None):
    global matrix, changed, baseX, baseY, baseTh, matrix, scans, cmap, matrice, fig, ax, arduino
    matrix = [[0 for col in range(1001)] for row in range(1001)]
    changed = [0 for i in range(360)]

    if btConnection != None:
        arduino = btConnection
    baseX = baseY = 500
    baseTh = 0
    matrix[baseX][baseY] = 15
    scans = 0

    cmap = ListedColormap(['k', 'w', 'r'])

    # create the figure
    fig, ax = plt.subplots(figsize = (7,7))
    matrice = ax.matshow(matrix, cmap = 'viridis')
    plt.colorbar(matrice)

if __name__ == "__main__":
    """Opening of the serial port"""
    try:
        arduino = serial.Serial("/dev/tty.NICE-BT-DevB", 115200)
        arduino.flushInput() #This gives the bluetooth a little kick
    except:
        print('Please check the port')
        sys.exit(0)

    init_variables()

    signal.signal(signal.SIGINT, signal_handler)
    # ani = animation.FuncAnimation(fig, update, frames=200, init_func = init, blit = True)
    ani = animation.FuncAnimation(fig, update_map(0), frames=200, interval=200, init_func = init_map(True), blit = True)
    plt.show()
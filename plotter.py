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
import random

#Initialise map
def init_map(testing = None):
    global distances, rawdata, wasSet, matrix, arduino, sin, cos
    """Initialising variables""" 
    distances = [0 for _ in range(360)]
    rawdata= [0 for _ in range(360)]
    wasSet = [0 for _ in range(360)]
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

#Deal with lidar input from arduino
def handle_scans():
    global rawdata, wasSet, distances, arduino, baseTh, baseX, baseY, mapTh
    for angle in range(360):
        #Calculate angle using curent orientation of the robot
        real_angle = (angle + mapTh) % 360
        rawdata[real_angle] = str(arduino.readline())[:-3].replace("\\r", "")
        # print(rawdata[angle])
        split = rawdata[real_angle].split(":")
        try:
            new_distance = float(split[1][1:])
            new_set = int(split[0][-2])
            new_travel_dist = float(split[2])

            if new_set == 1:
                distances[real_angle] = new_distance
            
            wasSet[real_angle] = new_set

        except Exception as e:
            print('hanlde_scans', e)
            print (rawdata[real_angle])
            
    # baseX -= round(math.cos(math.radians(baseTh)) * travelDistance / 4);
    # baseY -= round(math.sin(math.radians(baseTh)) * travelDistance / 4);

#Update matrix map with current readings where distances read by ladar are scaled to 1/4
def update_map(tag):
    global distances, rawdata, wasSet, matrix, baseX, baseY, baseTh, changed, arduino, mapTh
    """Receiving data and storing it in a list"""
    print("reading")

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
        real_angle = (angle + mapTh) % 360
        
        if wasSet[real_angle] == 1:
            changed[real_angle] = 1

            #Polar coordinates to cartesian coordinates
            deltaX = distances[real_angle] * math.sin(math.radians(real_angle)) / 4
            deltaY = distances[real_angle] * math.cos(math.radians(real_angle)) / 4

            pointX = baseX + round(deltaX)
            pointY = baseY + round(deltaY)

            #Assign point it to an old edge or create a new one if it doesn't fit any
            k, m, clusterX, clusterY, cluster, cluster_ind, new_points = feature_extraction(k, m, clusterX, clusterY, pointX, pointY, real_angle, cluster, cluster_ind, new_points)
            count+=1

            #Delete all points between current position of robot and observed point
            draw_line(baseX, baseY, pointX, pointY, "delete")

        #! Assuming we have no moving obstacles we don't need this else check for changed[angle] is 1
        elif changed[real_angle] == -1:
            #If no obstacle found at this angle delete all points from robot location until the max distance achieved by radar(2m)
            changed[real_angle] = 0
            
            #from 5 as 3*sqrt(2) ~ 4.2
            for distance in range(5, 2001):
                #Polar coordinates to cartesian coordinates
                deltaX = distance * math.sin(math.radians(real_angle)) / 4
                deltaY = distance * math.cos(math.radians(real_angle)) / 4

                pointX = baseX + round(deltaX)
                pointY = baseY + round(deltaY)

                try:
                    if pointX >= 0 and pointX <= 2000 and pointY >= 0 and pointY <= 2000:
                        edit_point(pointX, pointY, "delete")
                    else:
                        break;
                except Exception as e:
                    print('update_map', e)
                    print("pointX, pointY: " + str(pointX) + " " + str(pointY))

    index = 0
    #Draw new landmark points on the map inidividually or use linear_reg_draw() method
    for angle in range(360):
        real_angle = (angle + mapTh) % 360
        if cluster[real_angle] != 0:
            newX, newY = new_points[index]
            edit_point(newX, newY, "create", cluster[real_angle])
            index += 1

    #Number of values where lidar returned a value
    print("******* count: " + str(count))
    matrice.set_array(matrix)

    return matrice,

#Check if a point is inside the matrix
def valid_point(x, y):
    if x >= 0 and x <= 2000 and y >= 0 and y <= 2000:
        return True
    return False

#Get current approximate position based on predicted future position by using traveled distance
def get_position(distance):
    global wasSet, baseX, baseY, baseTh
    handle_scans()

    x = baseX + round(math.cos(math.radians(baseTh)) * distance)
    y = baseY + round(math.sin(math.radians(baseTh)) * distance)

    # x = baseX - round(math.cos(math.radians(baseTh)) * distance)
    # y = baseY - round(math.sin(math.radians(baseTh)) * distance)

    left = -24
    right = 25
    maxMatches = 0
    gX = gY = gTh = 0

    #Get lidar readings of landmarks in the new position
    current_observations = get_observations(x, y)
    current_observations =  sorted(current_observations, key = lambda x: (x[0], x[1]))
    print("No of observations:",len(current_observations))

    #Choose the point around our predicted location that fits best the new observations 
    for i in range(left, right, 2):
        for j in range(left, right, 2):
            if valid_point(x + i, y + j):
                matches = simulate_point(x+i, y+j)
                start_angle = 0
                if matches >  maxMatches:
                    maxMatches = matches
                    gX, gY, gTh = x+i, y+j, (baseTh + start_angle) % 360

    print("Best match for position is",gX, gY, gTh, "with number of matches", maxMatches)
    return (gX, gY, gTh)

def get_observations(x, y):
    global distances, sin, cos, baseTh, mapTh
    current_observations = []
    for angle in range(360):
        real_angle = (angle + mapTh) % 360
        if wasSet[real_angle] == 1:
            deltaX = distances[real_angle] * sin[real_angle] / 4
            deltaY = distances[real_angle] * cos[real_angle] / 4

            pointX = x + round(deltaX)
            pointY = y + round(deltaY)

            if(valid_point(pointX, pointY)):
                current_observations.append((pointX, pointY, real_angle))
    return current_observations

#Use simulated location to map observations to points and check how many landmarks they match
def simulate_point(x, y):
    global baseTh, matrix, distances, sin, cos, mapTh
    matches = 0
    for angle in range(360):
        real_angle = (angle + mapTh) % 360
        if wasSet[real_angle] == 1:
            deltaX = distances[real_angle] * sin[real_angle] / 4
            deltaY = distances[real_angle] * cos[real_angle] / 4

            pointX = x + round(deltaX)
            pointY = y + round(deltaY)
            
            if valid_point(pointX, pointY) and matrix[pointX][pointY] > 0:
                matches += 1
    return matches

#Edit matrix method to delete/create a new point and add a frame to it
def edit_point(x, y, action, value = None):
    global matrix
    left = -5
    right = 6

    if not valid_point(x, y):
        return

    if action == "create":
        #Assign to action the edge number if needed
        # action = value

        action = 7
        if matrix[x][y] != 0:
            return
    else:
        action = 0
        if matrix[x][y] == 0:
            return

    for i in range(left, right):
        for j in range(left, right):
            if valid_point(x+i, y+j):
                try:
                    matrix[x+i][y+j] = action
                except Exception as e:
                    print('edit_point', e)
                    print ("Edit point: " + str(x+i) + " " + str(y+j))

#Draw a line between two points
def draw_line(startX, startY, endX, endY, action, value = None):
    minX, maxX = min(startX, endX), max(startX, endX)
    minY, maxY = min(startY, endY), max(startY, endY)

    #max - 2 because of 2 * padding / min + 3 because 2*padding + 1
    padding_min = 2 * 6 + 1
    padding_max = 2 * 6

    if startX == endX:
        for y in range(minY + padding_min, maxY - padding_max):
            edit_point(startX, y, action, value)
    elif startY == endY:
        for x in range(minX + padding_min, maxX - padding_max):
            edit_point(x, startY, action, value)
    else:
        #Calculate line equation if points are not on same axis and draw between them 
        a = startY - endY
        b = endX - startX
        c = startX * (endY - startY) + startY * (startX - endX)
        if maxX - minX >= maxY - minY:
            for x in range (minX + padding_min, maxX - padding_max):
                y = int((-1) * (a * x + c) / b)
                edit_point(x, y, action, value)
        else:
            for y in range (minY + padding_min, maxY - padding_max):
                x = int((-1) * (b * y + c) / a)
                edit_point(x, y, action, value)
    
#Map nearby points to an edge
def feature_extraction(k, m, clusterX, clusterY, pointX, pointY, angle, cluster, cluster_ind, new_points):
    #Add curent point to cluster X/Y
    clusterX.append(pointX)
    clusterY.append(pointY)
    cluster[angle] = cluster_ind

    if len(clusterX) > 1:
        #Get the previously added point
        oldX, oldY = new_points[len(new_points) - 1]
        newK, newM = linear_fit(clusterX, clusterY)

        if k == m == -1:
            k, m = newK, newM
        expectedY = k * pointX + m
        dist = points_distance(pointX, pointY, oldX, oldY)

        #If distance between current point and previously added point is less than 2 cm and point matches linear regression function add to edge
        if (dist < 40 and abs(pointY - expectedY) < 20) or dist == 0:
            k, m = newK, newM
        else:
            # print(angle, pointX, pointY, oldX, oldY, "expected:", expectedY)
            cluster_ind += 1
            cluster[angle] = cluster_ind
            clusterX = [pointX]
            clusterY = [pointY]
            k = m = -1
    
    #If last point and first point match unite first edge with last edge
    if angle == 359 and cluster[0] != 0:
        firstX, firstY = new_points[0]
        expectedY = k * firstX + m
        dist = points_distance(pointX, pointY, firstX, firstY)
        if (dist < 40 and abs(pointY - expectedY) < 20) or dist == 0:
            oldCluster = cluster[angle]
            for i in reversed(range(len(cluster))):
                if cluster[i] == oldCluster:
                    cluster[i] = 1
                else:
                    break;

    new_points.append((pointX, pointY))

    return k, m, clusterX, clusterY, cluster, cluster_ind, new_points

#Distance between two points 
def points_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

def signal_handler(sig, frame):
    print('Shutting down plotter')
    arduino.write('2'.encode())
    arduino.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
    sys.exit(0) 

#Initialise plotter variables
def init_variables(btConnection = None):
    global matrix, changed, baseX, baseY, baseTh, matrix, cmap, matrice, fig, ax, arduino, mapTh
    
    #Matrix to plot
    matrix = [[0 for col in range(2001)] for row in range(2001)]
    changed = [0 for i in range(360)]

    if btConnection != None:
        arduino = btConnection

    #Base in the centre of the map
    baseX = baseY = 1000
    baseTh = 180
    mapTh = 180

    #Max value added to robot location
    matrix[baseX][baseY] = 15

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
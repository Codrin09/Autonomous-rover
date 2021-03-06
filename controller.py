import sys
import signal
import serial
import select 
import plotter
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import datetime
import lee
import math
from debug import *

"""Opening of the serial port"""
try:
    # arduino = serial.Serial("/dev/tty.usbmodem14101", 115200)
    arduino = serial.Serial("/dev/tty.HC-05-DevB", 115200, timeout = 0.1)
    arduino.flushInput() #This gives the bluetooth a little kick
except:
    print('Please check the port')
    sys.exit(0)

#Call main with any parameter for debug mode
def main(debug = None):
    started = 0
    
    if debug:
        debug_controls()  
    else:
        dest = input("Give distance from original robot pose (on X and Y axis): ")  
        xy = dest.split(" ")
        xDest = int(xy[0])
        yDest = int(xy[1])

        print("Send \"start\" to begin")

    plotter.init_variables(arduino)   
    plotter.init_map(arduino)
    path = []
    goalX = plotter.baseX + xDest
    goalY = plotter.baseY + yDest

    plt.ion()
    plt.show()

    while True:
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()[:-1]
            if line:
                send_msg(line)
            else: # an empty line means stdin has been closed
                print('eof')
                exit(0)
        else:
            #! Use decode to change bytes object to string
            message_received = str(arduino.readline()[:-2])[2:-1]
            if message_received:
                print(message_received)
                if message_received == "Create new path":
                    print("Creating new path")
                    #Show destination
                    plot_object(plotter.matrix, goalX, goalY, 11)
                    #Show current location
                    plot_object(plotter.matrix, plotter.baseX, plotter.baseY, 14)

                    path = lee.motion_planning(plotter.baseX, plotter.baseY, goalX, goalY, plotter.matrix)
                    if len(path) > 0 and path[-1] != (goalX, goalY): 
                        path.append((goalX, goalY))
                    
                    print(path)
                elif message_received == "Sending lidar readings":
                    plotter.update_map(0, True)

                    crashing = False
                    for angle in range(360):
                        if plotter.distances[angle] < 150 and plotter.distances[angle] > 100 and plotter.wasSet[angle] == 1:
                            crashing = True
                            break
                    if crashing:
                        send_msg("m 20 0")
                        print("Going backwards")
                        update_position(plotter, -1)
                        send_msg("l 1")

                    plt.draw()
                    plt.pause(0.001)
            #If robot didn't complete path do next move
            elif len(path) > 0:
                # print(datetime.datetime.now())
                started = 1

                #Get new intermediate destination
                nextX, nextY = path.pop(0)
                next_rotation = plotter.baseTh - round(math.degrees(math.atan2(nextY - plotter.baseY, nextX - plotter.baseX)))
                while len(path) > 0:
                    newX, newY = path[0]
                    rotation_angle = plotter.baseTh - round(math.degrees(math.atan2(newY - nextY, newX - nextX)))
                    #! Check for close angle +- 10 deg instead of equal
                    if abs(rotation_angle - next_rotation) < 22:
                        nextX, nextY = path.pop(0)
                    else:
                        break

                plot_object(plotter.matrix, goalX, goalY, 11)
                plot_object(plotter.matrix, plotter.baseX, plotter.baseY, 14)
                
                crashed = lee.path_check(plotter.baseX, plotter.baseY, nextX, nextY, plotter.matrix)
                        
                if crashed == True:
                    print("Crashing after map update")
                    print("Creating new path")
                    foundPath = False
                    while not foundPath:
                        path = lee.motion_planning(plotter.baseX, plotter.baseY, goalX, goalY, plotter.matrix)
                        if len(path) > 0:
                            foundPath = True
                            if path[-1] != (goalX, goalY): 
                                path.append((goalX, goalY))
                        else:
                            send_msg("m 20 0")
                            print("Going backwards")
                            update_position(plotter, -1)
                    print(path)
                    nextX, nextY = path.pop(0)

                # print(datetime.datetime.now())
                #Calculate rotation angle for robot
                rotation_angle = plotter.baseTh - round(math.degrees(math.atan2(nextY - plotter.baseY, nextX - plotter.baseX)))
                if rotation_angle > 180:
                    rotation_angle -= 360
                elif rotation_angle <= -180:
                    rotation_angle += 360
                
                cmd = "r " + str(abs(rotation_angle))
                cmd += " f" if rotation_angle > 0 else " t"
                print("New command:",cmd)
                
                if rotation_angle != 0:
                    send_msg(cmd)
                    incoming_msg = str(arduino.readline()[:-2])[2:-1]
                    while incoming_msg.startswith("Finish rotate") == False:
                        incoming_msg = str(arduino.readline()[:-2])[2:-1]
                    
                    if rotation_angle != 0:
                        rotation_angle /= abs(rotation_angle)
                        rotation_angle *= int(incoming_msg.split(' ')[2])

                print("Rotation was", int(rotation_angle))
                plotter.baseTh -= int(rotation_angle)
                plotter.mapTh += int(rotation_angle)
                print("New orientation is", plotter.baseTh)
                
                #Calculate distance between current location and new destination and send it to robot
                distance = round(plotter.points_distance(plotter.baseX, plotter.baseY, nextX, nextY) * 4)
                if distance < 200:
                    distance /= 4
                else:
                    distance -= 0.2 * distance
                cmd = "m " + str(distance) + " 1"
                print("New command:",cmd)
                send_msg(cmd)

                update_position(plotter, 1)

                #Update the map with new readings
                # send_msg("l 1")
            elif started == 1:
                # Rotate to get in initial pose
                rotation_angle = plotter.baseTh - 180
                cmd = "r " + str(abs(rotation_angle))
                cmd += " f" if rotation_angle > 0 else " t"
                print("Rotating to get initial pose", cmd)
                send_msg(cmd)

                started = 0
                print("Target achieved")
                plot_object(plotter.matrix, plotter.baseX, plotter.baseY, 14)
                send_msg("q")

#Clean noise from lidar around robot(we know for certain there is no obstacle where the robot lies)
def plot_object(matrix, baseX, baseY, value):
    for i in range(-31, 32):
        for j in range(-28, 29): 
            if matrix[baseX+i][baseY+j] != 7:
                matrix[baseX+i][baseY+j] = value
    plotter.matrice.set_array(matrix)
    plt.draw()
    plt.pause(0.001)
    
def update_position(plotter, direction):
    #Remove robot from old location on matrix
    plot_object(plotter.matrix, plotter.baseX, plotter.baseY, 0)

    #Wait for response from robot with actual traveled distance
    finish_action = ''
    while not finish_action.startswith("Distance"):
        finish_action = str(arduino.readline()[:-2])[2:-1]
        # print(finish_action)

    split = finish_action.split(":")
    print(split)
    
    #Get new orientation from the gyro
    new_orientation = ''
    while not new_orientation.startswith("Orientation"):    
        new_orientation = str(arduino.readline()[:-2])[2:-1]
    new_orientation = new_orientation.split(":")
    delta_th = round(float(new_orientation[1]))
    plotter.baseTh -= delta_th
    plotter.mapTh += delta_th
    print("New orientation delta", delta_th)

    #Get new observations of landmarks and use them to aproximate new position
    print("Waiting for new lidar reading")
    # print(datetime.datetime.now())
    send_msg("l 1")
    # print(datetime.datetime.now())
    while str(arduino.readline()[:-2])[2:-1] != 'Sending lidar readings':
        pass
    plotter.baseX, plotter.baseY, plotter.baseTh = plotter.get_position(direction * float(split[1]) / 4)

    print("Updating map")
    # print(datetime.datetime.now())
    plotter.update_map(0, False)
    # print(datetime.datetime.now())

#Deal with loss of bytes and retry until message was properly sent
def send_msg(line):
    line = add_checksum(line)
    response = "Send again"
    while response != "Success":
        if response == "Send again":
            arduino.write(line.encode())
        response = str(arduino.readline()[:-2])[2:-1]

#Xor checksum for message
def add_checksum(line):
    xor = 0
    for c in line:
        xor = ord(c) ^ xor
    line += chr(xor)

    return line

def signal_handler(sig, frame):
    print('Shutting down controller')
    send_msg("q")
    arduino.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
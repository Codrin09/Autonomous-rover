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
    arduino = serial.Serial("/dev/tty.NICE-BT-DevB", 115200, timeout = 2)
    arduino.flushInput() #This gives the bluetooth a little kick
except:
    print('Please check the port')
    sys.exit(0)

#Call main with any parameter for debug mode
def main(debug = None):

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
            message_received = str(arduino.readline()[:-2])[2:-1]
            if message_received:
                print(message_received)

                #!debug(messsage_received)

                if message_received == "Create new path":
                    print("Creating new path")
                    clean_noise(plotter.matrix, plotter.baseX, plotter.baseY)
                    path = lee.motion_planning(plotter.baseX, plotter.baseY, plotter.baseX + xDest, plotter.baseY + yDest, plotter.matrix)
                    print(path)
                elif message_received == "Sending lidar readings":
                    plotter.update_map(0)
                    plt.draw()
                    plt.pause(0.001)
            #If robot didn't complete path do next move
            elif len(path) > 0:
                #Get new intermediate destination
                nextX, nextY = path.pop(0)

                #Calculate rotation angle for robot
                rotation_angle = round(math.degrees(math.atan2(nextX - plotter.baseX, nextY - plotter.baseY))) - plotter.baseTh
                if rotation_angle > 180:
                    rotation_angle -= 360
                elif rotation_angle < -180:
                    rotation_angle += 360
                
                cmd = "r " + str(abs(rotation_angle))
                cmd += " false" if rotation_angle > 0 else " true"
                print("New command:",cmd)
                send_msg(cmd)

                while str(arduino.readline()[:-2])[2:-1] != "Finish rotate":
                    pass
                plotter.baseTh += rotation_angle
                
                #Calculate distance between current location and new destination and send it to robot
                distance = round(plotter.points_distance(plotter.baseX, plotter.baseY, nextX, nextY) * 4) 
                cmd = "m " + str(distance)
                print("New command:",cmd)
                send_msg(cmd)

                #Wait for response from robot with actual traveled distance
                finish_action = ''
                while not finish_action.startswith("Distance"):
                    finish_action = str(arduino.readline()[:-2])[2:-1]
                    if finish_action:
                        print(finish_action)

                split = finish_action.split(":")
                print(split)
                
                #Get new orientation from the gyro
                new_orientation = ''
                while not new_orientation.startswith("Orientation"):
                    new_orientation = str(arduino.readline()[:-2])[2:-1]
                new_orientation = new_orientation.split(":")
                delta_th = round(float(new_orientation[1]))
                plotter.baseTh += delta_th
                print("New orientation delta", delta_th)

                #Get new observations of landmarks and use them to aproximate new position
                send_msg("l 1")
                while str(arduino.readline()[:-2])[2:-1] != 'Sending lidar readings':
                    pass
                plotter.baseX, plotter.baseY, plotter.baseTh = plotter.get_position(float(split[1]) / 4)
                print("New position", plotter.baseX, plotter.baseY, plotter.baseTh)

                #Update the map with new readings
                send_msg("l 3")

#Clean noise from lidar around robot(we know for certain there is no obstacle where the robot lies)
def clean_noise(matrix, baseX, baseY):
    for i in range(-31, 32):
        for j in range(-29, 30): 
            matrix[baseX+i][baseY+j] = 0

#Deal with loss of bytes and retry until message was properly sent
def send_msg(line):
    line = add_checksum(line)
    response = None
    while response != "Success":
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
    arduino.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
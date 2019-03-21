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

"""Opening of the serial port"""
try:
    # arduino = serial.Serial("/dev/tty.usbmodem14101", 115200)
    arduino = serial.Serial("/dev/tty.NICE-BT-DevB", 115200, timeout = 2)
    arduino.flushInput() #This gives the bluetooth a little kick
except:
    print('Please check the port')
    sys.exit(0)

def main():
    print("1 - Powering motors controlled by PID\n2 - Powering motors at maximum speed without PID")
    print("3 - Turn 45 degrees anti clockwise\n4 - Turn 90 degrees clockwise")
    print("5 - Turn 45 degrees clockwise\n6 - Turn 90 degrees clockwise")
    print("7 - Testing")
    print("8 - Start scanning and mapping")

    plotter.init_variables(arduino)   
    plotter.init_map(arduino)
    last_line = None
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
                    path = lee.motion_planning(plotter.baseX, plotter.baseY, 1050, 1050, plotter.matrix)
                    print(path)
                elif message_received == "Sending lidar readings":
                    plotter.update_map(0)
                    plt.draw()
                    plt.pause(0.001)
            elif len(path) > 0:
                nextX, nextY = path.pop(0)

                rotation_angle = round(math.degrees(math.atan2(nextY - plotter.baseY, nextX - plotter.baseX))) - plotter.baseTh
                cmd = "r " + str(abs(rotation_angle))
                cmd += " 0" if rotation_angle > 0 else " 1"
                print("New command:",cmd)
                send_msg(cmd)

                while str(arduino.readline()[:-2])[2:-1] != "Finish rotate":
                    pass
                plotter.baseTh += rotation_angle
                
                distance = round(plotter.points_distance(plotter.baseX, plotter.baseY, nextX, nextY) * 4) 
                cmd = "m " + str(distance)
                print("New command:",cmd)
                send_msg(cmd)

                finish_action = ''
                while not finish_action.startswith("Distance"):
                    finish_action = str(arduino.readline()[:-2])[2:-1]

                print("Finish action:", finish_action)

                split = finish_action.split(":")
                print(split)
                send_msg('l')
                plotter.baseX, plotter.baseY, plotter.baseTh = plotter.get_position(float(split[1]) / 4)
                print("New position", plotter.baseX, plotter.baseY, plotter.baseTh)

                send_msg('l')

def send_msg(line):
    line = add_checksum(line)
    response = None
    while response != "Success":
        arduino.write(line.encode())
        response = str(arduino.readline()[:-2])[2:-1]

def add_checksum(line):
    xor = 0
    for c in line:
        xor = ord(c) ^ xor
    line += chr(xor)

    return line

def debug(message_received):
    # print(re.sub(r'[^\w]', ' ', str(message_received)))
    if message_received == b'Sending lidar readings\r\n':
        print("Received ladar readings")

        plotter.update_map(0)
        plt.draw()
        plt.pause(0.001)
    elif message_received == b'Get position\r\n':
        arduino.write(input("Press 1 after position moved ").encode())

        while arduino.readline() != b'Sending lidar readings\r\n':
            pass
        print("Getting current position")
        print(datetime.datetime.now())
        (newX, newY, newTh) = plotter.get_position(1)
        plotter.baseX = newX
        plotter.baseY = newY
        plotter.baseTh = newTh

        print("Done getting position")
        print(datetime.datetime.now())
        # print(newX, newY)
    elif message_received == b'New orientation\r\n':
        message_received = str(arduino.readline())[:-3].replace("\\r", "")
        print(message_received)
        print("New orientation received with delta", float(message_received[2:]))
        plotter.baseTh +=  round(float(message_received[2:]))
    elif message_received == b'Create new path\r\n':
        print("Path planning")
        path = lee.motion_planning(plotter.baseX, plotter.baseY, 1500, 1500, plotter.matrix)
        print(path)
    elif message_received == b'Rotate\r\n':
        print("Received rotation")
        #! clockwise/anti clock-ws: +/-
        plotter.baseTh -= 45
    elif message_received == b'Send again\r\n':
        arduino.write(last_line.encode())
    else:
        print(message_received)
def signal_handler(sig, frame):
    print('Shutting down controller')
    arduino.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
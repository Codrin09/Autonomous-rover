import sys
import signal
import serial
import select 
import plotter
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import datetime

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
    print("8 - Start scanning and mapping")

    plotter.init_variables()   
    plotter.init_map(arduino)

    plt.ion()
    plt.show()

    while True:
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if line:
                arduino.write(line.encode())
            else: # an empty line means stdin has been closed
                print('eof')
                exit(0)
        else:
            message_received = arduino.readline()
            if message_received is not b'':
                # print(re.sub(r'[^\w]', ' ', str(message_received)))
                if message_received == b'Sending ladar readings\r\n':
                    print("Received ladar readings")
                    print(datetime.datetime.now())

                    plotter.update_map(0, arduino)

                    print("done updating")
                    print(datetime.datetime.now())
                    plt.draw()
                    plt.pause(0.001)
                    print("done drawing")
                    print(datetime.datetime.now())
                else:
                    # pass
                    print(message_received)
        # line = sys.stdin.readline()
        # arduino.write(line.encode())
        
        # message_received = arduino.readline()
        # if message_received is not None:
        #     print(message_received)

def signal_handler(sig, frame):
    print('Shutting down')
    arduino.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
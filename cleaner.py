import sys
import signal
import serial

"""Opening of the serial port"""
try:
    # arduino = serial.Serial("/dev/tty.usbmodem14101", 115200)
    arduino = serial.Serial("/dev/tty.NICE-BT-DevB", 115200, timeout = 2)
    arduino.flushInput() #This gives the bluetooth a little kick
except:
    print('Please check the port')
    sys.exit(0)

def main():
    print("Cleaning bluetooth data sent from the arduino")
    while True:
        line = arduino.readline()
        if line is not b'':
            print(line)
        else:
            print("Finished cleaning, exiting!")
            arduino.close()
            break

def signal_handler(sig, frame):
    print('Shutting down')
    arduino.close() #Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
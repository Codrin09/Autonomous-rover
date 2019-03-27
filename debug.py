#Debug class
def debug_controls():
    print("1 - Powering motors controlled by PID\n2 - Powering motors at maximum speed without PID")
    print("3 - Turn 45 degrees anti clockwise\n4 - Turn 90 degrees clockwise")
    print("5 - Turn 45 degrees clockwise\n6 - Turn 90 degrees clockwise")
    print("7 - Testing")
    print("8 - Start scanning and mapping")

def debug(message_received):
    # print(re.sub(r'[^\w]', ' ', str(message_received)))
    if message_received == b'Sending lidar readings\r\n':
        print("Received lidar readings")

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
    else:
        print(message_received)
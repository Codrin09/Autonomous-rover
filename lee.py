import datetime
from plotter import valid_point
#  Dimensions of rover 245 x 225
def check_crash(x, y, matrix):
    #Used -31 32, -29 30
    #Last -33 34 -30 31
    for i in range(-31, 32, 4):
        for j in range(-28, 29, 4):
            if valid_point(x+i, y+j) and matrix[x+i][y+j] != 7:
                continue 
            return True
    return False

def path_check(currentX, currentY, destX, destY, matrix):
    minX, maxX = min(currentX, destX), max(currentX, destX)
    minY, maxY = min(currentY, destY), max(currentY, destY)
    a, b, c = line_equation(currentX, currentY, destX, destY)

    crashed = False
    if maxX - minX >= maxY - minY:
        for x in range (minX, maxX, 28):
            y = int((-1) * (a * x + c) / b)
            if check_crash(x, y, matrix):
                crashed = True
                break;
        x = maxX
        y = int((-1) * (a * x + c) / b)
        if check_crash(x, y, matrix):
            crashed = True
    else:
        for y in range (minY, maxY, 28):
            x = int((-1) * (b * y + c) / a)
            if check_crash(x, y, matrix):
                crashed = True
                break;
        y = maxY
        x = int((-1) * (b * y + c) / a)
        if check_crash(x, y, matrix):
            crashed = True   

    return crashed

def motion_planning(startX, startY, goalX, goalY, matrix):
    global step
    queue = []
    queue.append((startX, startY))

    step = 50
    
    leeMatrix = [[0 for col in range(2001)] for row in range(2001)]

    stop = False
    print("Start lee", datetime.datetime.now())
    while len(queue) > 0:
        currentX, currentY = queue.pop(0)
        # print(currentX, currentY)
        for i in range(-1,2):
            if stop:
                break
            for j in range(-1,2):
                destX = currentX + i * step
                destY = currentY + j * step
                if(valid_point(destX, destY)) and (destX != currentX or destY != currentY):
                    
                    crashed = path_check(currentX, currentY, destX, destY, matrix)
 
                    if crashed == False and (destX != startX or destY != startY) and (leeMatrix[destX][destY] == 0 or (leeMatrix[currentX][currentY] + 1) < leeMatrix[destX][destY]):
                        dist = 0
                        if i * j == -1 or i * j == 1:
                            dist = 1.41
                        else:
                            dist = 1
                        leeMatrix[destX][destY] = leeMatrix[currentX][currentY] + dist
                        if abs(destX - goalX) < step and abs(destY - goalY) < step:
                            stop = True
                            goalX = destX
                            goalY = destY
                            break
                        queue.append((destX, destY))
    
    print("Finish lee", datetime.datetime.now())

    # print(leeMatrix[goalX][goalY])
    path = retrieve_path(leeMatrix, startX, startY, goalX, goalY)
    return path

def retrieve_path(leeMatrix, startX, startY, goalX, goalY):
    global step
    path = []
    currentX, currentY = goalX, goalY
    path.append((goalX, goalY))
    while currentX != startX or currentY != startY:
        min_distance = leeMatrix[currentX][currentY]
        newX = newY = 0
        for i in range(-1,2):
            for j in range(-1,2):
                destX = currentX + i * step
                destY = currentY + j * step

                if leeMatrix[destX][destY] < min_distance and leeMatrix[currentX][currentY] - leeMatrix[destX][destY] < 2 and ((destX == startX and destY == startY) or leeMatrix[destX][destY] > 0):                        
                    newX, newY = destX, destY
                    min_distance = leeMatrix[destX][destY]
        if newX == 0 and newY == 0:
            print("Failed retrieving path")
            return []
        # print("path lee", leeMatrix[newX][newY])
        path.append((newX, newY))
        currentX, currentY = newX, newY
    
    path.pop(len(path) - 1)
    path.reverse()
    return path

def line_equation(currentX, currentY, destX, destY):
    a = currentY - destY
    b = destX - currentX
    c = currentX * (destY - currentY) + currentY * (currentX - destX)

    if currentY == destY:
        a = 0
        b = -1
        c = currentY
    if currentX == destX:
        a = -1
        b = 0
        c = currentX
    
    return (a, b, c)

if __name__ == "__main__":
    matrix = [[0 for col in range(2001)] for row in range(2001)]
    path = motion_planning(1000,1000, 900, 1000, matrix)
    print(path)

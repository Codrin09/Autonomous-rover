import datetime
from plotter import valid_point
#  Dimensions of rover 245 x 225
def check_crash(x, y, matrix):
    for i in range(-31, 32, 6):
        for j in range(-29, 30, 6):
            if valid_point(x+i, y+j) and matrix[x+i][y+j] == 0:
                continue 
            return True
    return False

def motion_planning(startX, startY, goalX, goalY, matrix):
    queue = []
    queue.append((startX, startY))

    padding_min = 10
    
    leeMatrix = [[0 for col in range(2001)] for row in range(2001)]

    stop = False
    print(datetime.datetime.now())
    while len(queue) > 0:
        currentX, currentY = queue.pop(0)
        # print(currentX, currentY)
        for i in range(-1,2):
            if stop:
                break
            for j in range(-1,2):
                destX = currentX + i * 50
                destY = currentY + j * 50
                if(valid_point(destX, destY)):
                    minX, maxX = min(currentX, destX), max(currentX, destX)
                    minY, maxY = min(currentY, destY), max(currentY, destY)
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

                    crashed = False
                    if maxX - minX >= maxY - minY:
                        for x in range (minX + padding_min, maxX, 10):
                            y = int((-1) * (a * x + c) / b)
                            if check_crash(x, y, matrix):
                                crashed = True
                                break;
                    else:
                        for y in range (minY + padding_min, maxY, 10):
                            x = int((-1) * (b * y + c) / a)
                            if check_crash(x, y, matrix):
                                crashed = True
                                break;
                    if not crashed and (destX != startX or destY != startY) and (leeMatrix[destX][destY] == 0 or (leeMatrix[currentX][currentY] + 1) < leeMatrix[destX][destY]):
                        dist = 0
                        if i * j == -1 or i * j == 1:
                            dist = 1.41
                        else:
                            dist = 1
                        leeMatrix[destX][destY] = leeMatrix[currentX][currentY] + dist
                        if destX == goalX and destY == goalY:
                            stop = True
                        queue.append((destX, destY))

    print(datetime.datetime.now())

    # print(leeMatrix[goalX][goalY])
    path = retrieve_path(leeMatrix, startX, startY, goalX, goalY)
    return path

def retrieve_path(leeMatrix, startX, startY, goalX, goalY):
    path = []
    currentX, currentY = goalX, goalY
    path.append((goalX, goalY))
    while currentX != startX or currentY != startY:
        min_distance = leeMatrix[currentX][currentY]
        newX = newY = 0
        for i in range(-1,2):
            for j in range(-1,2):
                destX = currentX + i * 50
                destY = currentY + j * 50

                if leeMatrix[destX][destY] < min_distance and (destX == startX and destY == startY or leeMatrix[destX][destY] > 0):
                    newX, newY = destX, destY
                    min_distance = leeMatrix[destX][destY]
        path.append((newX, newY))
        currentX, currentY = newX, newY
    
    path.pop(len(path) - 1)
    path.reverse()
    return path

if __name__ == "__main__":
    matrix = [[0 for col in range(2001)] for row in range(2001)]
    path = motion_planning(1000,1000, 900, 1000, matrix)
    print(path)

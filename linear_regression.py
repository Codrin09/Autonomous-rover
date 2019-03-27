def linear_fit(x, y):
    """For set of points `(xi, yi)`, return linear polynomial `f(x) = k*x + m` that
    minimizes the sum of quadratic errors.
    """
    k = m = -1
    try:
        meanx = sum(x) / len(x)
        meany = sum(y) / len(y)
        #If all values in x are the same xi-meanx will be 0 so we need to fix that(e.g set it to infinity)
        for xi in x:
            if xi == meanx:
                meanx += 1
        k = sum((xi-meanx)*(yi-meany) for xi,yi in zip(x,y)) / sum((xi-meanx)**2 for xi in x)
        m = meany - k*meanx
    except Exception as e:
        pass
    return k, m

#Use newly created edges found using linear regression
def linear_reg_draw(cluster, baseTh):
    oldVal = 0
    xCoords = []
    yCoords = []
    startX = startY = 0
    for angle in range(360):
        real_angle = (angle + baseTh) % 360
        if cluster[real_angle] != 0:
            newX, newY = new_points[index]

            if oldVal == 0:
                oldVal = cluster[real_angle]
                startX, startY = newX, newY
            elif oldVal != cluster[real_angle]:
                #Take only first and last points that define an edge and draw all point between them 
                # draw_line(startX, startY, lastX, lastY, "create", cluster[real_angle])

                #Or use linear regression to create a line that fits points assigned to edge to draw
                k, m = linear_fit(xCoords, yCoords)
                for x in xCoords:
                    y = round(k * x + m)
                    edit_point(x, y, "create", cluster[real_angle])
                xCoords = []
                yCoords = []                
                startX, startY = newX, newY
                oldVal = cluster[real_angle]
            elif real_angle == 359:
                #Take only first and last points that define an edge and draw all point between them 
                # draw_line(startX, startY, newX, newY, "create", cluster[real_angle])

                #Or use linear regression to create a line that fits points assigned to edge to draw
                k, m = linear_fit(xCoords, yCoords)
                for x in xCoords:
                    y = round(k * x + m)
                    edit_point(x, y, "create", cluster[real_angle])

            xCoords.append(newX)
            yCoords.append(newY)

            lastX, lastY = newX, newY
            index += 1
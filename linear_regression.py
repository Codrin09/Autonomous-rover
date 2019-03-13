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



if __name__ == "__main__":
    x = [1,2,3,4,6,7]
    y = [5,4,3,2,-2,-1]
    a , b = linear_fit(x, y)

    sampleY = a * 3 + b
    print(sampleY)
def ana(y):
    global x
    x *= 2
    y *= 2
    print(y)
    print(x)

if __name__ == "__main__":
    """Opening of the serial port"""
    x = "ana"
    y = 10
    ana(y)
    print(y)
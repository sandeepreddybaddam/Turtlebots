import math
import numpy as np

def angle(x1, y1, x2, y2):
    xd = x1 - x2
    yd = y1 - y2
    f = (y2-y1)/math.sqrt(xd*xd +yd*yd)
    return np.arcsin(f) #radians
a = angle(0, 0, 1, -1)
print(a*180/3.14)
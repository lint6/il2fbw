import numpy as np

def invert(Mat):
    return np.linalg.inv(Mat)

def backwardDiff(x, h):
    if len(x) >= 2:
        return (x[-1]-x[-2])/h
    else:
        return np.zeros(np.array(x).shape)
    
def backwardDiff_2Order(x, h):
    if len(x) >= 3:
        return (3.0*x[-1]-4.0*x[-2]+x[-3])/(2.0*h)
    else:
        return np.zeros(np.array(x).shape)
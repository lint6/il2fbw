import numpy as np

def invert(Mat):
    try:    
        np.linalg.inv(Mat)
    except:
        return np.linalg.pinv(Mat)
    else:
        return np.linalg.inv(Mat)

def backwardDiff(x, h):
    x = np.array(x)
    if len(x) >= 2:
        return np.array((x[-1]-x[-2])/h).flatten()
    else:
        return np.zeros(np.array(x).shape).flatten()
    
def backwardDiff_2Order(x, h):
    x = np.array(x)
    if len(x) >= 3:
        return np.array((3.0*x[-1]-4.0*x[-2]+x[-3])/(2.0*h)).flatten()
    else:
        return np.zeros(x[-1].shape).flatten()
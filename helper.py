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
    
def rotation(angle):
    '''
    World to Body rotation
    '''
    roll = angle[0]
    pitch = angle[1]
    yaw = angle[2]
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr,  cr]])

    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])

    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    
    R = Rz @ Ry @ Rx
    return angle, R
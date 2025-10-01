import numpy as np
import helper 
def BmatEstimation(omega, u, h):
    '''
    Uses numerical differenation to calulcate the current effective matrix
    '''
    domega = helper.backwardDiff_2Order(omega, h)
    dL = domega[0]
    dM = domega[1]
    dN = domega[2]
    du = helper.backwardDiff_2Order(u, h)
    da = du[0]
    de = du[1]
    dr = du[2]
    eps = 1e-16
    Bmat = np.array( [[dL/(da+eps), dL/(de+eps), dL/(dr+eps)],
                      [dM/(da+eps), dM/(de+eps), dM/(dr+eps)],
                      [dN/(da+eps), dN/(de+eps), dN/(dr+eps)]])
    return Bmat
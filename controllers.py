import numpy as np
import helper

def toRate(RateGain, stick):
    '''
    RateGain: [roll, pitch, sideslip]
    '''
    return np.deg2rad(RateGain) * stick[0:3]
    
def LC_pid(error, error_int, LCgain, h):
    '''
    error: 3x3 matrix of past three error per channel
    LCgain: array of gain
    '''
    vc_p = error[-1] * LCgain[0]
    vc_i = error_int * LCgain[1]
    vc_d = helper.backwardDiff(error, h) * LCgain[2]
    return vc_p + vc_d

def LC_pi(error, error_int, LCgain):
    '''
    error: 3x3 matrix of past three error per channel
    LCgain: array of gain
    '''
    vc_p = error * LCgain[0]
    vc_i = error_int * LCgain[1]
    return vc_p + vc_i

def INDI(vc, x, u, Bmat, h):
    '''
    Rate: 3x1 vector of current aircraft rate
    input: 3x1 vector of desired aircraft rate
    '''
    e_dot = vc - helper.backwardDiff(x, h).flatten()
    du = helper.invert(Bmat) @ e_dot.flatten()
    u += du
    return u

def controlLoopINDI(x, u, error_int, input, h, LCgain, Bmat):
    '''
    x: deque of state (Omega)
    u: current control
    input: deque of input (Omega_set)
    h: time step
    hist: buffer array of past state
    '''
    x = np.array(x)
    input = np.array(input)
    error = input - x
    vc = LC_pid(error, error_int, LCgain, h)
    u = INDI(vc, x, u, Bmat, h)
    u = np.clip(u, -1, 1)
    return u

def controlLoopRate(x, error_integral, input, LCgain):
    '''
    x: deque of state (Omega)
    u: current control
    input: deque of input (Omega_set)
    h: time step
    hist: buffer array of past state
    '''
    x = np.array(x)
    input = np.array(input)
    error = np.array(np.array(input) - np.array(x))[0]
    u = LC_pi(error, error_integral, LCgain)
    u = np.clip(u, -1, 1)
    return u
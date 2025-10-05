import numpy as np
import helper

def toRate(RateGain, sideslip, stick):
    '''
    RateGain: [roll, pitch, Sideforce]
    '''
    rate_set = RateGain * stick[0:3]
    rate_set[-1] = -1*(rate_set[-1] - 800*sideslip)
    rate_set[-1] = np.clip(rate_set[-1], -30, 30)
    return np.deg2rad(rate_set)
    
def LC_pd(error, LCgain, h):
    '''
    error: 3x3 matrix of past three error per channel
    LCgain: array of gain
    '''
    vc_p = error[-1] * LCgain[0]
    vc_d = helper.backwardDiff(error, h) * LCgain[1] * LCgain[0]
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
    omega_dot = helper.backwardDiff(x, h)
    e_dot = vc - omega_dot
    du = helper.invert(Bmat) @ e_dot
    u += du
    u = np.clip(u, -1, 1)
    return u, du, omega_dot

def controlLoopINDI(x, u, AoA, input, h, LCgain, Bmat):
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
    vc = LC_pd(error, LCgain, h)
    u, du, omega_dot = INDI(vc, x, u, Bmat, h)
    return u, du, [vc, omega_dot]

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
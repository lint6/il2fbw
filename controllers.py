import numpy as np
import helper

def toRate(RateGain, sideslip, stick):
    '''
    RateGain: [roll, pitch, Sideforce]
    '''
    rate_set = RateGain * stick[0:3]
    sideslip_damp = np.clip(-1*sideslip, -5, 5)
    raw_rate = rate_set[-1]
    rate_set[-1] = np.clip(sideslip_damp+raw_rate, -30, 30)
    return np.deg2rad(rate_set)

def aoaLimiter(value, control_var, limit = [-10, 15], slope = -150):
    '''
    value: value to be limited
    limit: limits
    control_var: the variable used to limit
    slope: the slope of the limiter function
    '''
    
    limit = np.deg2rad(limit)
    control_var = np.clip(control_var, np.min(limit)-0.15, np.max(limit)+0.15)
    value[1] = np.clip(value[1],
                    slope*(control_var-np.min(limit))**3,
                    slope*(control_var-np.max(limit))**3)
    return value
    
def LC_pd(error, LCgain, h):
    '''
    error: 3x3 matrix of past three errord per channel
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

def INDI(vc, x, aoa, u, Bmat, h):
    '''
    Rate: 3x1 vector of current aircraft rate
    input: 3x1 vector of desired aircraft rate
    '''
    omega_dot = helper.backwardDiff(x, h)
    # vc = aoaLimiter(vc, aoa)
    e_dot = vc - omega_dot
    du = helper.invert(Bmat) @ e_dot
    du = np.clip(du, -0.3, 0.3)
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
    u, du, omega_dot = INDI(vc, x, AoA, u, Bmat, h, )
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
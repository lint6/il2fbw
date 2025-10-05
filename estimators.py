import numpy as np
import helper

def accelerationCleaner(rotation, acceleration):
    '''
    Removes gravity from the acceleration
    '''
    gravity = [0,0,9.80665] 
    gravity = rotation.T @ gravity
    acceleration_clean = acceleration - gravity
    acceleration_world = rotation @ acceleration
    return acceleration, acceleration_clean, acceleration_world
    
def airspeedEstimator(omega, acceleration, velocity, dt=1/50):
    '''
    Finds turn radius and velocity
    '''
    weight = np.linspace(0.1, 1, len(omega))
    acceleration_avg = np.average(acceleration, axis=0, weights=weight)
    omega_avg = np.average(omega, axis=0, weights=weight)+1e-16
    
    omega_weight = np.clip((omega_avg/0.3)**2, 1e-16, 1.0)
    
    # Rotation Based 
    Vx_az = np.abs(acceleration_avg[2]/omega_avg[1])
    Vx_ay = np.abs(acceleration_avg[1]/omega_avg[2])
    Vz_ax = np.abs(acceleration_avg[0]/omega_avg[1])
    Vz_ay = np.abs(acceleration_avg[1]/omega_avg[0])
    
    Vx_rot = np.average([Vx_az, Vx_ay], weights=[omega_weight[1], omega_weight[2]])
    Vz_rot = np.sign(omega_avg[1])*np.average([Vz_ax, Vz_ay], weights=[omega_weight[1], omega_weight[0]])
    V_rot = [Vx_rot, 0.0, Vz_rot]
    
    
    # Numerical Integration
    V_num = (velocity + acceleration_avg * dt) * [1, 0, 1]

    # Averaging out
    omega_weight_sum = np.array([np.clip(omega_weight[1]+omega_weight[2], 1e-16, 1), 0, np.clip(omega_weight[1]+omega_weight[0], 1e-16, 1)])

    # Absolute Diff weight
    abs_weight = np.clip(0.8 / np.linalg.norm(V_rot - V_num), 1e-16, 1)
    V_avg = np.average([V_rot, V_num], axis=0, weights=[abs_weight*omega_weight_sum, 1-abs_weight*omega_weight_sum])
    V_avg = np.clip(V_avg, [25, 0, -100], [400, 0, 100])
    
    return V_avg

def aoaEstimator(V, acc, CLa = 2*np.pi, area=20, mass = 3000):
    AoA = -1*(2 * acc[-1][-1] * mass) / (CLa * V[0]**2 * area)
    sideslip = -1*(2 * acc[-1][1] * mass) / (CLa * V[0]**2 * area)
    sideslip = np.clip(sideslip, -25, 25)
    return AoA, sideslip

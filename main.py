'''Main Python file'''

import pyvjoy
import time
from collections import deque
import numpy as np

# Companion Files
import update
import controllers
import stick
import telemetry
import helper
from RLS import RLS

VirtualStick = pyvjoy.VJoyDevice(1)
RateGain = [120, 60, 10] # roll rate, pitch rate, yaw rate  
PIDgain = np.array([[250.0, 15.0, 50.0],    # Roll [kp, kd]
                    [500.0, 30.0, 25.0],    # Pitch
                    [200.0, 120.0, 50.0]]).T # Yaw
PIgain = np.array([[0.8, 0.001],    # Roll [kp, ki]
                   [0.7, 0.001],    # Pitch
                   [0.1, 0.001]]).T # Yaw
settingMode = False # Set to True to use the setting mode to change key mapping
debugMode = False # Forces the script to run regardless of IL-2 status
INDI = True # Set to True to use estimated INDI controller, may be unstable
if INDI:
    L_estimator = RLS(size=5)
    M_estimator = RLS(size=3)
    N_estimator = RLS(size=5)

u = np.array([0.0, 0.0, 0.0])

Running = True
omega = deque(maxlen=5)
angle = deque(maxlen=5)
omega_set = deque(maxlen=5)
u_hist = deque(maxlen=5)
error_integral = np.array([0.0, 0.0, 0.0])
control_stick = stick.detectStick()
aircraft_data = None
t0 = time.time()
debug_var = 0


if settingMode:
    t = 0
    print('----Setting Mode----')
    while settingMode:
        x = 0
        y = 0
        z = np.sin(t)
        t += 0.1
        update.vjoyUpdate(x, y, z, VirtualStick)
        time.sleep(1/50)

print('----Controller Running----')
if INDI:
    print('INDI')
else:
    print('Rate')
    
while Running:
    if not debugMode:
        aircraft_data = telemetry.reciveData()
    if aircraft_data or debugMode:
        dt = np.clip(time.time() - t0, 0.015, 0.025)
        t0 = time.time()
        input = np.array(stick.getStickInput(control_stick))
        if debugMode:
            print('____Loop Num:', int(debug_var*100), '____')
            omega_set.append(np.array([np.sin(debug_var+0.1), np.cos(debug_var+0.1), -np.sin(debug_var+0.1)]).flatten())
            omega.append(0.5*np.array([[np.sin(debug_var), np.cos(debug_var), -np.sin(debug_var)]]).flatten())
            debug_var += 0.01
            time.sleep(dt)
        else:
            omega_set.append(controllers.toRate(RateGain, input))
            omega.append(aircraft_data[5:8]*np.array([1, -1, -1]))
        error_integral += np.array(omega_set).flatten()[-1] - np.array(omega).flatten()[-1]

        if INDI: # INDI control
            '''RLS'''
            '''Technically this RLS is running at one loop behind the telemetry'''
            # learning = False
            # if learning:
            #     u = input[:3]
            # if len(omega) >= 2:
            #     omega_last = np.array(omega[-2]).flatten()
            # else:
            #     omega_last = np.array([0.0, 0.0, 0.0]).flatten()
            # LN_regression_vector = np.array([1, omega_last[0], omega_last[2], u[0], u[2]])
            # M_regression_vector = np.array([1, omega_last[1], u[1]])
            #                     # [1, p, q, r, da, de, dr]
            # target_vector = helper.backwardDiff(omega, dt)

            # L_estimator.Interate(LN_regression_vector, target_vector[0])
            # M_estimator.Interate(M_regression_vector,  target_vector[1])
            # N_estimator.Interate(LN_regression_vector, target_vector[2])

            Bmat = np.array([[2500, 0, 15],
                             [0, 5000, 0],
                             [-250, 0, 750]
                             ])

            '''INDI'''
            u = controllers.controlLoopINDI(omega, u, error_integral, omega_set, dt, PIDgain, Bmat)
            
        else: # Standard Rate Control
            u = controllers.controlLoopRate(omega, error_integral, omega_set, PIgain)
        
        try:
            for i in u:
                int(i)
        except:
            print('NaN control detected, resetting control')
            Bmat = np.array( [[1,0,0],
                              [0,1,0],
                              [0,0,1]])
            u = np.array([0.0,0.0,0.0])
        
        update.vjoyUpdate(u[0], u[1], u[2], VirtualStick)
        
print('----Controller Stopped----')
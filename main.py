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
RateGain = [120, 90, 10] # roll rate, pitch rate, yaw rate
PDgain = np.array([[1, 0.0],    # Roll [kp, kd]
                   [1, 0.0],    # Pitch
                   [1, 0.0]]).T # Yaw
PIgain = np.array([[0.8, 0.001],    # Roll [kp, ki]
                   [0.7, 0.001],    # Pitch
                   [0.1, 0.001]]).T # Yaw
settingMode = False # Set to True to use the setting mode to change key mapping
INDI = False # Set to True to use estimated INDI controller, may be unstable
if INDI:
    L_estimator = RLS(size=7)
    M_estimator = RLS(size=7)
    N_estimator = RLS(size=7)

u = np.array([0.0, 0.0, 0.0])
Bmat = np.array( [[1,0,0],
                  [0,1,0],
                  [0,0,1]])
Running = True
omega = deque(maxlen=5)
angle = deque(maxlen=5)
omega_set = deque(maxlen=5)
u_hist = deque(maxlen=5)
error_integral = np.array([0.0, 0.0, 0.0])
control_stick = stick.detectStick()
t0 = time.time()


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
    print('Using INDI')
else:
    print('Using Rate Controller')
    
while Running:
    aircraft_data = telemetry.reciveData()
    if aircraft_data:
        dt = np.clip(time.time() - t0, 0.015, 0.025)
        t0 = time.time()
        input = np.array(stick.getStickInput(control_stick))
        omega_set.append(controllers.toRate(RateGain, input))
        omega.append(aircraft_data[5:8] * np.array([1, -1, -1]))
        error_integral += np.array(omega_set)[-1] - np.array(omega)[-1]

        if INDI: # INDI control
            '''RLS'''
            '''Technically this RLS is running at one loop behind the telemetry'''
            if len(omega) >= 2:
                omega_last = np.array(omega[-2])
            else:
                omega_last = np.array([0, 0, 0])
            regression_vector = np.array([1, omega_last[0], omega_last[1], omega_last[2], u[0], u[1], u[2]])
                                # [1, p, q, r, da, de, dr]
            target_vector = helper.backwardDiff(omega, dt)
            L_estimator.Interate(regression_vector, target_vector[0])
            M_estimator.Interate(regression_vector, target_vector[1])
            N_estimator.Interate(regression_vector, target_vector[2])
            Bmat = np.array([[L_estimator.theta[-3], L_estimator.theta[-2], L_estimator.theta[-1]],
                             [M_estimator.theta[-3], M_estimator.theta[-2], M_estimator.theta[-1]],
                             [N_estimator.theta[-3], N_estimator.theta[-2], N_estimator.theta[-1]]
                             ])
            
            print('Estimated Control Matrix:', Bmat)
                                
            '''INDI'''
            u = controllers.controlLoopINDI(omega, u, omega_set, dt, PDgain, Bmat)
            
        else: # Standard Rate Control
            u = controllers.controlLoopRate(omega, error_integral, omega_set, PIgain)
        
        try:
            for i in u:
                int(i)
        except:
            print('NaN control detected, resetting control')
            u = np.array([0.0,0.0,0.0])
        
        update.vjoyUpdate(u[0], u[1], u[2], VirtualStick)
        
print('----Controller Stopped----')
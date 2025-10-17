'''Keeps aircraft as a class'''

import numpy as np
from collections import deque
import estimators
import helper
import controllers

class Aircraft():
    def __init__(self, RateGain, PDgain, B, mass = 3000, aoa_lim = [-10, 15], g_lim = [-3, 6], dt = 1/50):
        # States
        self.omega = deque(maxlen=3)
        self.omega_dot = []
        self.angle = deque(maxlen=3)
        self.acc_raw = deque(maxlen=3)
        self.acc_clean = deque(maxlen=3)
        self.acc_world = deque(maxlen=3)
        self.vel_body = [100.0, 0.0, 0.0]
        self.aoa = deque(maxlen=5)
        self.sideslip = deque(maxlen=5)
        
        # Controls
        self.RateGain = RateGain
        self.PDgain = PDgain
        self.omega_set = deque(maxlen=3)
        self.u = np.array([0.0, 0.0, 0.0])
        self.du = np.array([0.0, 0.0, 0.0])
        
        # Aircraft Charateristics
        self.B = B
        self.aoa_lim = np.deg2rad(aoa_lim) # aoa limiter
        self.g_lim = g_lim # g limiter
        self.mass = mass
        
        # Extra
        self.dt = dt
        self.mu = np.diag(1e-5*np.array([1,1,1]))
        
    def update(self, aircraft_data, stick_in):
        # Generate rotation matrix
        body_ang, R_eb = helper.rotation([aircraft_data[4],aircraft_data[3],aircraft_data[2]]*np.array([1, -1, -1]))
        
        # Append Current aircraft state
        self.omega.append(aircraft_data[5:8]*np.array([1, -1, -1]))
        self.angle.append(body_ang)
        acc = estimators.accelerationCleaner(R_eb, aircraft_data[8:]*np.array([1, -1, -1]), np.linalg.norm(self.vel_body))
        self.acc_raw.append(acc[0])
        self.acc_clean.append(acc[3]*self.mass)
        # self.acc_world.append(acc[2])
        
        # Estimate Aircraft State
        self.vel_body = estimators.airspeedEstimator(self.omega, self.acc_raw, self.vel_body)
        aoa, sideslip = estimators.aoaEstimator(self.vel_body, self.acc_clean)
        self.aoa.append(aoa)
        self.sideslip.append(sideslip)
        sideslip = np.average(self.sideslip, weights=np.linspace(0.1, 1, len(self.sideslip)))

        
        # Create setpoint
        self.omega_set.append(controllers.toRate(self.RateGain, self.acc_clean[-1][1], stick_in))

    def updateEffeciveness(self):
        airspeed = np.linalg.norm(self.vel_body)
        # print('airspeed: ', airspeed)
        self.effectiveness = 0.5 * airspeed**2  * self.B
    

    def adaptiveEffectiveness(self):
        du = self.du.reshape(-1,1)
        airspeed = np.linalg.norm(self.vel_body)
        effective = 0.5 * airspeed**2  * self.B
        error = (effective @ du - (helper.backwardDiff(self.omega, self.dt)*-1).reshape(-1,1))
        mu = self.mu
        B = (effective - mu * (error @ du.T)) / (0.5 * airspeed**2)
        self.B = B * [[1, 0, 1], [0, 1, 0], [1, 0, 1]]
        self.effectiveness = 0.5 * airspeed**2  * self.B
    
    def INDI(self):
        self.updateEffeciveness()
        # AoA limiter
        # self.aoaLimiter()
        aoa = np.average(self.aoa, weights=np.linspace(0.1, 1, len(self.aoa)))
        aoa *= np.clip(np.abs(self.omega[-1][1])*2, 0, 1)
        
        self.u, self.du, self.omega_dot = controllers.controlLoopINDI(self.omega, 
                                             self.u, 
                                             aoa,
                                             self.omega_set, 
                                             self.dt, 
                                             self.PDgain, 
                                             self.effectiveness,
                                             )

        return self.u

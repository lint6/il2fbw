'''
RLS w/o forgetting
'''

#TODO: implenment RLS funciton without forgetting factor
import numpy as np

# Function and classes
class RLS():
    def __init__(self, size, theta_hat = None, adaptive = True, tuning=15):
        print('RLS is forgetting')
        self.degree = size # Size of estimation vector
        if self.theta: 
            self.theta = theta_hat
        else:
            self.theta = np.zeros(self.degree)
            
        self.P = np.identity(self.degree)*1e4 # covariance matrix
        self.FF = 1.0 # forgetting factor
        self.FF_min = 0.1
        self.tuning = tuning
        self.adaptive = adaptive

        
    def Interate(self,a, y):
        '''
        Main Update loop
        a: regression vector
        y: target vector
        '''

        self.a = a # Update Regression Vector Value
        self.Update_L() # Update RLS gain
        self.Update_theta_hat(y) # Update estimation
        self.Update_P() # Update covariance matrix
        self.Updata_FF(y) # Update forgetting factor

        
    def Update_L(self):
        '''Calculate new RLS gain'''
        self.L = self.P @ self.a.T / (self.a @ self.P @ self.a.T + self.FF)
        
    def Update_theta_hat(self, y):
        self.theta = self.theta + (self.L @ (y - self.a @ self.theta))
        
    def Update_P(self):
        self.P = (1/self.FF) * (np.identity(self.degree) - self.L @ self.a) @ self.P
    
    def Updata_FF(self, y):
        FF = 1 - (1/self.tuning) * (1-self.a @ self.L) * (y - self.a @ self.theta)**2
        self.FF = min(max(FF, self.FF_min), 1)
'''Main Python file'''

import pyvjoy
import time
import numpy as np
import matplotlib.pyplot as plt

# Companion Files
import update
import stick
import telemetry
import aircraft

VirtualStick = pyvjoy.VJoyDevice(1)
RateGain = [60, 90, -25] # roll rate, pitch rate, Side force
PDgain = np.array([[16, 0.015],    # Roll [kp, Td]
                   [7, 0.01],    # Pitch
                   [3.5, 0.005]]).T # Yaw
Bmat = np.array([[0.065, 0, 0.001],
                 [0, 0.007, 0],
                 [0.0, 0, 0.009]
                 ])

FlightControl = aircraft.Aircraft(RateGain, PDgain, Bmat)
settingMode = False # Set to True to use the setting mode to change key mapping
tuningMode = False # Forces the script to run regardless of IL-2 status
Running = True

control_stick = stick.detectStick()
telemetry_data = None
debug_var = 0
maxTime = 5
tunning_graph = []
tunning_graph_dot = []


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

print('----Controller Ready----')
print('INDI Mode')

    
while Running:
    telemetry_data = telemetry.reciveData()
    if telemetry_data:
        stick_input = np.array(stick.getStickInput(control_stick))
        FlightControl.update(telemetry_data, stick_input)
        u = FlightControl.INDI()
        if tuningMode:
            debug_var += 1
            print('debug_var: ', debug_var/50, '/', maxTime)
            tunning_graph.append([FlightControl.omega_set[-1], FlightControl.omega[-1]])
            tunning_graph_dot.append(FlightControl.omega_dot)
            if debug_var >= maxTime*50:
                Running = False
            
        update.vjoyUpdate(u[0], u[1], u[2], VirtualStick)
        
print('----Controller Stopped----')
print('Making Graph')

roll_graph = np.array(tunning_graph).T[0]
pitch_graph = np.array(tunning_graph).T[1]
yaw_graph = np.array(tunning_graph).T[2]
t_graph = np.linspace(0, maxTime, num=debug_var)

fig, axes = plt.subplots(3, 1, figsize=(5, 5))

# Subplot 1
axes[0].plot(t_graph, roll_graph[0], color='orange',label='Setpoint')
axes[0].plot(t_graph, roll_graph[1], color='blue', alpha=0.7, label='Actual')
axes[0].set_ylabel('Roll [rad/s]')
axes[0].legend()
axes[0].grid(True)

# Subplot 2
axes[1].plot(t_graph, pitch_graph[0], color='orange', label='Setpoint')
axes[1].plot(t_graph, pitch_graph[1], color='blue', alpha=0.7, label='Actual')
axes[1].set_ylabel('Pitch [rad/s]')
axes[1].legend()
axes[1].grid(True)

# Subplot 3
axes[2].plot(t_graph, yaw_graph[0], color='orange', label='Setpoint')
axes[2].plot(t_graph, yaw_graph[1], color='blue', alpha=0.7, label='Actual')
axes[2].set_xlabel('Time [s]')
axes[2].set_ylabel('Yaw [rad/s]')
axes[2].legend()
axes[2].grid(True)

plt.tight_layout()

roll_dot_graph = np.array(tunning_graph_dot).T[0]
pitch_dot_graph = np.array(tunning_graph_dot).T[1]
yaw_dot_graph = np.array(tunning_graph_dot).T[2]

fig2, axes2 = plt.subplots(3, 1, figsize=(5, 5))

# Subplot 1
axes2[0].plot(t_graph, roll_dot_graph[0], color='orange',label='Setpoint')
axes2[0].plot(t_graph, roll_dot_graph[1], color='blue', alpha=0.7, label='Actual')
axes2[0].set_ylabel('Roll [rad/s2]')
axes2[0].legend()
axes2[0].grid(True)

# Subplot 2
axes2[1].plot(t_graph, pitch_dot_graph[0], color='orange', label='Setpoint')
axes2[1].plot(t_graph, pitch_dot_graph[1], color='blue', alpha=0.7, label='Actual')
axes2[1].set_ylabel('Pitch [rad/s2]')
axes2[1].legend()
axes2[1].grid(True)

# Subplot 3
axes2[2].plot(t_graph, yaw_dot_graph[0], color='orange', label='Setpoint')
axes2[2].plot(t_graph, yaw_dot_graph[1], color='blue', alpha=0.7, label='Actual')
axes2[2].set_xlabel('Time [s]')
axes2[2].set_ylabel('Yaw [rad/s2]')
axes2[2].legend()
axes2[2].grid(True)


plt.tight_layout()
plt.show()
plt.clf()
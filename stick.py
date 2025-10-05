import numpy as np
import pygame

pygame.init()
pygame.joystick.init()

def detectStick():
    physical_stick = None
    if pygame.joystick.get_count():
        print('----Connected Sticks----')
        for i in range(pygame.joystick.get_count()):
            js = pygame.joystick.Joystick(i)
            js.init()
            print(i, js.get_name())
            if 'vjoy' not in js.get_name().lower():
                physical_stick = js
    if physical_stick is None:
        print('Warning, no physical joystick detected')
        print('using last stick detected instead\n')
        for i in range(pygame.joystick.get_count()):
            js = pygame.joystick.Joystick(i)
            js.init()
            physical_stick = js
    return physical_stick

def getStickInput(stick, deadzone = 0.02):
    pygame.event.pump()
    axes = [stick.get_axis(i) for i in range(stick.get_numaxes())]
    # Apply deadzone
    for i in range(len(axes)):
        if abs(axes[i]) <= 0.025:
            axes[i] = 0
    return np.array(axes)
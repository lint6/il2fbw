def to_vjoy(x):
    try:
        int(x)
    except:
        print('nan error when updating control')
        return int(16383.5)
    else:
        return int((x + 1) * 16383.5)

def vjoyUpdate(axisX, axisY, axisZ, VJoyDevice = None):
    '''
    Update Function for VJoy
    AxisX: Roll Channel [-1, 1]
    AxisY: Pitch Channel [-1, 1]
    AxisZ: Yaw Channel [-1, 1]
    VJoyDevice: VJoyDevice
    '''
    if VJoyDevice == None:
        raise RuntimeError('No VJoy Device')
    VJoyDevice.data.wAxisX = to_vjoy(axisX)
    VJoyDevice.data.wAxisY = to_vjoy(axisY)
    VJoyDevice.data.wAxisZ = to_vjoy(axisZ)
    VJoyDevice.update()
    return
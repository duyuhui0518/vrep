# -*- coding: utf-8 -*-
try:
    import vrep
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

import time
import sys
import ctypes

print('Program started')
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')
else:
    print('Failed connecting to remote API server')
    sys.exit('Program Ended')

nominalLinearVelocity = 0.3
wheelRadius = 0.027
interWheelDistance = 0.119

res, objs = vrep.simxGetObjects(clientID, vrep.sim_handle_all, vrep.simx_opmode_blocking)
if res == vrep.simx_return_ok:
    print('Number of objects in the scene: ', len(objs))
else:
    print('Remote API function call returned with error code: ', res)

time.sleep(2)

startTime = time.time()
vrep.simxGetIntegerParameter(clientID, vrep.sim_intparam_mouse_x, vrep.simx_opmode_streaming)

res, display = vrep.simxGetUIHandle(clientID, "sensorDisplay", vrep.simx_opmode_blocking)
res, leftSensor = vrep.simxGetObjectHandle(clientID, "line_sensor0", vrep.simx_opmode_blocking)
res, middleSensor = vrep.simxGetObjectHandle(clientID, "line_sensor2", vrep.simx_opmode_blocking)
res, rightSensor = vrep.simxGetObjectHandle(clientID, "line_sensor5", vrep.simx_opmode_blocking)
res, leftJointDynamic = vrep.simxGetObjectHandle(clientID, "right_joint", vrep.simx_opmode_blocking)
res, rightJointDynamic = vrep.simxGetObjectHandle(clientID, "left_joint", vrep.simx_opmode_blocking)

if res != vrep.simx_return_ok:
    print('Failed to get sensor Handler')
    vrep.simxFinish(clientID)
    sys.exit('Program ended')


def setLeds(elHandle, left, middle, right):
    vrep.simxSetUIButtonProperty(clientID, elHandle, 8,
                                 vrep.sim_buttonproperty_staydown, vrep.simx_opmode_oneshot)
    vrep.simxSetUIButtonProperty(clientID, elHandle, 16,
                                 vrep.sim_buttonproperty_staydown, vrep.simx_opmode_oneshot)
    vrep.simxSetUIButtonProperty(clientID, elHandle, 24,
                                 vrep.sim_buttonproperty_staydown, vrep.simx_opmode_oneshot)
    if left:
        vrep.simxSetUIButtonProperty(clientID, elHandle, 8,
                                     vrep.sim_buttonproperty_staydown + vrep.sim_buttonproperty_isdown,
                                     vrep.simx_opmode_oneshot)
    if middle:
        vrep.simxSetUIButtonProperty(clientID, elHandle, 16,
                                     vrep.sim_buttonproperty_staydown + vrep.sim_buttonproperty_isdown,
                                     vrep.simx_opmode_oneshot)
    if right:
        vrep.simxSetUIButtonProperty(clientID, elHandle, 24,
                                     vrep.sim_buttonproperty_staydown + vrep.sim_buttonproperty_isdown,
                                     vrep.simx_opmode_oneshot)


sensorReading = [False, False, False]
sensorReading[0] = (vrep.simxReadVisionSensor(clientID, leftSensor, vrep.simx_opmode_streaming) == 1)
sensorReading[1] = (vrep.simxReadVisionSensor(clientID, middleSensor, vrep.simx_opmode_streaming) == 1)
sensorReading[2] = (vrep.simxReadVisionSensor(clientID, rightSensor, vrep.simx_opmode_streaming) == 1)

while time.time() - startTime < 50:
    returnCode, data = vrep.simxGetIntegerParameter(clientID, vrep.sim_intparam_mouse_x,
                                                    vrep.simx_opmode_buffer)  # Try to retrieve the streamed data

    # Read the sensors:

    sensorReading[0] = (vrep.simxReadVisionSensor(clientID, leftSensor, vrep.simx_opmode_buffer)[1])
    sensorReading[1] = (vrep.simxReadVisionSensor(clientID, middleSensor, vrep.simx_opmode_buffer)[1])
    sensorReading[2] = (vrep.simxReadVisionSensor(clientID, rightSensor, vrep.simx_opmode_buffer)[1])

    setLeds(display, sensorReading[0], sensorReading[1], sensorReading[2])

    # Decide about left and right velocities:
    s = 1.0
    linearVelocityLeft = nominalLinearVelocity * s
    linearVelocityRight = nominalLinearVelocity * s

    if not sensorReading[0]:
        linearVelocityLeft = linearVelocityLeft * 0.3
    if not sensorReading[2]:
        linearVelocityRight = linearVelocityRight * 0.3

    vrep.simxSetJointTargetVelocity(clientID, leftJointDynamic, linearVelocityLeft / (s * wheelRadius),
                                    vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, rightJointDynamic, linearVelocityRight / (s * wheelRadius),
                                    vrep.simx_opmode_oneshot)

    time.sleep(0.005)
import pybullet as p
import pybullet_data
import numpy as np
import time
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("turtlebot/turtlebot.urdf",cubeStartPos, cubeStartOrientation)
d = 0.115
r = 0.0352
dt = 1./240.

for _ in range(1000):
    (linkWorldPosition,linkWorldOrientation,_,_,_,_,_,worldLinkAngularVelocity) = p.getLinkState(boxId,4, computeLinkVelocity=1, computeForwardKinematics=1)
    action = [-10,10]
    p.setJointMotorControl2(boxId, 0 , p.VELOCITY_CONTROL, targetVelocity = action[0])
    p.setJointMotorControl2(boxId, 1 , p.VELOCITY_CONTROL, targetVelocity = action[1])
    p.stepSimulation()
    time.sleep(1./240.)
    print(p.getEulerFromQuaternion(linkWorldOrientation), linkWorldPosition)
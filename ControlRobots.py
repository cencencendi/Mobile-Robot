import pybullet as p
import pybullet_data
import numpy as np
import time

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("husky/husky.urdf",cubeStartPos, cubeStartOrientation)

def maju(boxId, waktu):
    orientasi = []
    for i in range(int(2400*waktu)):
        p.setJointMotorControl2(boxId, 2 , p.VELOCITY_CONTROL, targetVelocity = 10, )
        p.setJointMotorControl2(boxId, 3 , p.VELOCITY_CONTROL, targetVelocity = 10, )
        p.setJointMotorControl2(boxId, 4 , p.VELOCITY_CONTROL, targetVelocity = 10, )
        p.setJointMotorControl2(boxId, 5 , p.VELOCITY_CONTROL, targetVelocity = 10, )
        time.sleep(1./2400.)
        (linkWorldPosition,
            linkWorldOrientation,
            localInertialFramePosition,
            localInertialFrameOrientation,
            worldLinkFramePosition,
            worldLinkFrameOrientation,
            worldLinkLinearVelocity,
            worldLinkAngularVelocity) = p.getLinkState(boxId,0, computeLinkVelocity=1, computeForwardKinematics=1)
        orientasi.append(list(linkWorldOrientation))
        p.stepSimulation()
    return np.array(orientasi)

def belok_kanan(boxId, waktu):
    orientasi = []
    for _ in range(int(2400*waktu)):
        p.setJointMotorControl2(boxId, 2 , p.VELOCITY_CONTROL, targetVelocity = 10, )
        p.setJointMotorControl2(boxId, 3 , p.VELOCITY_CONTROL, targetVelocity = 0, )
        p.setJointMotorControl2(boxId, 4 , p.VELOCITY_CONTROL, targetVelocity = 10, )
        p.setJointMotorControl2(boxId, 5 , p.VELOCITY_CONTROL, targetVelocity = 0, )
        time.sleep(1./2400.)
        (linkWorldPosition,
            linkWorldOrientation,
            localInertialFramePosition,
            localInertialFrameOrientation,
            worldLinkFramePosition,
            worldLinkFrameOrientation,
            worldLinkLinearVelocity,
            worldLinkAngularVelocity) = p.getLinkState(boxId,0, computeLinkVelocity=1, computeForwardKinematics=1)
        orientasi.append(list(linkWorldOrientation))
        p.stepSimulation()
    return np.array(orientasi)

def belok_kiri(boxId, waktu):
    orientasi = []
    for _ in range(int(2400*waktu)):
        p.setJointMotorControl2(boxId, 2 , p.VELOCITY_CONTROL, targetVelocity = 0, )
        p.setJointMotorControl2(boxId, 3 , p.VELOCITY_CONTROL, targetVelocity = 10, )
        p.setJointMotorControl2(boxId, 4 , p.VELOCITY_CONTROL, targetVelocity = 0, )
        p.setJointMotorControl2(boxId, 5 , p.VELOCITY_CONTROL, targetVelocity = 10, )
        time.sleep(1./2400.)
        (linkWorldPosition,
            linkWorldOrientation,
            localInertialFramePosition,
            localInertialFrameOrientation,
            worldLinkFramePosition,
            worldLinkFrameOrientation,
            worldLinkLinearVelocity,
            worldLinkAngularVelocity) = p.getLinkState(boxId,0, computeLinkVelocity=1, computeForwardKinematics=1)
        orientasi.append(list(linkWorldOrientation))
        p.stepSimulation()
    return np.array(orientasi)

def mundur(boxId, waktu):
    orientasi = []
    for _ in range(int(2400*waktu)):
        p.setJointMotorControl2(boxId, 2 , p.VELOCITY_CONTROL, targetVelocity = -10, )
        p.setJointMotorControl2(boxId, 3 , p.VELOCITY_CONTROL, targetVelocity = -10, )
        p.setJointMotorControl2(boxId, 4 , p.VELOCITY_CONTROL, targetVelocity = -10, )
        p.setJointMotorControl2(boxId, 5 , p.VELOCITY_CONTROL, targetVelocity = -10, )
        time.sleep(1./2400.)
        (linkWorldPosition,
            linkWorldOrientation,
            localInertialFramePosition,
            localInertialFrameOrientation,
            worldLinkFramePosition,
            worldLinkFrameOrientation,
            worldLinkLinearVelocity,
            worldLinkAngularVelocity) = p.getLinkState(boxId,0, computeLinkVelocity=1, computeForwardKinematics=1)
        orientasi.append(list(linkWorldOrientation))
        p.stepSimulation()
    return np.array(orientasi)

def berhenti(boxId, waktu):
    orientasi = []
    for _ in range(int(2400*waktu)):
        p.setJointMotorControl2(boxId, 2 , p.VELOCITY_CONTROL, targetVelocity = 0, )
        p.setJointMotorControl2(boxId, 3 , p.VELOCITY_CONTROL, targetVelocity = 0, )
        p.setJointMotorControl2(boxId, 4 , p.VELOCITY_CONTROL, targetVelocity = 0, )
        p.setJointMotorControl2(boxId, 5 , p.VELOCITY_CONTROL, targetVelocity = 0, )
        time.sleep(1./2400.)
        (linkWorldPosition,
            linkWorldOrientation,
            localInertialFramePosition,
            localInertialFrameOrientation,
            worldLinkFramePosition,
            worldLinkFrameOrientation,
            worldLinkLinearVelocity,
            worldLinkAngularVelocity) = p.getLinkState(boxId,0, computeLinkVelocity=1, computeForwardKinematics=1)
        orientasi.append(list(linkWorldOrientation))
        p.stepSimulation()
    return np.array(orientasi)

if __name__ == "__main__":
    langkah = [maju(boxId, 0.1), belok_kanan(boxId, 0.2), maju(boxId, 0.2), belok_kiri(boxId, 0.2), mundur(boxId, 0.1), berhenti(boxId, 0.1)]
    
    # orientation = np.array([])
    # for i in langkah:
    #     print(i.shape)
        # orientation = np.concatenate((orientation, i), axis=None)

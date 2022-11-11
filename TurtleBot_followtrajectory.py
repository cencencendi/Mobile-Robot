import pybullet as p
import pybullet_data
import numpy as np
import time
import matplotlib.pyplot as plt

class TurtleBot2:
    def __init__(self, sim_active):
            self.sim_active = sim_active
            if self.sim_active:
                physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
            else:
                physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
            p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
            p.setGravity(0,0,-10)
            planeId = p.loadURDF("plane.urdf")
            cubeStartPos = [0,0,0]
            cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
            self.boxId = p.loadURDF("turtlebot/turtlebot.urdf",cubeStartPos, cubeStartOrientation)
            self.d = 0.115
            self.r = 0.0352
            self.dt = 1./240.

    def rot_mat(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1]])

    def rotmat2theta(self, rotmat):
        return np.arctan2(rotmat[1,0], rotmat[0,0])

    def main(self, pose_targets, Kp):
        Kp = Kp
        done = False

        pose_now = np.array([0,0,0], dtype=np.float32)
        orientation_now = np.array([0,0,0], dtype=np.float32)

        action = np.array([0,0], dtype=np.float32) #ul,ur
        action_list, orientation_list, pose_list = [], [], []

        for pose_target in pose_targets:
            pose_target = pose_target.reshape(3,)
            # while not done:
            (linkWorldPosition0,linkWorldOrientation0,_,_,_,_,_,worldLinkAngularVelocity0) = p.getLinkState(self.boxId,0, computeLinkVelocity=1, computeForwardKinematics=1)
            (linkWorldPosition4,linkWorldOrientation4,_,_,_,_,_,worldLinkAngularVelocity4) = p.getLinkState(self.boxId,4, computeLinkVelocity=1, computeForwardKinematics=1)
            pose_now = np.array(linkWorldPosition0)

            pose_error = pose_target - pose_now

            orientation_to_pose_target = np.arctan2(pose_error[1], pose_error[0])
            
            # orientation_now += np.array(worldLinkAngularVelocity, dtype=np.float32)*dt

            orientation_now = np.array(p.getEulerFromQuaternion(linkWorldOrientation4))
            orientation_error = self.rot_mat(orientation_to_pose_target)@self.rot_mat(orientation_now[2]).T
            theta_error = self.rotmat2theta(orientation_error)
            if theta_error>0:
                v_r = (theta_error)*2*self.d/self.r
                v_l = 0
            elif theta_error<0:
                v_r = 0
                v_l = -(theta_error)*2*self.d/self.r
            else:
                v_r = 0
                v_l = 0

            v_l, v_r = np.clip(v_l, 0, 1), np.clip(v_r, 0, 1)
            targetVelocity = np.linalg.norm(pose_error)
            targetVelocity = np.clip(targetVelocity, 0, 5)
            action[0], action[1] = Kp*(targetVelocity+v_l), Kp*(targetVelocity+v_r)

            p.setJointMotorControl2(self.boxId, 0 , p.VELOCITY_CONTROL, targetVelocity = action[0])
            p.setJointMotorControl2(self.boxId, 1 , p.VELOCITY_CONTROL, targetVelocity = action[1])
            time.sleep(self.dt)
            p.stepSimulation()

            action_list.append([Kp*(targetVelocity+v_l), Kp*(targetVelocity+v_r)])
            orientation_list.append(orientation_now)
            pose_list.append(pose_now)
        p.disconnect()
        print(f"Position now: {pose_now}, Orientation: {orientation_now}, with orientation error = {theta_error}, and pose_error: {pose_error}")
        return np.array(action_list), np.array(orientation_list), np.array(pose_list)

            
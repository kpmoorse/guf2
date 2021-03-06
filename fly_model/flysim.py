import pybullet as p
import time
import pybullet_data
import pandas as pd
import numpy as np
import csv
from qfunc import *
import matplotlib.pyplot as plt
import modes
# import qarray

class Fly(object):
    
    def __init__(self, startPos, startOrn, gui=True, apply_forces=True, cmd=(0,0,0,0,0,0)):

        self.i = 0
        self.startPos = startPos
        self.startOrn = startOrn
        self.cmd = cmd

        self.apply_forces = apply_forces

        self.global_state = np.empty((0,6)) # x, y ,z, roll, pitch, yaw
        self.hinge_state = np.empty((0,6)) # posL, devL, rotL, posR, devR, rotR

        self.lifts = np.empty((0,2)) # left, right
        self.drags = np.empty((0,2)) # left, right

        self.forces = np.empty((0,3)) # Fx, Fy, Fz
        self.torques = np.empty((0,3)) # Tx, Ty, Tz

        self.forces_separated = np.empty((0,6))

        self.aoa = [0,0] # Angle of attack [left, right]
        self.aoa_dot = [0,0] # Rate of change of aoa

        # Initialize physics
        if gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # remove overlay
        # self.dt = 1./1200. # seconds
        self.dt = 1./1440. # seconds
        p.setTimeStep(self.dt)

        # Initialize scene
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.resetDebugVisualizerCamera(4.5,30,-20,[0,0,3])
        # p.resetDebugVisualizerCamera(4.5,90,-80,[0,0,3])
        if self.apply_forces:
            p.setGravity(0,0,-12)
        else:
            p.setGravity(0,0,0)
        self.planeId = p.loadURDF("plane.urdf")

        # Load fly model
        # self.flyId = p.loadSDF("fly.sdf")[0]
        self.flyId = p.loadURDF("fly.urdf", startPos, startOrn)

        # Load wing blade element parameters
        wing_params = npread('wing_params_5.csv')
        self.N_wbe = wing_params.shape[0]
        self.wbe_x = wing_params[:,0]
        self.wbe_tops = wing_params[:,1]
        self.wbe_bottoms = wing_params[:,2]

        # Initialize markers
        self.mkL = []
        self.mkR = []
        # for i in range(self.N_wbe):
        #     # self.mkL.append(p.loadURDF("marker_ball.urdf", [1,0,3], p.getQuaternionFromEuler([0,0,0])))
        #     # self.mkR.append(p.loadURDF("marker_ball.urdf", [1,0,3], p.getQuaternionFromEuler([0,0,0])))
        #     self.mkL.append(p.loadURDF("arrow_red.urdf", [1,0,3], p.getQuaternionFromEuler([0,0,0])))
        #     self.mkR.append(p.loadURDF("arrow_red.urdf", [1,0,3], p.getQuaternionFromEuler([0,0,0])))

        # Generate link and joint index dictionaries
        num_joints = p.getNumJoints(self.flyId)
        self.link_dict = {}
        self.joint_dict = {}

        for i in range(num_joints):
            joint_info = p.getJointInfo(self.flyId, i)
            self.link_dict[joint_info[12].decode('ascii')] = joint_info[0]
            self.joint_dict[joint_info[1].decode('ascii')] = joint_info[0]

        self.a, self.X, self.b = modes.read_modes()

        self.wk_len = 100 #from legendre kinematics

        # Generate joint index list for arrayed motor control
        self.motor_list = [
            self.joint_dict["hingeL-pos"],
            self.joint_dict["hingeL-dev"],
            self.joint_dict["hingeL-rot"],
            self.joint_dict["hingeR-pos"],
            self.joint_dict["hingeR-dev"],
            self.joint_dict["hingeR-rot"]
        ]

        for i, joint in enumerate(self.motor_list):
            p.resetJointState(self.flyId, joint, self.calc_legendre(i%self.wk_len)[0][i])

        self.cog = calc_cog(0.5,np.array([1,0,0]))

        self.sim_time = 0

    def init_modes(self):

        pass

    def step_simulation(self):

        # mag=0

        p.removeAllUserDebugItems()

        lifts = np.zeros(2)
        drags = np.zeros(2)

        net_force = np.zeros(3)
        net_torque = np.zeros(3)

        target, target_last = self.calc_legendre(self.i%self.wk_len) #from legendre model

        # Chirp kinematics if forces are being applied
        if self.apply_forces:
            target = self.linramp(self.sim_time, target, 100*self.dt)

        # flip = np.sign(target[0] - target_last[0])

        # Loop over wing kinematics
        p.setJointMotorControlArray(
            self.flyId,
            self.motor_list,
            controlMode=p.POSITION_CONTROL,
            targetPositions=target,
            forces=[1e10]*6
            )

        # # Apply stabilizing external torque
        # if self.apply_forces:
        #     p.applyExternalTorque(
        #         self.flyId,
        #         -1,
        #         # (0,8.5,0),
        #         (0,5,0),
        #         p.LINK_FRAME
        #         )

        # Calculate rotated center of gravity, relative to body origin
        rot_mat = p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(self.flyId)[1])
        rot_mat = np.reshape(np.array(rot_mat),[3,3])
        rot_cog = np.dot(rot_mat, self.cog)
        # print(self.cog)
        # print(rot_cog)

        ##### Apply quasisteady model to left wing #####

        # Draw wing velocity and direction vectors
        wlVel_base = p.getLinkState(self.flyId, self.link_dict["wingL"], computeLinkVelocity=1)[6]
        # wing_pos, vel_orn = worldPlusVector(self.flyId, self.link_dict["wingL"], [0,0,1])

        # Calculate angle of attack and its derivative
        aoa_last = self.aoa[0]
        self.aoa[0] = qAngle(
            worldPlusVector(self.flyId,self.link_dict["wingL"],[0,0,0],[0,0,1])[1],
            vec2q(wlVel_base)
        )
        self.aoa_dot[0] = (self.aoa[0] - aoa_last)/self.dt

        # Calculate center of pressure coefficient
        xcp = 0.82*abs(self.aoa[0])/np.pi + 0.05

        # Loop over wingblade elements
        for i in range(self.N_wbe):
        # for i in range(0):

            wbe_scale = self.wbe_x[i]/0.1
            wlVel = [a*wbe_scale for a in wlVel_base]

            # Calculate center of pressure relative to wing base
            cpRel = [0, self.wbe_x[i], (1-xcp)*self.wbe_tops[i] + (xcp)*self.wbe_bottoms[i]]

            # Update reference markers
            cpAbs, vel_orn = worldPlusVector(self.flyId, self.link_dict["wingL"], cpRel, [0,0,0])
            # p.resetBasePositionAndOrientation(self.mkL[i], cpAbs, vec2q(wlVel))
            # p.resetBasePositionAndOrientation(self.mkL[i], cpAbs, normal)
            # p.addUserDebugLine(
            #     cpAbs,
            #     np.array(cpAbs)+np.array(wlVel),
            #     lineColorRGB=[1,0,0],
            #     lineWidth=3,
            #     lifeTime=0.1
            #     )

            # Apply lift & drag forces
            span = q2vec(worldPlusVector(self.flyId, self.link_dict["wingL"],[0,0,0],[0,1,0])[1])
            normal = q2vec(p.getLinkState(self.flyId, self.link_dict["wingL"])[1])
            flip = -np.sign(np.dot(wlVel, normal))

            drag = self.calc_drag(self.aoa[0], wlVel)
            lift = self.calc_lift(self.aoa[0], wlVel, span, flip)
            frot = self.calc_frot(self.aoa_dot[0], wlVel, normal, flip)
            # print((np.linalg.norm(drag),np.linalg.norm(lift),np.linalg.norm(frot)))

            if self.apply_forces and self.i>10:
                force = lift+drag+frot
                # if i == self.N_wbe-1:
                #     net_force *= 0.0
                p.applyExternalForce(
                    self.flyId,
                    self.link_dict["wingL"],
                    force,
                    cpAbs,
                    p.WORLD_FRAME
                    )

            lifts[1] = np.linalg.norm(lift)
            drags[1] = np.linalg.norm(drag)

            # Accumulate forces and torques
            net_force += lift+drag+frot
            lever = np.array(cpAbs - np.array(p.getBasePositionAndOrientation(self.flyId)[0]+rot_cog))
            net_torque += np.cross(lever, lift+drag+frot)
        
        # p.resetBasePositionAndOrientation(self.mkGrnId1, wing_pos, vec2q(lift+drag))

        ##### Apply quasisteady model to right wing #####

        # Draw wing velocity and direction vectors
        wlVel_base = p.getLinkState(self.flyId, self.link_dict["wingR"], computeLinkVelocity=1)[6]
        # wing_pos, vel_orn = worldPlusVector(self.flyId, self.link_dict["wingR"], [0,0,0], [0,0,1])

        # Calculate angle of attack and its derivative
        aoa_last = self.aoa[1]
        self.aoa[1] = qAngle(
            worldPlusVector(self.flyId,self.link_dict["wingR"],[0,0,0],[0,0,1])[1],
            vec2q(wlVel_base)
        )
        self.aoa_dot[1] = (self.aoa[1] - aoa_last)/self.dt

        # Calculate center of pressure coefficient
        xcp = 0.82*abs(self.aoa[1])/np.pi + 0.05

        # Loop over wingblade elements
        for i in range(self.N_wbe):

            wbe_scale = self.wbe_x[i]/0.1
            wlVel = [a*wbe_scale for a in wlVel_base]

            # Calculate center of pressure relative to wing base
            cpRel = [0, -self.wbe_x[i], (1-xcp)*self.wbe_tops[i] + (xcp)*self.wbe_bottoms[i]]

             # Update reference markers
            cpAbs, vel_orn = worldPlusVector(self.flyId, self.link_dict["wingR"], cpRel, [0,0,0])
            # p.resetBasePositionAndOrientation(self.mkR[i], cpAbs, vec2q(wlVel))
            # p.addUserDebugLine(
            #     cpAbs,
            #     np.array(cpAbs)-np.array(wlVel),
            #     lineColorRGB=[1,0,0],
            #     lineWidth=3
            #     )

            # Apply lift & drag forces
            span = q2vec(worldPlusVector(self.flyId, self.link_dict["wingR"], [0,0,0], [0,1,0])[1])
            normal = q2vec(p.getLinkState(self.flyId, self.link_dict["wingR"])[1])
            flip = -np.sign(np.dot(wlVel, normal))
            
            drag = self.calc_drag(self.aoa[1], wlVel)
            lift = self.calc_lift(self.aoa[1], wlVel, span, flip)
            frot = self.calc_frot(self.aoa_dot[1], wlVel, normal, flip)
            # if i%10==0:
            #     print(lift)
            # if not init:
            if self.apply_forces and self.i>10:
                force = lift+drag+frot
                p.applyExternalForce(
                    self.flyId,
                    self.link_dict["wingR"],
                    force,
                    cpAbs,
                    p.WORLD_FRAME
                    )

            lifts[0] = np.linalg.norm(lift)
            drags[0] = np.linalg.norm(drag)

            net_force += lift+drag+frot
            lever = np.array(cpAbs - np.array(p.getBasePositionAndOrientation(self.flyId)[0]+rot_cog))
            net_torque += np.cross(lever, lift+drag+frot)

        # p.resetBasePositionAndOrientation(self.mkGrnId2, wing_pos, vec2q(lift+drag))

        p.stepSimulation()
        self.sim_time += self.dt
        time.sleep(self.dt)

        self.lifts = np.append(self.lifts, lifts[None,:], 0)
        self.drags = np.append(self.drags, drags[None,:], 0)

        self.forces = np.append(self.forces, net_force[None,:], 0)
        self.torques = np.append(self.torques, net_torque[None,:], 0)
        self.hinge_state = np.append(self.hinge_state, target.T[None,:], 0)

        self.i += 1
    
    def calc_com(self):
        
        for link in self.link_dict:
            # print(link)
            pass

    # Calculate drag force from quasisteady model
    @staticmethod
    def calc_drag(aoa, vel):
        vel = np.array(vel)

        cD = 1.464 * np.sin(0.0342*aoa - 1.667) + 2.008
        drag = -cD * vel * np.linalg.norm(vel) * 0.01
        return drag

    # Calculate lift force from quasisteady model
    @staticmethod
    def calc_lift(aoa, vel, span, flip=1):
        vel = np.array(vel)

        cL = 1.597 * np.sin(0.0407*aoa - 0.369) + 0.317
        lift = cL * np.cross(vel, flip*span) * np.linalg.norm(vel) * 0.01
        return(lift)

    # Calculate rotational force from quasisteady model
    @staticmethod
    def calc_frot(aoa_dot, vel, normal, flip):
        vel = np.array(vel)

        cR = 1.55
        frot = cR * aoa_dot * np.linalg.norm(vel) * normal * 0.004

        return frot

    @staticmethod
    def linramp(t,x,tc):
        if t < tc:
            x_ramp = x*t/tc
        else:
            x_ramp = x
        return x_ramp

    def calc_legendre(self, i):

        kin = modes.calc_kinematics(self.a, self.X, self.b, self.cmd)

        # Adjust reference frames
        kin[:,2] -= np.pi/2
        kin[:,5] -= np.pi/2
        kin[:,3:5] *= -1

        return kin[i,:], kin[i-1,:]

    def reset(self):

        p.resetBasePositionAndOrientation(self.flyId, self.startPos, self.startOrn)
        self.forces = np.empty((0,3)) # Fx, Fy, Fz
        self.torques = np.empty((0,3)) # Tx, Ty, Tz
        self.i = 0

    def __del__(self):

        p.disconnect()

# Import CSV as numpy array via pandas
def npread(file, sep=',', header=None):
    df = pd.read_csv(file, sep=sep, header=header)
    arr = np.asarray(df)
    return arr

def npwrite(arr, file):
    df = pd.DataFrame(arr)
    df.to_csv(file, header=False, index=False)

def calc_cog(m, ctr):
    cog = np.array([0.,0.,0.])
    cog += (np.array([0.21891405,  0.00028719, -0.06895534])) * 1.0 # Thorax
    cog += (np.array([2.68651766e-01, -5.60478757e-05,  1.11634186e-03]) + np.array([0.7, 0, 0.4])) * 1.0 # Head
    cog += (np.array([-7.34098503e-01, -4.75165506e-09,  3.17227650e-02]) + np.array([-0.25, 0, -0.2])) * 1.0 # Abdomen
    cog += ctr * m
    return cog


if __name__ == "__main__":

    mag = -2

    flyStartPos = [0,0,4]
    flyStartOrn = p.getQuaternionFromEuler([0,0,0])
    fly = Fly(flyStartPos, flyStartOrn, gui=0, apply_forces=0, cmd=(0,0,0,0,0,0))
    aoa = []
    for i in range(100):
        fly.step_simulation()
        # aoa.append(fly.aoa)

    # plt.plot(aoa)
    # plt.show()
    
    # f = fly.torques[:,1]

    if mag != 0:
        tag = ("n","p")[int(mag>0)] + str(abs(mag))
    else:
        tag = "00"

    # npwrite(f,'res_pos/my_{}.csv'.format(tag))

    # # plt.plot(fly.wingstate[:100,:3])
    # plt.plot(fly.wingstate[:,0],fly.wingstate[:,1])
    # for i in range(100):
    #     plt.arrow(
    #         fly.wingstate[i,0],
    #         fly.wingstate[i,1],
    #         fly.drags[i,0]*0.01,
    #         fly.lifts[i,0]*0.01
    #     )
    # plt.gca().set_aspect('equal')
    # plt.show()

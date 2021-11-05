#!/usr/bin/env python3

import numpy as np
import math
from mujoco_py import load_model_from_path, MjSim, MjViewer

# Loads model from path
model = load_model_from_path("model/quadrotor_v01.xml")

# represents a running simulation including its state
sim = MjSim(model)

# Display GUI showing the scene of an MjSim with a mouse-movable camera
viewer = MjViewer(sim)

# Returns a copy of the simulator state
sim_state = sim.get_state()

class Quadrotor:
    def __init__(self, control_param=None):
        # PID gains
        self.PID = control_param

        # Drag constants
        km = 1.0
        kf = 2.0
        
        # A matrix
        z   = np.array([0.0, 0.0, 1.0])
        # Cross the motor position with the direction of the force (z for normal quadrotor)
        # last term is air drag 
        m0  = np.cross(np.array([0.1, 0.1, 0.01]), z)   + ((km/kf) * ((-1)**(1+1)) * z)
        m1  = np.cross(np.array([0.1, -0.1, 0.01]), z)  + ((km/kf) * ((-1)**(1+2)) * z)
        m2  = np.cross(np.array([-0.1, -0.1, 0.01]), z) + ((km/kf) * ((-1)**(1+3)) * z)
        m3  = np.cross(np.array([-0.1, 0.1, 0.01]), z)  + ((km/kf) * ((-1)**(1+4)) * z)

        self.A = np.array([ np.insert(m0, 0, 1),
                            np.insert(m1, 0, 1),
                            np.insert(m2, 0, 1),
                            np.insert(m3, 0, 1)]).transpose()

        self.inv_A = np.linalg.inv(self.A)
        self.controllability = np.linalg.matrix_rank(self.A)
        

        # logs
        self.log_p = []
        self.log_R = []
        self.log_time = []
        self.log_u = []
        self.log_rpy = []
        self.log_th = []
        self.log_tor = []

    def control(self, des_pos, des_quat, rpy_d, des_vel=None, des_acc=None):
        # Get the current and desired state
        p_d = des_pos
        q_d = des_quat
        R_d = quat2rot(q_d)
        v_d, omega_d = des_vel
        a_des, alpha_des = np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])

        if des_acc:
            a_des, alpha_des = des_acc

        p = sim.data.get_body_xpos("quadrotor")
        v = sim.data.get_body_xvelp("quadrotor")
        omega = sim.data.get_body_xvelr("quadrotor")
        rpy = quat_to_rpy(sim.data.get_body_xquat("quadrotor"))
        R = quat2rot(sim.data.get_body_xquat("quadrotor"))

        # errors in position
        ep = p_d - p
        ev = v_d - v
        self.log_p.append(ep)
        self.PID.e_p_i += ep
        for i in range(len(self.PID.e_p_i)):
            if self.PID.e_p_i[i] > self.PID.cap_p_i:
                self.PID.e_p_i[i] = self.PID.cap_p_i
            elif self.PID.e_p_i[i] < -self.PID.cap_p_i:
                self.PID.e_p_i[i] = -self.PID.cap_p_i

        # errors in rotations for logs
        erpy = rpy_d - rpy
        for i in range(len(erpy)):
            if erpy[i] <= -np.pi:
                erpy[i] += 2 * np.pi
            elif erpy[i] > np.pi:
                erpy[i] -= 2 * np.pi
        self.log_rpy.append(erpy)

        # PID control for position in {W}
        kp_z, kd_z, ki_z = self.PID.kpz, self.PID.kdz, self.PID.kiz
        kp_xy, kd_xy, ki_xy = self.PID.kpxy, self.PID.kdxy, self.PID.kixy
        ar = np.concatenate([kp_xy * ep[:2] + kd_xy * ev[:2] + ki_xy * self.PID.e_p_i[:2],
                             np.array([kp_z * ep[2] + kd_z * ev[2] + ki_z * self.PID.e_p_i[2]])]) + a_des
        ar[2] += g
        f = self.PID.mass * ar

        if self.controllability != 6:
            # special cases of under-actuation
            z_d = ar / np.linalg.norm(ar)
            if self.controllability == 4:
                x_c = np.array([np.cos(rpy_d[2]), np.sin(rpy_d[2]), 0])
            else:
                _, pitch_d, yaw_d = rpy_d
                x_c = np.array([np.cos(pitch_d)*np.cos(yaw_d), np.cos(pitch_d)*np.sin(yaw_d), np.sin(pitch_d)])
            y_d = np.cross(z_d, x_c)
            y_d = y_d / np.linalg.norm(y_d)
            x_d = np.cross(y_d, z_d)
            R_d = np.vstack([x_d, y_d, z_d]).T

        eR = 0.5 * vee_map(R_d.T.dot(R) - R.T.dot(R_d))
        self.log_R.append(eR)
        self.PID.e_R_i += eR
        for i in range(len(self.PID.e_R_i)):
            if self.PID.e_R_i[i] > self.PID.cap_R_i:
                self.PID.e_R_i[i] = self.PID.cap_R_i
            elif self.PID.e_R_i[i] < -self.PID.cap_R_i:
                self.PID.e_R_i[i] = -self.PID.cap_R_i

        eomega = omega_d - omega

        f = R.T.dot(f)

        kp_rp, kd_rp, ki_rp = self.PID.kprp, self.PID.kdrp, self.PID.kirp
        kp_y, kd_y, ki_y = self.PID.kpy, self.PID.kdy, self.PID.kiy

        aR = np.concatenate([-kp_rp * eR[:2] + kd_rp * eomega[:2] - ki_rp * self.PID.e_R_i[:2],
                             np.array([-kp_y * eR[2] + kd_y * eomega[2] - ki_y * self.PID.e_R_i[2]])]) + alpha_des

        tau = self.PID.inertia * aR

        w = np.concatenate([f, tau])[2:]
        # print("wrench: {}".format(w))
        u_crude = self.inv_A.dot(w)

        u = u_crude
        for i in range(len(u_crude)):
            if u_crude[i] < 0:
                u[i] = 0

        sim.data.ctrl[0] = u[0]
        sim.data.ctrl[1] = u[1]
        sim.data.ctrl[2] = u[2]
        sim.data.ctrl[3] = u[3]
        #print("u: {}".format(u))
        self.log_u.append(u)

        self.log_th.append(R.dot(w[:3]))
        self.log_tor.append(w[3:])


class PID_param:
    def __init__(self, mass, inertia, KZ, KXY, KRP, KY):
        # integral stuff
        self.cap_R_i = 5.0
        self.e_R_i = np.array([0.0, 0.0, 0.0])

        self.cap_p_i = 0.5
        self.e_p_i = np.array([0.0, 0.0, 0.0])

        self.mass = mass
        self.inertia = inertia
        self.kpz, self.kdz, self.kiz = KZ
        self.kpxy, self.kdxy, self.kixy = KXY
        self.kprp, self.kdrp, self.kirp = KRP
        self.kpy, self.kdy, self.kiy = KY

def quat_to_rpy(quat):
    assert np.shape(quat) == (4,)
    roll = np.arctan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]), 1 - 2 * (quat[1] ** 2 + quat[2] ** 2))
    pitch = np.arcsin(2 * (quat[0] * quat[2] - quat[3] * quat[1]))
    yaw = np.arctan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
    return np.array([roll, pitch, yaw])

def quat2rot(quat):
    # Covert a quaternion into a full three-dimensional rotation matrix.
    # Extract the values from quat
    q0 = quat[0]
    q1 = quat[1]
    q2 = quat[2]
    q3 = quat[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


# convert euler angles (roll, pitch, yaw) into quaternion (qx, qy, qz, qw)
def euler2quat(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    return np.array([sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy,
                     cr * cp * cy + sr * sp * sy])


def vee_map(skew_s):
    # convert a skew-symmetric matrix to the corresponding array
    return np.array([skew_s[2, 1], skew_s[0, 2], skew_s[1, 0]])


# Wow, numpy does not have null space :(
def null_space(A, rcond=None):
    u, s, vh = np.linalg.svd(A, full_matrices=True)
    M, N = u.shape[0], vh.shape[1]
    if rcond is None:
        rcond = np.finfo(s.dtype).eps * max(M, N)
    tol = np.amax(s) * rcond
    num = np.sum(s > tol, dtype=int)
    Q = vh[num:, :].T.conj()
    return Q


if __name__ == "__main__":
    r1 = Quadrotor( PID_param(0.4, 0.05,
                         (8.0, 4, 0.5),
                         (4.0, 10.0, 0.0),
                         (4.0, 5.0, 0.0),
                         (10.0, 5.0, 0.0)))

    g = 9.81
    while (True):
        t = sim.data.time
        r1.control(np.array([math.cos(t), math.sin(t), 1]),     # xyz pose Desired
                    np.array([0, 0, 0, 0]), # Rotation Quaternion Desired
                    np.array([0, 0 , 0]),   # Roll Pitch Yaw Desired
                    (np.array([0,0,0]), np.array([0,0,0])),
                    (np.array([0,0,0]), np.array([0,0,0]))
        )
                    # These would be desired velocity and acceleration of pose and roll, pitch, yaw
                    #(np.array([dx[i], dy[i], dz[i]]), np.array([droll[i], dpitch[i], dyaw[i]])),
                    #(np.array([ddx[i], ddy[i], ddz[i]]), np.array([ddroll[i], ddpitch[i], ddyaw[i]])))
        # Advances the simulation by calling mj_step
        sim.step()
        viewer.render()
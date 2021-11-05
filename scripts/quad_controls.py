#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mujoco_py import load_model_from_path, MjSim, MjViewer
import time

# Loads model from path
model = load_model_from_path("model/boxrotor.xml")

# represents a running simulation including its state
sim = MjSim(model)

# Display GUI showing the scene of an MjSim with a mouse-movable camera
viewer = MjViewer(sim)

# Returns a copy of the simulator state
sim_state = sim.get_state()

'''
    Moves robot in a circle: 
    Using velocity 
    parameters: radius 
'''


def raise_rope_move():
    mass = .4  # new mass
    z_p = 0
    kp1 = 2
    kp2 = 0

    kd1 = 1
    kd2 = 0

    ki1 = 0
    ki2 = 0.000

    kpp2 = np.array([5, 5, 1]) # What are these values ?
    kdd2 = np.array([2, 2, 1]) # 

    # For Orientation
    kp3_z = 5
    kd3_z = 5

    kp3_x = 2
    kd3_x = 2

    kp3_y = 2
    kd3_y = 2

    z_acel = 0

    z_t = 1
    x_t1 = 5/2
    y_t1 = -0.35

    x_r1 = np.array([x_t1, y_t1, z_t]) # What are these values ?

    x_t2 = 5/2
    y_t2 = 0.35

    x_r2 = np.array([x_t2, y_t2, z_t]) # What are these values ?

    tolerance = 0.05

    dt = 0.01
    R = 3  # tradjectory radius
    w = 2.0  # trajectory angular speed (rad/s)

    f_z = 0
    w_v = 5
    R_v = 20

    # orientation
    curr_rpos1 = sim.data.get_body_xquat("quadrotor1") 
    curr_rvel1 = sim.data.get_body_xvelr("quadrotor1")

    # orientation
    curr_rpos2 = sim.data.get_body_xquat("quadrotor2")
    curr_rvel2 = sim.data.get_body_xvelr("quadrotor2")

    print("xquat", curr_rpos1)
    print("xvelr", curr_rvel1)

    curr_pos1 = sim.data.get_body_xpos("quadrotor1")
    curr_pos2 = sim.data.get_body_xpos("quadrotor2")

    curr_vel1 = sim.data.get_body_xvelp("quadrotor1")
    curr_vel2 = sim.data.get_body_xvelp("quadrotor2")

    ae_x1 = ae_y1 = ae_z1 = 0 # What are these values ?
    ae_x2 = ae_y2 = ae_z2 = 0 # What are these values ?

    e_vx1_k1 = e_vx2_k1 = 0.0 # What are these values ?
    ae_z1_k1 = 0 # What are these values ?

    time = 10.0

    plt.axis([0, time, 0, 5]) # What are these values ?

    for t in range(int(time/dt)):

        x_p1 = curr_pos1

        curr_rposadj1 = quat_to_rpy(curr_rpos1)
        curr_rposadj2 = quat_to_rpy(curr_rpos2)

        qcx1 = curr_rposadj1[0]
        qcy1 = curr_rposadj1[1]
        qcz1 = curr_rposadj1[2]


        # I had been accidently using linear velocity instead of angular velocity 
        # for orientation p2d-control

        qx_v1 = curr_rvel1[0]
        qy_v1 = curr_rvel1[1]
        qz_v1 = curr_rvel1[2]

        # Drone 2
        x_p2 = curr_pos2

        qcx2 = curr_rposadj2[0]
        qcy2 = curr_rposadj2[1]
        qcz2 = curr_rposadj2[2]

        qx_v2 = curr_rvel2[0]
        qy_v2 = curr_rvel2[1]
        qz_v2 = curr_rvel2[2]

        # PD Control - Position
        e_x1 = (x_r1 - np.array([t * dt/2, 0, 0])) - x_p1

        x_acel1 = kpp2 * e_x1 + (kdd2 / dt) * (e_x1 - e_vx1_k1)

        e_vx1_k1 = e_x1

        e_x2 = (x_r2 - np.array([t * dt/2, 0, 0])) - x_p2
        x_acel2 = kpp2 * e_x2 + (kdd2 / dt) * (e_x2 - e_vx2_k1)
        e_vx2_k1 = e_x2


        # PD Control - Orientation
        qcx1_acel = kp3_x * (0 - qcx1) + kd3_x * (-qx_v1)
        qcy1_acel = kp3_y * (0 - qcy1) + kd3_y * (-qy_v1)
        qcz1_acel = kp3_z * (0 - qcz1) + kd3_z * (-qz_v1)

        qcx2_acel = kp3_x * (0 - qcx2) + kd3_x * (-qx_v2)
        qcy2_acel = kp3_y * (0 - qcy2) + kd3_y * (-qy_v2)
        qcz2_acel = kp3_z * (0 - qcz2) + kd3_z * (-qz_v2)

        f_z1 = (mass * x_acel1[2]) + (mass * 9.81)
        f_z2 = (mass * x_acel2[2]) + (mass * 9.81)


        sim.data.ctrl[0] = x_acel1[0]
        sim.data.ctrl[1] = x_acel1[1]
        sim.data.ctrl[2] = f_z1
        sim.data.ctrl[3] = qcx1_acel
        sim.data.ctrl[4] = qcy1_acel
        sim.data.ctrl[5] = qcz1_acel

        sim.data.ctrl[6] = x_acel2[0]
        sim.data.ctrl[7] = x_acel2[1]
        sim.data.ctrl[8] = f_z2
        sim.data.ctrl[9] = qcx2_acel
        sim.data.ctrl[10] = qcy2_acel
        sim.data.ctrl[11] = qcz2_acel

        print("curr_pos2", x_p1)
        plt.subplot(321)
        plt.scatter(t * dt, x_p1[0], c = 'r', marker = '.')
        plt.subplot(322)
        plt.scatter(t * dt, x_p1[1], c='r', marker='.')
        plt.subplot(323)
        plt.scatter(t * dt, x_p1[2], c='r', marker='.')
        plt.subplot(324)
        plt.scatter(t * dt, x_p2[0], c = 'r', marker = '.')
        plt.subplot(325)
        plt.scatter(t * dt, x_p2[1], c='r', marker='.')
        plt.subplot(326)
        plt.scatter(t * dt, x_p2[2], c='r', marker='.')

        # Advances the simulation by calling mj_step
        sim.step()
        viewer.render()


def quat_to_rpy(quat):
    assert np.shape(quat) == (4,)
    roll = np.arctan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]), 1 - 2 * (quat[1] ** 2 + quat[2] ** 2))
    pitch = np.arcsin(2 * (quat[0] * quat[2] - quat[3] * quat[1]))
    yaw = np.arctan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
    return np.array([roll, pitch, yaw])


def main(args):
    if args == 8:
        raise_rope_move()
        time.sleep(10)



if __name__ == "__main__":
    main(8)  # raise and move

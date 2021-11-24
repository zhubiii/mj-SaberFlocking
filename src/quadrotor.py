import numpy as np

"""
Quadrotor class for utilizng geometric controller
Built for use in MuJoCo

Parameters
----------
self:                   This object
control_param:          The PID param class specifying gains
sim:                    The MuJoCo simulation reference
id:                     The unique ID of the instantiated quadrotor

Attributes
----------
self.sim:               Represents a running simulation including its state
self.id:                The instantiated quadrotors unique ID
self.name:              The name of the quadrotor in the simulation, specified by generated XML file
self.PID:               Stores PID gains for Z, XY, R(oll)P(itch), Y(aw)
self.curr_pose2D:       The 2-vector of the position
self.curr_vel2D:        The 2-vector of the velocity
self.curr_pose:         The 3-vector of the position
self.curr_vel:          The 3-vector of the velocity
self.A:                 Static 4x4 A matrix for simple quadrotor
                            - Composed of the forces (row 0) and the torques (rows 1-3)
                            - Cross of the motor position vector with the propeller force (unit z vector)
                            - Added to the cross product is a drag constant
                                - Sign is determined by CW or CCW motor rotation
self.inv_A:             Inverse of A matrix
self.controllability:   Rank of A matrix (4 for simple quadrotor)
self.log_p:             Logs the error in position
self.log_R:             Logs the error in orientation
self.log_u:             Logs the input force vector to the motors
self.log_rpy:           Logs errors in rotations roll, pitch, yaw
self.log_th:                
self.log_tor:               

Methods
-------
self.control(self, des_pos, des_quat, rpy_d, des_vec, des_acc)
    Geometric controller that calculates and sends the force inputs
    for each motor


"""
class Quadrotor:
    def __init__(self, control_param=None, sim=None, id=None):
        # MuJoCo simulation state
        self.sim = sim

        # ID for the instantiated drone
        self.id = id

        # Quadrotor name
        self.name = "quadrotor"+str(id)

        # PID gains
        self.PID = control_param

        # Current Pose and Velocity for algorithms
        self.curr_pose2D    = None
        self.curr_vel2D     = None
        self.curr_pose      = None
        self.curr_vel       = None

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
        self.log_u = []
        self.log_rpy = []
        self.log_th = []
        self.log_tor = []

    """
    Geometric Controller based on Taeyoung Lee
    implemented by Daniel Mellinger. Calculates 
    'u' the desired force vector for each motor

    Parameters
    ----------
    des_pos:    Desired XYZ position
    des_quat:   Desired quaternion
    rpy_d:      Desired roll, pitch, yaw
    des_vel:    Desired velocity of pose and roll, pitch, yaw
    des_acc:    Desired acceleration of pose and roll, pitch, yaw

    Return:
    -------
    None
    """
    def control(self, des_pos, des_quat, rpy_d, des_vel=None, des_acc=None):
        # Get desired state
        p_d = des_pos
        q_d = des_quat
        R_d = quat2rot(q_d)
        v_d, omega_d = des_vel
        a_des, alpha_des = np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])

        if des_acc:
            a_des, alpha_des = des_acc

        # Get current state from simulation
        p = self.sim.data.get_body_xpos(self.name)
        v = self.sim.data.get_body_xvelp(self.name)
        self.curr_pose2D    = p[0:2]
        self.curr_vel2D     = v[0:2]
        self.curr_pose      = p
        self.curr_vel       = v
        omega = self.sim.data.get_body_xvelr(self.name)
        rpy = quat_to_rpy(self.sim.data.get_body_xquat(self.name))
        R = quat2rot(self.sim.data.get_body_xquat(self.name))

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
        ar[2] += 9.81
        f = self.PID.mass * ar

        z_d = ar / np.linalg.norm(ar)
        x_c = np.array([np.cos(rpy_d[2]), np.sin(rpy_d[2]), 0])
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

        # Send forces to each motor in mujoco
        # MuJoCo stores the ctrl as a continuous array so
        # we need to index to the correct quadrotor based on its ID
        self.sim.data.ctrl[(self.id*4)]     = u[0]
        self.sim.data.ctrl[(self.id*4)+1]   = u[1]
        self.sim.data.ctrl[(self.id*4)+2]   = u[2]
        self.sim.data.ctrl[(self.id*4)+3]   = u[3]
        #print("u: {}".format(u))
        self.log_u.append(u)

        self.log_th.append(R.dot(w[:3]))
        self.log_tor.append(w[3:])


"""
Class to manage and store the PID parameters

Parameters
----------
mass
inertia
KZ:     PID constants for Z axis
KXY:    PID constants for X and Y axis
KRP:    PID constants for Roll and Pitch
KY:     PID constants for Yaw

"""
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

"""
Function to convert quaternion to euler angles

Parameters
----------
quat:           quaternion

Return
------
Euler angles:   np.array([roll, pitch, yaw])
"""
def quat_to_rpy(quat):
    assert np.shape(quat) == (4,)
    roll = np.arctan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]), 1 - 2 * (quat[1] ** 2 + quat[2] ** 2))
    pitch = np.arcsin(2 * (quat[0] * quat[2] - quat[3] * quat[1]))
    yaw = np.arctan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]), 1 - 2 * (quat[2] ** 2 + quat[3] ** 2))
    return np.array([roll, pitch, yaw])

"""
Covert a quaternion into a full three-dimensional rotation matrix.

Parameters
----------
quat:           quaternion

Return
------
rot_matrix:     3 dimensional rotation matrix
"""
def quat2rot(quat):
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


"""
Convert euler angles (roll, pitch, yaw) into quaternion (qx, qy, qz, qw)

Parameters
----------
roll
pitch
yaw

Return
------
quaternion:     np.array([qx, qy, qz, qw])
"""
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

"""
Convert a skew-symmetric matrix to the corresponding array

Parameters
----------
skew_s:     Skew-symmetric matrix

Return
------
Corresponding array for skew-symmetric matrix
"""
def vee_map(skew_s):
    return np.array([skew_s[2, 1], skew_s[0, 2], skew_s[1, 0]])


"""
Create null space
"""
def null_space(A, rcond=None):
    u, s, vh = np.linalg.svd(A, full_matrices=True)
    M, N = u.shape[0], vh.shape[1]
    if rcond is None:
        rcond = np.finfo(s.dtype).eps * max(M, N)
    tol = np.amax(s) * rcond
    num = np.sum(s > tol, dtype=int)
    Q = vh[num:, :].T.conj()
    return Q
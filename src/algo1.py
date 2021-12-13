from quadrotor import Quadrotor, PID_param
from mujoco_py import load_model_from_path, MjSim, MjViewer
import numpy as np
import math
import time
import copy
import sys

"""
Main Method
"""
if __name__ == "__main__":
    # List of Quadrotor objects
    Quadrotors = []
    # Number of quadrotors
    N = None

    # Ensure that model file passed in
    if len(sys.argv) < 2:
        print("\nUSAGE:   python3 main.py [Nquadrotor.xml]\n")
        exit(1)
    # Loads model from path and extracts N number of quadrotors
    try:
        filename = sys.argv[1]
        N = int(''.join(map(str,[int(x) for x in filename if x.isdigit()])))
        assert(int(N))
        model = load_model_from_path(filename)
    except:
        print("\nUSAGE:   python3 main.py [(N>1)quadrotor.xml]\n")
        print("🚨Ensure that you are using an XML file generated by gen_Ndrones.py🚨\n")
        exit(1)

    # represents a running simulation including its state
    sim = MjSim(model)

    # Display GUI showing the scene of an MjSim with a mouse-movable camera
    viewer = MjViewer(sim)

    # Returns a copy of the simulator state
    sim_state = sim.get_state()




    for i in range(N):
        # Append N quadrotors to object list
        Quadrotors.append(
            Quadrotor( PID_param(0.4, 0.05,
                            (8.0, 7, 0.01),
                            (4.0, 10.0, 0.0),
                            (4.0, 7.0, 0.0),
                            (10.0, 7.0, 0.0)),
                            sim, i
            )
        )
    
    """
    d is the distance between agents
    r is the radius of the FOV
    """
    d = 2
    r = 1.25 * d

    """
    σ norm from (8)

    For smooth collective potential of flock
    and spatial adjacency matrix
    """
    ϵ = .1                                                  # How smooth the function is
    σ_norm = lambda z: (np.sqrt(1+ϵ * z**2) - 1) / ϵ        # Nonnegative map
    σ_norm_vec = lambda z: (np.sqrt(1 + ϵ * np.linalg.norm(z, axis=1)**2) - 1) / ϵ
    σ_norm_single_vec = lambda z: (np.sqrt(1 + ϵ * np.linalg.norm(z)**2) - 1) / ϵ

    """
    GRADIENT FUNCTION (derivative) of (8)
    σ_ϵ in (9)
    """
    σ_ϵ = lambda z: z / np.sqrt(1+ϵ * z**2)

    # parameters in the sigma world
    r_α = σ_norm(r)
    d_α = σ_norm(d)

    """
    BUMP FUNCTION
    ρ_h in (10)

    h: The point which the function begins to descend toward zero
    """
    def ρ_h(z, h=0.5):
        #if 0<= z < h:
         #return 1
        #elif h<= z < 1:
         #return (1+np.cos(math.pi*(z-h)/(1-h)))/2
        #else:
         #return 0
        z2 = copy.copy(z)
        z2[(0<= z) * (z<h)] = 1                             # If z [0,h)               
    
        Zr = (h<= z) * (z < 1)                              # Interesting way to do &&...If z [h, 1]
        z2[Zr] = (1+np.cos(math.pi*(z[Zr]-h)/(1-h)))/2      
        z2[z > 1] = 0                                       # Else, set to zero
        z2[z < 0] = 0
        return z2

    """
    Spatial Adjacency Matrix
    a_ig in (11)
    """
    a_ij = lambda z: ρ_h(σ_norm(z) / r_α)
    a_ij_vec = lambda z: ρ_h(σ_norm_vec(z) / r_α)
    a_ij_single_vec = lambda z: ρ_h(σ_norm_single_vec(z) / r_α)

    """
    ACTION FUNCTION
    From equation (15)

    PARAMETERS
    ----------
    a,b:    Seem to determine how fast agents move to converge
            should keep low for drones not to have too much momentum
    """
    a, b = .15, .25                                             # Parameters that guarentee ϕ(0)=0
    c = abs(a-b)/np.sqrt(4*a*b)           

    σ1 = lambda z: z/np.sqrt(1+z**2)                        # Uneven Sigmoidal function
    ϕ_α = lambda z: ρ_h(z/r_α) * ϕ(z-d_α)                   # Action function that vanishes for z >= r_α
    ϕ = lambda z: ((a+b) * σ1(z+c) + (a-b)) / 2             # Equation 15, action function

    """
    STRESS ELEMENTS
    s_ij in (43)
    """
    s_ij = lambda z: ϕ_α(σ_norm(z)) / (1 + ϵ*σ_norm(z))
    s_ij_vec = lambda z: ϕ_α(σ_norm_vec(z)) / (1 + ϵ*σ_norm_vec(z))
    s_ij_single_vec = lambda z: ϕ_α(σ_norm_single_vec(z)) / (1 + ϵ*σ_norm_single_vec(z))

    """
    HELPER FOR ALGORITHM 1
    Equivalent to N_i(q) in paper, which returns the neighbors of
    robot i given the set of all positions q

    PARAMETERS
    ----------
    in_i:       Index of specific agent
    in_q:       position input
    in_r:       Radius
    sim:        Simulation object

    RETURN
    ------
    List of neighbors of agent i

    MJReference:
    p = self.sim.data.get_body_xpos(self.name)
    v = self.sim.data.get_body_xvelp(self.name)
    """
    def neighbors(in_i, in_q, in_r=r):
        Ns = []
        for j in range(N):
            if in_i == j:
                continue
            elif np.sqrt(np.sum((sim.data.get_body_xpos("quadrotor"+str(j)) - in_q)**2)) < in_r:
                Ns.append(j)
        return Ns

    """
    ALGORITHM 1
    Eq (23)

    PARAMETERS
    ----------
    in_q:       position input
    in_p:       velocity input
    in_i:       Index of specific agent

    RETURN
    ------
    Input acceleration to agent

    MJReference:
    p = self.sim.data.get_body_xpos(self.name)
    v = self.sim.data.get_body_xvelp(self.name)
    """
    def u_i_α(in_q, in_p, in_i):
    #     import pdb; pdb.set_trace()
        N_i = neighbors(in_i, in_q)             # Indices of neighbors
        # Do not run if no neighbors
        if (N_i):
            q_j =  []                           # Vector of neighbor positions
            p_j =  []                           # Vector of neighbor velocities
            zq  =  []                           # Delta position
            zp  =  []                           # Delta velocity
            for idx in N_i:
                name = "quadrotor"+str(idx)
                q_j.append(sim.data.get_body_xpos(name))
                p_j.append(sim.data.get_body_xvelp(name))
            zq.append(q_j - in_q)
            zp.append(p_j - in_p)
            # TODO: kinda jank having unnecessary nested lists
            aij = a_ij_vec(zq[0])                  # Spatial adjacency matrix
            sij = s_ij_vec(zq[0])                  # Stress elements associated with each edge (i,j) of proximity net G(q)
        #     import pdb; pdb.set_trace()
            term1 = np.transpose(sij* np.asarray(zq).transpose())
            term2 = np.transpose(aij * np.asarray(zp).transpose())
            return np.sum(term1 + term2, axis=0) - 0.3*in_q - 2.5*in_p   # Add a rendevouz point...so alg2
            #return np.sum(term1 + term2, axis=0)   # See Paragraph around Eq 43
        else:
            return np.array([[0,0,0]])

    first = True
    # Set by the xml file
    dt    = 0.01
    while (True):
        for i in range(N):
            if (not first):
                curr_pose = Quadrotors[i].curr_pose
                curr_vel  = Quadrotors[i].curr_vel
                u = u_i_α(np.asarray(curr_pose), np.asarray(curr_vel), i)
                # TODO: Fix this jank
                u = u[0]

                # We only send 2D commands, keep desired z-pos fixed
                Quadrotors[i].control(
                            np.array([curr_pose[0]+curr_vel[0]*dt, curr_pose[1]+curr_vel[1]*dt, 1]),                            # xyz pose Desired
                            np.array([0, 0, 0, 0]),                                                                             # Rotation Quaternion Desired
                            np.array([0, 0 , 0]),                                                                               # Roll Pitch Yaw Desired
                            (np.array([curr_vel[0]+u[0]*dt, curr_vel[1]+u[1]*dt, 0]), np.array([0,0,0])),                       # Velocity Desired
                            (np.array([u[0], u[1], 0]), np.array([0,0,0]))                                                      # Acceleration Desired
                )
            else:
                # This should only run once to initialize object fields
                Quadrotors[i].control(np.array([0, 0, 0]),                               # xyz pose Desired
                            np.array([0, 0, 0, 0]),                                      # Rotation Quaternion Desired
                            np.array([0, 0 , 0]),                                        # Roll Pitch Yaw Desired
                            (np.array([0,0,0]), np.array([0,0,0])),                      # Velocity Desired
                            (np.array([0,0,0]), np.array([0,0,0]))                       # Acceleration Desired
                )
        first = False

        # Advances the simulation by calling mj_step
        sim.step()
        viewer.render()
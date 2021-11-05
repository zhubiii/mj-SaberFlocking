#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

mjtNum* quat_to_rpy(const mjtNum* quat)
{
    mjtNum* ret = new mjtNum[3];
    mjtNum roll = mju_atan2(
        2*(quat[0]*quat[1]+quat[2]*quat[3]),
        1-2*(mju_pow(quat[1], 2)+mju_pow(quat[2],2)
    ));
    mjtNum pitch = mju_asin(2*(quat[0]*quat[1]-quat[3]*quat[1]));
    mjtNum yaw = mju_atan2(
        2*(quat[0]*quat[3]+quat[1]*quat[2]),
        1-2*(mju_pow(quat[2], 2)+mju_pow(quat[3],2)
    ));
    ret[0] = roll, ret[1] = pitch, ret[2] = yaw;
    return ret;
}

// simple controller applying damping to each dof
void mycontroller(const mjModel* m, mjData* d)
{
    if( m->nu==m->nv )
    {
        int z_p = 0, kp = 2, kd = 1, ki = 0;
        mjtNum kpp2[3] = {5, 5, 1};
        mjtNum kdd2[3] = {2, 2, 1};

        int kp3_x, kd3_x = 2;
        int kp3_y, kd3_y = 2;
        int kp3_z, kd3_z = 5;

        int z_accel = 0;

        mjtNum x_t = 5/2, y_t = -0.35, z_t = 1;
        mjtNum x_r[3] = {x_t, y_t, z_t};

        mjtNum tolerance = 0.05;
        mjtNum dt = 0.01;
        int R = 3;              // Trajectory radius
        mjtNum w = 2.0;         // Angular Speed

        int f_z = 0, w_v = 5, R_v = 20;

        // Orientation
        // PSOE
        mjtNum* curr_rpos = d->xquat;
        mjtNum* curr_rvel = d->qvel;

        mjtNum* curr_pos = d->xpos;
        mjtNum* curr_vel = d->qvel; // Is this correct?

        int ae_x, ae_y, ae_z = 0;
        mjtNum e_vx_k = 0.0;
        int ae_z_k = 0;

        // Supposed to be in loop
        mjtNum* x_p = curr_pos;
        
        mjtNum* curr_rposadj = quat_to_rpy(curr_rpos);

        mjtNum qcx = curr_rposadj[0];
        mjtNum qcy = curr_rposadj[1];
        mjtNum qcz = curr_rposadj[2];

        // how to get angular velocity?
        mjtNum qx_v = curr_rvel[0];
        mjtNum qy_v = curr_rvel[1];
        mjtNum qz_v = curr_rvel[2];


        mjtNum tmp[3] = {d->time * dt/2, 0, 0};
        // PD Control - Position
        mjtNum e_x[3];
        mju_sub3(e_x, x_r, tmp);
        mju_sub3(e_x, e_x, x_p);

        mju_zero3(tmp);
        mjtNum tmp2[3];
        mjtNum tmp3[3];
        mjtNum tmp4[3];
        mjtNum x_accel[3];
        for (int i=0; i<3; ++i)
            tmp[i] = kpp2[i] * e_x[i];
        mju_scl3(tmp2, tmp, 1/dt);
        mju_subFrom(tmp3, e_x, e_vx_k);
        for (int i=0; i<3; ++i)
            tmp4[i] = tmp2[i] * tmp3[i];
        mju_add3(x_accel, tmp, tmp4);

        d->ctrl[0] = x_accel[0];
        d->ctrl[1] = x_accel[1];
        d->ctrl[2] = f_z;
        d->ctrl[3] = 0;
        d->ctrl[4] = 0;
        d->ctrl[5] = 0;



        // PD Control - Orientation



    }
}

// main function
int main(int argc, const char** argv)
{
    // recommended version check
    if( mjVERSION_HEADER!=mj_version() )
    {
        printf("ERR: Version Mismatch\n");
        return 0;
    }

    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic modelfile\n");
        return 0;
    }

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Test Quadrotor", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // install control callback
    mjcb_control = mycontroller;

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
            mj_step(m, d);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}

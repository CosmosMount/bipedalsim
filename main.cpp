#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <iostream>
#include <cmath>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

#include "vmc.hpp"
#include "pid.hpp"
#include "lqr.hpp"
#include "odom.hpp"
#include "config.hpp"
#include "plotter.hpp"
#include "quaternion_ekf.hpp"

class sim
{
public:

    sim(const char* modelPath)
    {
        exit = false;
        char error[1000] = "Could not load binary model";
        model = mj_loadXML(modelPath, 0, error, 1000);
        if (!model) 
        {
            mju_error_s("Load model error: %s", error);
        }
        simdata = mj_makeData(model);
        if (!glfwInit())
            mju_error("Could not initialize GLFW");
        // create window, make OpenGL context current, request v-sync
        window = glfwCreateWindow(1200, 900, "Sim", NULL, NULL);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        // 关键步骤：将 'this' 指针存储在窗口中，供静态回调使用
        glfwSetWindowUserPointer(window, this);
        // initialize visualization data structures
        mjv_defaultCamera(&cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);
        // create scene and context
        mjv_makeScene(model, &scn, 2000);
        mjr_makeContext(model, &con, mjFONTSCALE_150);
        // qekftall GLFW mouse and keyboard callbacks
        glfwSetKeyCallback(window, keyboard);
        glfwSetCursorPosCallback(window, mouse_move);
        glfwSetMouseButtonCallback(window, mouse_button);
        glfwSetScrollCallback(window, scroll);

        id_rjoint4 = mj_name2id(model, mjOBJ_ACTUATOR, "rjoint4_actuator");
        id_rjoint1 = mj_name2id(model, mjOBJ_ACTUATOR, "rjoint1_actuator");
        id_rwheel  = mj_name2id(model, mjOBJ_ACTUATOR, "rwheel_actuator");
        
        id_ljoint4 = mj_name2id(model, mjOBJ_ACTUATOR, "ljoint4_actuator");
        id_ljoint1 = mj_name2id(model, mjOBJ_ACTUATOR, "ljoint1_actuator");
        id_lwheel  = mj_name2id(model, mjOBJ_ACTUATOR, "lwheel_actuator");

        lqr_controller.InitMatX(&refX[0], &observedX[0]);
        llen_pd.SetParam(300.0f, -10.0f, 50.0f, -50.0f);
        rlen_pd.SetParam(300.0f, -10.0f, 50.0f, -50.0f);
        roll_pd.SetParam(80.0f, 8.0f, 15.0f, -15.0f);
        cmd_len = 0.21f;
    }

    ~sim()
    {
        exit = true;
        // free visualization storage
        mjv_freeScene(&scn);
        mjr_freeContext(&con);

        // free MuJoCo model and data
        mj_deleteData(simdata);
        mj_deleteModel(model);

        // terminate GLFW
        glfwTerminate();
    }

    void run() 
    {
        control_thread = std::thread(&sim::control_loop, this);

        while (!glfwWindowShouldClose(window)) 
        {
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            {
                std::lock_guard<std::mutex> lock(mtx);
                mjv_updateScene(model, simdata, &opt, NULL, &cam, mjCAT_ALL, &scn);
            }
            mjr_render(viewport, &scn, &con);
            
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

private:

    // mujoco variables
    mjModel* model = nullptr;
    mjData* simdata = nullptr;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    GLFWwindow *window;
    int id_rjoint4, id_rjoint1, id_rwheel;
    int id_ljoint4, id_ljoint1, id_lwheel;

    // thread variables
    std::thread control_thread;
    std::atomic<bool> exit;
    std::mutex mtx;

    // sensor variables
    QuaternionEKF qekf;
    Odometry odometry;
    odometry_info_t odom;

    // vmc solver variables
    cVMCSolver lsolver;
    cVMCSolver rsolver;
    float Lqdot[2] = {0};
    float Rqdot[2] = {0};
    float Lxdot[2] = {0};
    float Rxdot[2] = {0};
    float LTp[2] = {0};
    float RTp[2] = {0};
    float LTpfdb[2] = {0};
    float RTpfdb[2] = {0};
    float TlRev[2] = {0};
    float TrRev[2] = {0};
    float llen;
    float rlen;
    float llen_dot;
    float rlen_dot;
    float prev_llen_dot = 0;
    float prev_rlen_dot = 0;
    float lphi;
    float rphi;
    float lphi_dot;
    float rphi_dot;
    float lalpha;
    float ralpha;
    float lalpha_dot;
    float ralpha_dot;
    float Tl[2] = {0};
    float Tr[2] = {0};

    // controller variables
    LQR lqr_controller;
    PDController llen_pd;
    PDController rlen_pd;
    PDController roll_pd;
    float Tout[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float observedX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float refX[10] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    // command variables
    float cmd_v;
    float cmd_dyaw;
    float cmd_roll;
    float cmd_len;

    // mouse interaction
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0;
    double lasty = 0;

    // plotter
    Plotter plotter;
    uint64_t tick_count = 0;

    /*--------------------------- GLFW相关键鼠操作获取函数，必须是 static 才能匹配 GLFW 的函数签名 -------------------------------*/
    // keyboard callback
    static void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods) 
    {
        // 获取存储的类实例指针
        sim* s = static_cast<sim*>(glfwGetWindowUserPointer(window));
        // backspace: reset simulation
        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) 
        {
            mj_resetData(s->model, s->simdata);
            mj_forward(s->model, s->simdata);
        }
    }

    // mouse button callback
    static void mouse_button(GLFWwindow *window, int button, int act, int mods) 
    {
        sim* s = static_cast<sim*>(glfwGetWindowUserPointer(window));
        // update button state
        s->button_left =
            (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        s->button_middle =
            (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        s->button_right =
            (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        // update mouse position
        glfwGetCursorPos(window, &s->lastx, &s->lasty);
    }

    // mouse move callback
    static void mouse_move(GLFWwindow *window, double xpos, double ypos) 
    {
        sim* s = static_cast<sim*>(glfwGetWindowUserPointer(window));
        // no buttons down: nothing to do
        if (!s->button_left && !s->button_middle && !s->button_right) 
        {
            return;
        }
        // compute mouse displacement, save
        double dx = xpos - s->lastx;
        double dy = ypos - s->lasty;
        s->lastx = xpos;
        s->lasty = ypos;
        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        // get shift key state
        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                            glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
        // determine action based on mouse button
        mjtMouse action;
        if (s->button_right) 
        {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } 
        else if (s->button_left) 
        {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        } 
        else 
        {
            action = mjMOUSE_ZOOM;
        }
        // move camera
        mjv_moveCamera(s->model, action, dx / height, dy / height, &s->scn, &s->cam);
    }

    // scroll callback
    static void scroll(GLFWwindow *window, double xoffset, double yoffset) 
    {
        sim* s = static_cast<sim*>(glfwGetWindowUserPointer(window));
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(s->model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &s->scn, &s->cam);
    }

    /*------------------------------------------------ 传感器数据获取辅助函数 ------------------------------------------------*/
    std::vector<float> get_sensor_data(const mjModel *model, const mjData *data, const std::string &sensor_name) 
    {
        int sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
        if (sensor_id == -1) 
        {
            std::cout << "no found sensor" << std::endl;
            return std::vector<float>();
        }
        int data_pos = model->sensor_adr[sensor_id];
        std::vector<float> sensor_data(model->sensor_dim[sensor_id]);
        for (int i = 0; i < sensor_data.size(); i++) 
        {
            sensor_data[i] = data->sensordata[data_pos + i];
        }
        return sensor_data;
    }

    /*----------------------------------------------- 控制线程，与渲染线程分开 ----------------------------------------------*/
    void control_loop()
    {
        while (!exit)
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(mtx);
                mj_step(model, simdata);

                // update sensor data
                std::vector<float> accel_data = get_sensor_data(model, simdata, "imu_accel");
                std::vector<float> gyro_data = get_sensor_data(model, simdata, "imu_gyro");
                if (accel_data.size() < 3 || gyro_data.size() < 3)
                    std::cerr << "IMU sensor data invalid" << std::endl;
                float ljoint1_pos = get_sensor_data(model, simdata, "ljoint1_pos")[0];
                float rjoint1_pos = get_sensor_data(model, simdata, "rjoint1_pos")[0];
                float ljoint4_pos = get_sensor_data(model, simdata, "ljoint4_pos")[0];
                float rjoint4_pos = get_sensor_data(model, simdata, "rjoint4_pos")[0];
                float ljoint1_vel = get_sensor_data(model, simdata, "ljoint1_vel")[0];
                float rjoint1_vel = get_sensor_data(model, simdata, "rjoint1_vel")[0];
                float ljoint4_vel = get_sensor_data(model, simdata, "ljoint4_vel")[0];
                float rjoint4_vel = get_sensor_data(model, simdata, "rjoint4_vel")[0];
                float lwheel_vel = get_sensor_data(model, simdata, "lwheel_vel")[0];
                float rwheel_vel = get_sensor_data(model, simdata, "rwheel_vel")[0];
                float ljoint1_tor = get_sensor_data(model, simdata, "ljoint1_tor")[0];
                float rjoint1_tor = get_sensor_data(model, simdata, "rjoint1_tor")[0];
                float ljoint4_tor = get_sensor_data(model, simdata, "ljoint4_tor")[0];
                float rjoint4_tor = get_sensor_data(model, simdata, "rjoint4_tor")[0];

                // quaternion ekf update
                double dt = model->opt.timestep;
                qekf.UpdateEKF(gyro_data[0], gyro_data[1], gyro_data[2], dt);
                qekf.ObserveEKF(accel_data[0], accel_data[1], accel_data[2]);

                // vmc solver update
                lsolver.Resolve(-ljoint4_pos, M_PI+ljoint1_pos);
                rsolver.Resolve(rjoint4_pos, M_PI-rjoint1_pos);

                // vmc inverse kinematics
                llen = lsolver.GetPendulumLen();
                rlen = rsolver.GetPendulumLen();
                lphi = lsolver.GetPendulumRadian();
                rphi = rsolver.GetPendulumRadian();
                lalpha = lphi-M_PI/2.0f+qekf.pitch;
                ralpha = rphi-M_PI/2.0f+qekf.pitch;
                Lqdot[0] = -ljoint4_vel;
                Lqdot[1] = ljoint1_vel;
                Rqdot[0] = rjoint4_vel;
                Rqdot[1] = -rjoint1_vel;
                lsolver.VMCVelCal(Lqdot, Lxdot);
                rsolver.VMCVelCal(Rqdot, Rxdot);
                llen_dot = Lxdot[0];
                rlen_dot = Rxdot[0];
                rphi_dot = Lxdot[1];
                lphi_dot = Rxdot[1];
                lalpha_dot = lphi_dot + qekf.pitch_dot;
                ralpha_dot = rphi_dot + qekf.pitch_dot;

                // odometry update
                float vel = 0.5f*(-lwheel_vel+rwheel_vel)*WHEEL_RADIUS;
                Eigen::Vector3f acc_body(accel_data[0], accel_data[1], accel_data[2]);
                odom = odometry.Update(qekf.q, acc_body, vel, qekf.yaw);

                // LTpfdb[0] = -ljoint1_tor;
                // LTpfdb[1] = -ljoint4_tor;
                // RTpfdb[0] = rjoint1_tor;
                // RTpfdb[1] = rjoint4_tor;

                // lsolver.VMCRevCal(TlRev, LTpfdb);
                // rsolver.VMCRevCal(TrRev, RTpfdb);

                // float Pl = TlRev[0]*std::cos(lalpha)+TlRev[1]/llen*std::sin(lalpha);
                // float Pr = TrRev[0]*std::cos(ralpha)+TrRev[1]/rlen*std::sin(ralpha);
                // float ddlenl = llen_dot - prev_llen_dot;
                // float ddlenr = rlen_dot - prev_rlen_dot;
                // float Nl = Pl + WHEEL_MASS*(odom.a_z - ddlenl*std::cos(lalpha));
                // float Nr = Pr + WHEEL_MASS*(odom.a_z - ddlenr*std::cos(ralpha));
                // float N = Nl + Nr;

                // lqr update
                observedX[0] = odom.x;//0.0f;//
                observedX[1] = odom.v;//0.0f;//
                observedX[2] = qekf.total_yaw;//0.0f;//
                observedX[3] = qekf.yaw_dot;//0.0f;//
                observedX[4] = lalpha;//0.0f;//
                observedX[5] = lalpha_dot;
                observedX[6] = ralpha;//0.0f;//
                observedX[7] = ralpha_dot;
                observedX[8] = qekf.pitch;
                observedX[9] = qekf.pitch_dot;

                roll_pd.SetRef(cmd_roll);
                float roll_pd_result = roll_pd.Update(qekf.roll, qekf.roll_dot);

                llen_pd.SetRef(cmd_len+0.03f+roll_pd_result);
                Tl[0] = llen_pd.Update(llen, llen_dot);

                rlen_pd.SetRef(cmd_len+0.03f-roll_pd_result);
                Tr[0] = rlen_pd.Update(rlen, rlen_dot);
                
                refX[0] = odom.x+cmd_v*0.001f;
                refX[1] = cmd_v;
                refX[2] = qekf.yaw+cmd_dyaw*0.001f;
                refX[3] = cmd_dyaw;
                refX[4] = 0.0f;
                refX[5] = 0.0f;
                refX[6] = 0.0f;
                refX[7] = 0.0f;
                refX[8] = 0.0f;
                refX[9] = 0.0f;

                lqr_controller.refreshLQRK(llen, rlen, false);
                lqr_controller.LQRCal(Tout);
                
                simdata->ctrl[id_lwheel]=-Tout[0];
                simdata->ctrl[id_rwheel]=Tout[1];

                Tl[1]=Tout[2];
                Tr[1]=Tout[3];

                // vmc forward kinematics
                lsolver.VMCCal(Tl, LTp);
                rsolver.VMCCal(Tr, RTp);

                simdata->ctrl[id_ljoint4]= -LTp[0];
                simdata->ctrl[id_ljoint1]= -LTp[1];
                simdata->ctrl[id_rjoint4]= RTp[0];
                simdata->ctrl[id_rjoint1]= RTp[1];

                if (++tick_count % 2 == 0) 
                {
                    char json_buf[2048]; // 增大缓冲区
                    snprintf(json_buf, sizeof(json_buf),
                        "{"
                        "\"time\":%.3f,"
                        // --- LQR 状态 ---
                        "\"odom_x\":%.3f,\"odom_v\":%.3f,"
                        "\"pitch\":%.3f,\"pitch_dot\":%.3f,"
                        "\"yaw\":%.3f,\"yaw_dot\":%.3f,"
                        // --- VMC 虚拟力 (输入) ---
                        "\"L_F_virt\":%.3f,\"R_F_virt\":%.3f,"    // Tl[0], Tr[0] (长度PD输出)
                        "\"L_Tau_virt\":%.3f,\"R_Tau_virt\":%.3f," // Tl[1], Tr[1] (LQR平衡力矩)
                        // --- VMC 几何状态 ---
                        "\"L_len\":%.3f,\"R_len\":%.3f,"
                        "\"L_alpha\":%.3f,\"R_alpha\":%.3f,"
                        "\"L_phi\":%.3f,\"R_phi\":%.3f,"
                        // --- 关节力矩 (最终输出) ---
                        "\"ctrl_L_j4\":%.3f,\"ctrl_L_j1\":%.3f,"
                        "\"ctrl_R_j4\":%.3f,\"ctrl_R_j1\":%.3f,"
                        "\"T_L_wheel\":%.3f,\"T_R_wheel\":%.3f,"
                        // --- VMC 逆解反馈 (地面反力估算) ---
                        "\"L_F_rev\":%.3f,\"R_F_rev\":%.3f,"       // TlRev[0], TrRev[0]
                        "\"L_Tau_rev\":%.3f,\"R_Tau_rev\":%.3f"    // TlRev[1], TrRev[1]
                        "}",
                        simdata->time,
                        // LQR
                        observedX[0], observedX[1], 
                        observedX[8], observedX[9],
                        observedX[2], observedX[3],
                        // VMC 虚拟力
                        Tl[0], Tr[0],
                        Tl[1], Tr[1],
                        // 几何
                        llen, rlen,
                        lalpha, ralpha,
                        lphi, rphi,
                        // 输出力矩 (对应 XML 里的电机)
                        simdata->ctrl[id_ljoint4], simdata->ctrl[id_ljoint1],
                        simdata->ctrl[id_rjoint4], simdata->ctrl[id_rjoint1],
                        simdata->ctrl[id_lwheel], simdata->ctrl[id_rwheel],
                        // 逆解反馈
                        TlRev[0], TrRev[0],
                        TlRev[1], TrRev[1]
                    );
                    
                    plotter.send(json_buf);
                }
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            std::chrono::duration<double> target_dt(model->opt.timestep);
            if (elapsed < target_dt) 
            {
                std::this_thread::sleep_for(target_dt - elapsed);
            }
        }
    }
};

int main()
{
    sim simulator("../mjcf/wbr_cod.xml");
    simulator.run();
    return 0;
}
#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"
#include "odom.hpp"
#include "lqr.hpp"
#include "kalmanfilter.hpp"
#include "vmc.hpp"
#include "config.hpp"

class sim
{
public:
    mjModel* model = nullptr;
    mjData* simdata = nullptr;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    GLFWwindow *window;

    // mouse interaction
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0;
    double lasty = 0;

    sim() = default;

    // keyboard callback
    // 必须是 static 才能匹配 GLFW 的函数签名
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

    void init()
    {
        char error[1000] = "Could not load binary model";
        // 注意：路径可能需要根据实际运行目录调整
        model = mj_loadXML("../mjcf/bipedal.xml", 0, error, 1000);
        if (!model) {
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

        // install GLFW mouse and keyboard callbacks
        glfwSetKeyCallback(window, keyboard);
        glfwSetCursorPosCallback(window, mouse_move);
        glfwSetMouseButtonCallback(window, mouse_button);
        glfwSetScrollCallback(window, scroll);
    }

    void end()
    {
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
        float cnt = 0;
        auto step_start = std::chrono::high_resolution_clock::now();
        while (!glfwWindowShouldClose(window)) 
        {
            step_start = std::chrono::high_resolution_clock::now();

            //测试forward
            simdata->ctrl[0] = sin(cnt);
            
            // 修复：使用 model 和 simdata 替代未定义的 m 和 d
            mj_forward(model, simdata);
            
            // std::cout << "qacc  qvel  qpos" << std::endl;
            // for (int i = 0; i < model->nq; i++) {
            //     std::cout << simdata->qacc[i] << "  " << simdata->qvel[i] << "  " << simdata->qpos[i]
            //                 << "  " << std::endl;
            // }
            // std::cout << std::endl;

            cnt += 0.01;

            //同步时间
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed_sec =
                std::chrono::duration<double>(current_time - step_start).count();
            double time_until_next_step = model->opt.timestep*5 - elapsed_sec;
            if (time_until_next_step > 0.0) {
                auto sleep_duration = std::chrono::duration<double>(time_until_next_step);
                std::this_thread::sleep_for(sleep_duration);
            }

            // get framebuffer viewport
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

            // update scene and render
            mjv_updateScene(model, simdata, &opt, NULL, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);

            // swap OpenGL buffers (blocking call due to v-sync)
            glfwSwapBuffers(window);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
        }
    }
};

int main()
{
    sim simulation;
    simulation.init();
    simulation.run();
    simulation.end();
    return 0;
}
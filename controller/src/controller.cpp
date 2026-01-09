#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include "lowlevel/msg/robot_fdb.hpp"
#include "lowlevel/msg/imu.hpp"
#include "kalmanfilter.hpp"
#include "odometry.hpp"
#include "lqr.hpp"
#include "pid.hpp"
#include "vmc.hpp"
#include "controller.hpp"

#define BIG_OFF 0.0f//0.38f//0.70f
#define SMALL_OFF 0.0f//0.55f//-0.35f

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller_node")
    {
        /*------------------- 创建消息订阅 --------------------------*/
        robotfdb_sub = this->create_subscription<lowlevel::msg::RobotFdb>(
            "/robot_feedback", 10,
            std::bind(&Controller::robotfdb_callback, this, std::placeholders::_1));
        keyboard_sub = this->create_subscription<std_msgs::msg::Int32>(
            "/keyboard/keypress", 10,
            std::bind(&Controller::keyboard_callback, this, std::placeholders::_1));

        /*------------------- 创建消息发布 ----------------------------*/
        // 控制力发布器
        rwheel_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/model/bipedal/joint/Rwheel_joint/cmd_force", 10);
        lwheel_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/model/bipedal/joint/Lwheel_joint/cmd_force", 10);
        rbigjoint_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/model/bipedal/joint/Rbig_joint/cmd_force", 10);
        lbigjoint_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/model/bipedal/joint/Lbig_joint/cmd_force", 10);
        rsmalljoint_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/model/bipedal/joint/Rsmall_joint/cmd_force", 10);
        lsmalljoint_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/model/bipedal/joint/Lsmall_joint/cmd_force", 10);

        // 腿长跟踪调试发布器（ref vs fdb）
        leg_length_ref_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/leg_length/reference", 10);
        leg_length_left_fdb_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/leg_length/left_feedback", 10);
        leg_length_right_fdb_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/leg_length/right_feedback", 10);
        leg_length_avg_fdb_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/leg_length/average_feedback", 10);

        // 摆角跟踪调试发布器（ref vs fdb）
        alpha_ref_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/alpha/reference", 10);
        alpha_fdb_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/alpha/feedback", 10);
        alphadot_fdb_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/alphadot/feedback", 10);

        // phi0 跟踪调试发布器（左右腿摆角）
        phi0_left_fdb_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/phi0/left_feedback", 10);
        phi0_right_fdb_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/phi0/right_feedback", 10);
        phi0_avg_fdb_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/phi0/average_feedback", 10);

        // Euler 角度调试发布器
        euler_pitch_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/euler/pitch", 10);
        euler_roll_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/euler/roll", 10);
        euler_yaw_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/euler/yaw", 10);
        pitchdot_fdb_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/pitchdot/feedback", 10);

        // 里程计调试发布器
        odom_x_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/odom/x", 10);
        odom_v_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/controller/odom/v", 10);

        // 控制力矩调试发布器（tau）
        tau_left_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/controller/tau/left", 10);
        tau_right_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/controller/tau/right", 10);

        // 统一的控制力发布器（包含 control_vector 和所有控制力）
        unified_control_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/controller/unified_control", 10);

        /*------------------------ 设置PID参数 --------------------------*/
        llengthpid.SetParam(300.0f, -10.0f, 50.0f, -50.0f);
        rlengthpid.SetParam(300.0f, -10.0f, 50.0f, -50.0f);
        // llengthpid.SetGains(100.0f, 0.0f, 100.0f, 50.0f, -50.0f);
        // rlengthpid.SetGains(100.0f, 0.0f, 100.0f, 50.0f, -50.0f);
        phi0pid.SetParam(10.0f, -1.0f, 10.0f, -10.0f);
        // yawpid.SetGains(18.0f, 0.0f, 4.5f);
        // yawdotpid.SetGains(1.0f, 0.0f, 0.6f);
        // rollpid.SetGains(0.7f, 0.0f, 0.01f);

        InitializeInertialParams();

        test_start_time_ = this->now();
        this->declare_parameter("test_mode", "both");    // "none", "leg_length", "alpha", "both"
        this->declare_parameter("leg_amplitude", 0.03);  // 腿长振幅 (m)
        this->declare_parameter("leg_frequency", 0.5);   // 腿长频率 (Hz)
        this->declare_parameter("alpha_amplitude", 0.3); // 摆角振幅 (rad)
        this->declare_parameter("alpha_frequency", 0.3); // 摆角频率 (Hz)
    }

    /* 消息订阅函数 */
    void robotfdb_callback(const lowlevel::msg::RobotFdb::SharedPtr msg)
    {
        if (!IsSystemReady(msg))
        {
            PublishZeroTorques();
            return;
        }

        Eigen::Quaternionf imu_q(msg->imu.qw, msg->imu.qx, msg->imu.qy, msg->imu.qz);
        Eigen::Vector3f imu_acc(msg->imu.acc_x, msg->imu.acc_y, msg->imu.acc_z);
        QuaternionToEuler(imu_q, Euler);
        Gyro = Eigen::Vector3f(msg->imu.gyro_x, msg->imu.gyro_y, msg->imu.gyro_z);
        float vel = (msg->l_wheel.spd_fdb - msg->r_wheel.spd_fdb) * WHEEL_RADIUS * 0.5f;
        odom.Odometry_Update(imu_q, imu_acc, vel, Euler.z());

        LegControl(PI+msg->l_big.pos_fdb+BIG_OFF, msg->l_small.pos_fdb+msg->l_big.pos_fdb+BIG_OFF+0.64f+SMALL_OFF,
                   PI-(msg->r_big.pos_fdb-BIG_OFF), -msg->r_small.pos_fdb-(msg->r_big.pos_fdb-BIG_OFF)+0.58f+SMALL_OFF,
                    msg->l_big.spd_fdb, msg->l_small.spd_fdb,
                    -msg->r_big.spd_fdb, -msg->r_small.spd_fdb);
    }

    void keyboard_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        return;
        int key_code = msg->data;
        
        // 调试输出：显示接收到的按键码
        RCLCPP_INFO(this->get_logger(), "Received key code: %d", key_code);
        
        double wheel_max = 8.0f;
        std::string action = "";
        
        switch(key_code) {
            case 87:  // W - Forward
                v_ref += 0.1f;
                action = "Forward";
                break;
                
            case 83:  // S - Backward
                v_ref -= 0.1f;
                action = "Backward";
                break;
                
            case 65:  // A - Turn Left
                t_rwheel = -wheel_max * 0.7;
                t_lwheel = -wheel_max * 0.7;
                action = "Turn Left";
                break;
                
            case 68:  // D - Turn Right
                t_rwheel = wheel_max * 0.7;
                t_lwheel = wheel_max * 0.7;
                action = "Turn Right";
                break;
                
            default:
                // 未知按键，停止
                v_ref = 0.0f;
                break;
        }
    }

    /* 消息发布函数 */
    void publishtorque()
    {
        auto rwheel_msg = std_msgs::msg::Float64();
        rwheel_msg.data = t_rwheel;
        rwheel_pub->publish(rwheel_msg);

        auto lwheel_msg = std_msgs::msg::Float64();
        lwheel_msg.data = t_lwheel;
        lwheel_pub->publish(lwheel_msg);

        auto rbigjoint_msg = std_msgs::msg::Float64();
        rbigjoint_msg.data = t_rbig;
        rbigjoint_pub->publish(rbigjoint_msg);

        auto lbigjoint_msg = std_msgs::msg::Float64();
        lbigjoint_msg.data = t_lbig;
        lbigjoint_pub->publish(lbigjoint_msg);

        auto rsmalljoint_msg = std_msgs::msg::Float64();
        rsmalljoint_msg.data = t_rsmall;
        rsmalljoint_pub->publish(rsmalljoint_msg);

        auto lsmalljoint_msg = std_msgs::msg::Float64();
        lsmalljoint_msg.data = t_lsmall;
        lsmalljoint_pub->publish(lsmalljoint_msg);
    }

private:

    /*=============================================== 控制器 =========================================================*/

    /*定义常量*/
    const float max_force = 100.0;
    const float Lmin = 0.14f;
    const float Lmax = 0.27f;

    /*定义控制量*/
    float yaw_ref = 0.0f;
    float v_ref = 0.0f;

    Eigen::Vector2f FTl;
    Eigen::Vector2f FTr;
    Eigen::Vector2f t_l;
    Eigen::Vector2f t_r;
    float t_rwheel = 0.0f;
    float t_lwheel = 0.0f;
    float t_rbig = 0.0f;
    float t_lbig = 0.0f;
    float t_rsmall = 0.0f;
    float t_lsmall = 0.0f;

    float l_ref = 0.20f;
    float alpha_ref_ = 0.0f;
    float phi0_ref = PI * 0.5f;
    Eigen::Vector2f lctrl;
    Eigen::Vector2f rctrl;
    Eigen::Vector3f Euler;
    Eigen::Vector3f Gyro;

    /*定义控制器*/
    cVMCSolver lsolver;
    cVMCSolver rsolver;
    Odometry odom;
    LQRControllerf lqr_controller;
    PDController phi0pid;
    PDController llengthpid;
    PDController rlengthpid;
    PIDControllerf yawpid;
    PIDControllerf yawdotpid;
    PIDControllerf rollpid;

    /*定义LQR量*/
    LQRControllerf::StateVector state_ref;
    LQRControllerf::StateVector state_obs;
    LQRControllerf::ControlVector control_vector;
    LQRControllerf::GainMatrix gain_matrix;

    /*定义控制接收与发布*/
    rclcpp::Subscription<lowlevel::msg::RobotFdb>::SharedPtr robotfdb_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr keyboard_sub;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rwheel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lwheel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rbigjoint_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lbigjoint_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rsmalljoint_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lsmalljoint_pub;

    void LegControl(float lphi1, float lphi2, float rphi1, float rphi2, float lphi1dot, float lphi2dot,
                    float rphi1dot, float rphi2dot)
    {
        // UpdateTestTargets();
        lsolver.Resolve(lphi1, lphi2);
        rsolver.Resolve(rphi1, rphi2);

        Eigen::Vector2f ldot;
        Eigen::Vector2f rdot;
        lsolver.VMCVelCal(Eigen::Vector2f(lphi1dot, lphi2dot), ldot);
        rsolver.VMCVelCal(Eigen::Vector2f(rphi1dot, rphi2dot), rdot);

        float lphi0 = lsolver.GetPendulumRadian();
        float rphi0 = rsolver.GetPendulumRadian();
        float L = 0.5f * (lsolver.GetPendulumLen() + rsolver.GetPendulumLen());

        // 计算实际摆角
        float alpha_fdb = 0.5f * (lphi0 + rphi0 - PI) + Euler(1);
        float alphadot_fdb = 0.5f * (ldot.y() + rdot.y()) + Gyro(1);
        float pitch_fdb = Euler(1);
        float pitchdot_fdb = Gyro(1);

        llengthpid.SetRef(l_ref);
        rlengthpid.SetRef(l_ref);

        state_obs << alpha_fdb, alphadot_fdb,
                      odom.GetOdomData().x, odom.GetOdomData().v,//0.0f, 0.0f,//
                      pitch_fdb, pitchdot_fdb;
        
        state_ref << alpha_ref_, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

        control_vector = lqr_controller.Update(state_obs, state_ref);

        phi0pid.SetRef(0.0f);
        float phi0_comp = phi0pid.Update(lphi0-rphi0, ldot.y()-rdot.y());//0.0f;//

        // 限幅
        auto clamp = [](float val, float limit) { return std::clamp(val, -limit, limit); };

        control_vector(0) = clamp(control_vector(0), max_force);
        control_vector(1) = clamp(control_vector(1), max_force);

        lctrl(0) = 0.0f;//llengthpid.Update(lsolver.GetPendulumLen(), ldot.x());
        rctrl(0) = 0.0f;//rlengthpid.Update(rsolver.GetPendulumLen(), rdot.x());
        lctrl(1) = control_vector(1)*0.5f;//+phi0_comp;//
        rctrl(1) = control_vector(1)*0.5f;//-phi0_comp;//

        lctrl(0) = clamp(lctrl(0), max_force);
        rctrl(0) = clamp(rctrl(0), max_force);
        lctrl(1) = clamp(lctrl(1), max_force);
        rctrl(1) = clamp(rctrl(1), max_force);

        Eigen::Vector2f ltau;
        Eigen::Vector2f rtau;
        
        lsolver.VMCCal(lctrl, ltau);
        rsolver.VMCCal(rctrl, rtau);

        // 添加重力补偿
        Eigen::Vector2f lgrav_comp = CalculateLeftLegGravityComp(lphi1, lphi2);
        Eigen::Vector2f rgrav_comp = CalculateRightLegGravityComp(rphi1, rphi2);
        
        // t_lbig = ltau(0) ;//+ lgrav_comp(0);
        // t_lsmall = ltau(1) ;//+ lgrav_comp(1);
        // t_rbig = -rtau(0) ;//+ rgrav_comp(0);   // 注意右侧符号
        // t_rsmall = -rtau(1) ;//+ rgrav_comp(1);
        
        
        // t_lbig = clamp(t_lbig, max_force);
        // t_lsmall = clamp(t_lsmall, max_force);
        // t_rbig = clamp(t_rbig, max_force);
        // t_rsmall = clamp(t_rsmall, max_force);

        t_lbig = control_vector(1)*0.5f+phi0_comp;
        t_rbig = -(control_vector(1)*0.5f-phi0_comp);

        t_lwheel = control_vector(0)*0.5f;
        t_rwheel = -control_vector(0)*0.5f;
        
        publishtorque();

        /*----------------------------------------- 调试输出 ----------------------------------------------*/
        // 发布腿长跟踪数据（ref vs fdb）
        PublishLegLengthTracking(l_ref, L, lsolver.GetPendulumLen(), rsolver.GetPendulumLen());
        // 发布摆角跟踪数据（ref vs fdb）
        PublishAlphaTracking(alpha_fdb, alphadot_fdb);
        // 发布 phi0 跟踪数据（左右腿摆角）
        PublishPhi0Tracking(lphi0, rphi0);
        // 发布 Euler 角度数据
        PublishEulerAngles(pitchdot_fdb);
        // 发布里程计数据
        PublishOdomData(odom.GetOdomData().x, odom.GetOdomData().v);
        // 发布控制力矩（tau）
        PublishTauValues(ltau, rtau);
        // 发布统一的控制力数据（包含 control_vector 和腿长控制力）
        PublishUnifiedControl(lctrl, rctrl, ltau, rtau);

        // RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 500,
        //     "\n [VMC Logger] lphi1: %.2f deg, lphi2: %.2f deg, lphi0: %.2f deg, lphidot: %.2f deg/s; 
        //     \n [VMC Logger] rphi1: %.2f deg, rphi2: %.2f deg, rphi0: %.2f deg, rphidot: %.2f deg/s; 
        //     \n [VMC Logger] llength: %.3f m, rlength: %.3f m, L: %.3f m, vL: %.2f m/s 
        //     \n [VMC Logger] tau: ltau1: %.2f Nm, ltau2: %.2f Nm, rtau1: %.2f Nm, rtau2: %.2f Nm",
        //     lphi1*RAD2DEG, lphi2*RAD2DEG, lphi0*RAD2DEG, ldot.y()*RAD2DEG,
        //     rphi1*RAD2DEG, rphi2*RAD2DEG, rphi0*RAD2DEG, rdot.y()*RAD2DEG,
        //     lsolver.GetPendulumLen(), rsolver.GetPendulumLen(), L, (ldot.x()+rdot.x())*0.5f,
        //     ltau.x(), ltau.y(), rtau.x(), rtau.y());        
        // RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 500,
        //     "\n [LQR Logger] alpha: %.2f rad, alphadot: %.2f rad/s, \n [LQR Logger] pitch: %.2f rad, pitchdot: %.2f rad/s;",
        //     (0.5f*(lphi0+rphi0-PI)+Euler(1)), (0.5f*(ldot.y()+rdot.y())+Gyro(1)), Euler(1), Gyro(1));
        // RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(), 500,
        //     "\n [Force Logger] Control Torques - FLl: %.2f N, FLr: %.2f N, Tp: %.2f N
        //     \n [Force Logger] Lbig: %.2f Nm, Lsmall: %.2f Nm, Rbig: %.2f Nm, Rsmall: %.2f Nm; 
        //     \n [Force Logger] Wheel Torques (Nm) - Left: %.2f, Right: %.2f",
        //     lctrl(0), rctrl(0), control_vector(1), t_lbig, t_lsmall, t_rbig, t_rsmall, t_lwheel, t_rwheel);
    }

    /*=================================================== 重力补偿 =========================================================*/
    // 惯性参数结构
    struct LinkInertial {
        float mass;                  // 质量 (kg)
        Eigen::Vector3f com_offset;  // 质心偏移 (相对关节，m)
        float link_length;           // link长度 (m)
    };
    
    LinkInertial lbig_params;
    LinkInertial lsmall_params;
    LinkInertial rbig_params;
    LinkInertial rsmall_params;
    
    const float g = 9.81f;  // 重力加速度
    
    void InitializeInertialParams()
    {
        // 左大腿 (Lbig_Link) - 从 SDF 提取
        lbig_params.mass = 0.279663f;
        lbig_params.com_offset = Eigen::Vector3f(-0.09915f, 0.010957f, -0.007039f);
        lbig_params.link_length = 0.214311f;  // 到 Lsmall_joint 的距离
        
        // 左小腿 (Lsmall_Link)
        lsmall_params.mass = 0.251916f;
        lsmall_params.com_offset = Eigen::Vector3f(0.079592f, 0.007638f, -0.04948f);
        lsmall_params.link_length = 0.216526f;  // 到 Lwheel_joint 的距离
        
        // 右大腿 (Rbig_Link)
        rbig_params.mass = 0.273515f;
        rbig_params.com_offset = Eigen::Vector3f(-0.098958f, -0.010875f, 0.001169f);
        rbig_params.link_length = 0.214999f;
        
        // 右小腿 (Rsmall_Link)
        rsmall_params.mass = 0.251916f;
        rsmall_params.com_offset = Eigen::Vector3f(0.088576f, -0.007215f, -0.055792f);
        rsmall_params.link_length = 0.214846f;
    }
    
    // 计算单个 link 的重力补偿力矩
    float CalculateGravityTorque(const LinkInertial& params, float joint_angle, 
                                 const Eigen::Vector3f& base_orientation)
    {       
        // 1. 计算质心在世界坐标系中的位置方向
        float pitch = base_orientation(1);  // 基座俯仰角
        
        // 2. 考虑基座俯仰 + 关节角度的总角度
        float total_angle = pitch + joint_angle;
        
        // 3. 质心相对关节轴的力臂（主要是 x 方向的偏移）
        float lever_arm = params.com_offset.x();  // 沿link方向的质心偏移
        
        // 4. 重力矩 = m * g * r * sin(θ)
        // sin(total_angle) 表示重力在垂直于link方向的分量
        float gravity_torque = params.mass * g * lever_arm * std::sin(total_angle);
        
        return gravity_torque;
    }
    
    // 左腿重力补偿（考虑串联效应）
    Eigen::Vector2f CalculateLeftLegGravityComp(float lphi1, float lphi2)
    {
        Eigen::Vector2f gravity_comp;
        
        // 大腿关节 (Lbig_joint) 的重力补偿
        // 需要补偿：自身重力 + 小腿及轮子的重力
        float lbig_self = CalculateGravityTorque(lbig_params, lphi1, Euler);
        
        // 小腿对大腿的附加力矩（通过杠杆作用）
        float lsmall_effect_on_big = lsmall_params.mass * g * 
                                     lbig_params.link_length * std::sin(lphi1 + Euler(1));
        
        gravity_comp(0) = lbig_self + lsmall_effect_on_big;
        
        // 小腿关节 (Lsmall_joint) 的重力补偿
        // 只需补偿自身和轮子的重力
        gravity_comp(1) = CalculateGravityTorque(lsmall_params, lphi2, Euler);
        
        return gravity_comp;
    }
    
    // 右腿重力补偿（镜像逻辑，注意轴方向相反）
    Eigen::Vector2f CalculateRightLegGravityComp(float rphi1, float rphi2)
    {
        Eigen::Vector2f gravity_comp;
        
        // 右侧轴方向为负，需要取反
        float rbig_self = -CalculateGravityTorque(rbig_params, -rphi1, Euler);
        float rsmall_effect_on_big = rsmall_params.mass * g * 
                                     rbig_params.link_length * std::sin(-rphi1 + Euler(1));
        
        gravity_comp(0) = rbig_self + rsmall_effect_on_big;
        gravity_comp(1) = -CalculateGravityTorque(rsmall_params, -rphi2, Euler);
        
        return gravity_comp;
    }

    /*====================================== 正弦测试目标 ======================================*/
    rclcpp::Time test_start_time_;
    void UpdateTestTargets()
    {
        std::string test_mode = this->get_parameter("test_mode").as_string();
        if (test_mode == "none") return;
        
        double elapsed = (this->now() - test_start_time_).seconds();
        
        // 腿长正弦测试
        if (test_mode == "leg_length" || test_mode == "both") {
            double leg_amp = this->get_parameter("leg_amplitude").as_double();
            double leg_freq = this->get_parameter("leg_frequency").as_double();
            
            // l_ref = 0.22 ± amplitude * sin(2π * freq * t)
            l_ref = 0.22f + leg_amp * std::sin(2.0 * M_PI * leg_freq * elapsed);
            
            // 限制在安全范围
            l_ref = std::clamp(l_ref, Lmin + 0.01f, Lmax - 0.01f);
        }
        
        // 摆角正弦测试（通过修改 state_ref）
        if (test_mode == "alpha" || test_mode == "both") {
            double alpha_amp = this->get_parameter("alpha_amplitude").as_double();
            double alpha_freq = this->get_parameter("alpha_frequency").as_double();
            alpha_ref_ = 0.1f + alpha_amp * std::sin(2.0 * M_PI * alpha_freq * elapsed);
        } else {
            alpha_ref_ = 0.0f;
        }
    }

    /*============================================= 系统检查 =======================================================*/

    bool IsSystemReady(const lowlevel::msg::RobotFdb::SharedPtr& msg)
    {
        // 检查四元数是否有效
        if (std::isnan(msg->imu.qw) || std::isnan(msg->imu.qx) || 
            std::isnan(msg->imu.qy) || std::isnan(msg->imu.qz)) {
            return false;
        }
        
        // 检查四元数归一化
        float q_norm = std::sqrt(msg->imu.qw*msg->imu.qw + msg->imu.qx*msg->imu.qx + 
                                 msg->imu.qy*msg->imu.qy + msg->imu.qz*msg->imu.qz);
        if (std::abs(q_norm - 1.0f) > 0.1f) {
            return false;
        }
        
        // 检查加速度是否有效
        if (std::isnan(msg->imu.acc_x) || std::isnan(msg->imu.acc_y) || 
            std::isnan(msg->imu.acc_z)) {
            return false;
        }
        
        // 检查关节位置是否有效
        if (std::isnan(msg->l_big.pos_fdb) || std::isnan(msg->l_small.pos_fdb) ||
            std::isnan(msg->r_big.pos_fdb) || std::isnan(msg->r_small.pos_fdb)) {
            return false;
        }
        
        return true;
    }
    
    void PublishZeroTorques()
    {
        t_lbig = 0.0f;
        t_lsmall = 0.0f;
        t_rbig = 0.0f;
        t_rsmall = 0.0f;
        t_lwheel = 0.0f;
        t_rwheel = 0.0f;
        
        publishtorque();
    }
    
    /*============================================= 调试发布器 =======================================================*/
       
    // 腿长跟踪调试发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leg_length_ref_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leg_length_left_fdb_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leg_length_right_fdb_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leg_length_avg_fdb_pub;
    
    // 摆角跟踪调试发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr alpha_ref_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr alpha_fdb_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr alphadot_fdb_pub;
    
    // phi0 跟踪调试发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr phi0_left_fdb_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr phi0_right_fdb_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr phi0_avg_fdb_pub;
    
    // Euler 角度发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr euler_pitch_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr euler_roll_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr euler_yaw_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitchdot_fdb_pub;
    
    // 里程计发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr odom_x_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr odom_v_pub;
    
    // 控制力矩发布器（tau）
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_left_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_right_pub;
    
    // 统一的控制力发布器
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr unified_control_pub;

    // 发布腿长跟踪数据
    void PublishLegLengthTracking(float L_ref, float L_avg, float L_left, float L_right)
    {
        std_msgs::msg::Float64 ref_msg, left_msg, right_msg, avg_msg;
        
        ref_msg.data = L_ref;  // 发布滤波后的参考值
        left_msg.data = L_left;
        right_msg.data = L_right;
        avg_msg.data = L_avg;
        
        leg_length_ref_pub->publish(ref_msg);
        leg_length_left_fdb_pub->publish(left_msg);
        leg_length_right_fdb_pub->publish(right_msg);
        leg_length_avg_fdb_pub->publish(avg_msg);
    }

    // 发布摆角跟踪数据
    void PublishAlphaTracking(float alpha_fdb, float alphadot_fdb)
    {
        std_msgs::msg::Float64 ref_msg, fdb_msg, alphadot_msg;
        
        ref_msg.data = alpha_ref_;
        fdb_msg.data = alpha_fdb;
        alphadot_msg.data = alphadot_fdb;
        
        alpha_ref_pub->publish(ref_msg);
        alpha_fdb_pub->publish(fdb_msg);
        alphadot_fdb_pub->publish(alphadot_msg);
    }

    // 发布 phi0 跟踪数据（左右腿摆角）
    void PublishPhi0Tracking(float lphi0, float rphi0)
    {
        std_msgs::msg::Float64 left_msg, right_msg, avg_msg;
        
        left_msg.data = lphi0;
        right_msg.data = rphi0;
        avg_msg.data = 0.5f * (lphi0 + rphi0);
        
        phi0_left_fdb_pub->publish(left_msg);
        phi0_right_fdb_pub->publish(right_msg);
        phi0_avg_fdb_pub->publish(avg_msg);
    }

    // 发布 Euler 角度数据
    void PublishEulerAngles(float pitchdot_fdb)
    {
        std_msgs::msg::Float64 pitch_msg, roll_msg, yaw_msg, pitchdot_msg;
        
        pitch_msg.data = Euler(1);  // Pitch
        roll_msg.data = Euler(0);   // Roll
        yaw_msg.data = Euler(2);    // Yaw
        pitchdot_msg.data = pitchdot_fdb;
        
        euler_pitch_pub->publish(pitch_msg);
        euler_roll_pub->publish(roll_msg);
        euler_yaw_pub->publish(yaw_msg);
        pitchdot_fdb_pub->publish(pitchdot_msg);
    }

    // 发布里程计数据
    void PublishOdomData(float x, float v)
    {
        std_msgs::msg::Float64 x_msg, v_msg;
        x_msg.data = x;
        v_msg.data = v;
        
        odom_x_pub->publish(x_msg);
        odom_v_pub->publish(v_msg);
    }

    // 发布控制力矩（tau）
    void PublishTauValues(const Eigen::Vector2f& ltau, const Eigen::Vector2f& rtau)
    {
        std_msgs::msg::Float64MultiArray left_msg, right_msg;
        
        // 左腿力矩 [tau_big, tau_small]
        left_msg.data.resize(2);
        left_msg.data[0] = ltau(0);  // 左大腿力矩
        left_msg.data[1] = ltau(1);  // 左小腿力矩
        
        // 右腿力矩 [tau_big, tau_small]
        right_msg.data.resize(2);
        right_msg.data[0] = rtau(0);  // 右大腿力矩
        right_msg.data[1] = rtau(1);  // 右小腿力矩
        
        tau_left_pub->publish(left_msg);
        tau_right_pub->publish(right_msg);
    }

    // 发布统一的控制力数据（包含 control_vector 和腿长控制力）
    void PublishUnifiedControl(const Eigen::Vector2f& lctrl, const Eigen::Vector2f& rctrl,
                               const Eigen::Vector2f& ltau, const Eigen::Vector2f& rtau)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(10);
        
        // LQR 控制向量
        msg.data[0] = control_vector(0)*0.01f;  // Tp (轮子力矩指令)
        msg.data[1] = control_vector(1)*0.01f;  // Fα (摆角控制力)
        msg.data[2] = lctrl(0)*0.01f;  // FLl (左腿长控制力)
        msg.data[3] = rctrl(0)*0.01f;  // FLr (右腿长控制力)
        msg.data[4] = lctrl(1);  // 左腿摆角控制力分量
        msg.data[5] = rctrl(1);  // 右腿摆角控制力分量
        msg.data[6] = ltau(0);   // 左大腿力矩
        msg.data[7] = ltau(1);   // 左小腿力矩
        msg.data[8] = -rtau(0);  // 右大腿力矩
        msg.data[9] = -rtau(1);  // 右小腿力矩
        
        unified_control_pub->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<Controller>();
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}
#ifndef VMCSOLVER_HPP
#define VMCSOLVER_HPP

#include <cmath>
#include <Eigen/Dense>

constexpr double PI = 3.14159265358979323846f;

class cVMCSolver
{
protected:
    /*雅可比矩阵 2x2*/
    Eigen::Matrix2f J;      // 雅可比矩阵
    Eigen::Matrix2f JT;     // 雅可比转置

    /*连杆长度，单位m*/
    float L1 = 0.215f;
    float L2 = 0.254f;

    /*关节角度 (弧度)*/
    float phi0 = PI / 2;   // 虚拟摆角度
    float phi1 = 0.0f;  // 电机1角度
    float phi2 = 0.0f;  // BC连杆角度
    float phi3 = 0.0f;  // CD连杆角度
    float phi4 = 0.0f;  // 电机4角度

    /*虚拟摆参数*/
    float PendulumLength = 0.0f;    // 虚拟摆长度 (L0)
    
    /*关键点坐标 (世界坐标系)*/
    Eigen::Vector2f CoorC;  // 足端点C坐标
    Eigen::Vector2f CoorB;  // 节点B坐标

public:
    cVMCSolver() 
    {
        J.setZero();
        JT.setZero();
        CoorC.setZero();
        CoorB.setZero();
    }

    /**
     * @brief 正运动学求解
     * @param phi1_radian 电机4角度 (右侧电机)
     * @param phi2_radian 电机1角度 (左侧电机)
     * @param leg_type 腿类型: 0=左腿, 1=右腿
     * 
     * 计算流程:
     * 1. 根据 phi1, phi2 计算点B和点D的位置
     * 2. 求解五连杆封闭链,得到 phi2 和 phi3
     * 3. 计算足端点C的位置
     * 4. 计算虚拟摆参数 (L0, phi0)
     * 5. 计算雅可比矩阵
     */
    void Resolve(float phi1_radian, float phi2_radian)
    {
        // 根据腿类型调整角度
        this->phi1 = phi1_radian;
        this->phi2 = phi2_radian;

        // 预计算三角函数
        float sin1 = std::sin(this->phi1);
        float cos1 = std::cos(this->phi1);
        float sin2 = std::sin(this->phi2);
        float cos2 = std::cos(this->phi2);

        // 计算节点B坐标 (左侧)
        this->CoorB << this->L1 * cos1,
                    this->L1 * sin1;

        // 计算节点C坐标 (足端点)
        this->CoorC << this->CoorB.x() + this->L2 * cos2,
                    this->CoorB.y() + this->L2 * sin2;

        // 计算虚拟摆参数
        this->phi0 = std::atan2(this->CoorC.y(), this->CoorC.x());
        this->PendulumLength = this->CoorC.norm();

        // 计算雅可比矩阵
        ComputeJacobian();
    }

    /**
     * @brief 正向VMC: 由足端力/力矩 -> 电机力矩
     * @param FT 足端力 [F_radial, Torque]
     * @param Tmotor 电机力矩 [T1, T4]
     * 
     * 公式: Tmotor = J^T * FT
     */
    void VMCCal(const Eigen::Vector2f& FT, Eigen::Vector2f& Tmotor)
    {
        Tmotor = JT * FT;
    }

    void VMCCal(const float* FT, float* Tmotor)
    {
        Eigen::Map<const Eigen::Vector2f> ft_vec(FT);
        Eigen::Map<Eigen::Vector2f> t_vec(Tmotor);
        t_vec = JT * ft_vec;
    }

    /**
     * @brief 逆向VMC: 由电机力矩 -> 足端力/力矩
     * @param Tmotor 电机力矩 [T1, T4]
     * @param FT 足端力 [F_radial, Torque]
     * 
     * 公式: FT = J * Tmotor
     */
    void VMCRevCal(Eigen::Vector2f& FT, const Eigen::Vector2f& Tmotor)
    {
        FT = J * Tmotor;
    }

    void VMCRevCal(float* FT, const float* Tmotor)
    {
        Eigen::Map<Eigen::Vector2f> ft_vec(FT);
        Eigen::Map<const Eigen::Vector2f> t_vec(Tmotor);
        ft_vec = J * t_vec;
    }

    /**
     * @brief 速度映射: 电机角速度 -> 足端速度
     * @param phi_dot 电机角速度 [phi1_dot, phi4_dot]
     * @param v_dot 足端速度 [v_radial, v_tangential]
     */
    void VMCVelCal(const Eigen::Vector2f& phi_dot, Eigen::Vector2f& v_dot)
    {
        v_dot = J * phi_dot;  // 离散化
    }

    void VMCVelCal(const float* phi_dot, float* v_dot)
    {
        Eigen::Map<const Eigen::Vector2f> phi_vec(phi_dot);
        Eigen::Map<Eigen::Vector2f> v_vec(v_dot);
        v_vec = J * phi_vec * 0.001f;
    }

    // Getter 方法
    inline float GetPendulumLen() const { return PendulumLength; }
    inline float GetPendulumRadian() const { return phi0; }
    inline float GetPhi0() const { return phi0; }
    inline float GetPhi1() const { return phi1; }
    inline float GetPhi2() const { return phi2; }


private:
    /**
     * @brief 计算雅可比矩阵
     * 
     * 基于MATLAB符号推导的结果:
     * J = [[J11, J12],
     *      [J21, J22]]
     * 
     */
    void ComputeJacobian()
    {
        // 预计算角度差的三角函数
        float sin_10 = std::sin(phi1-phi0);
        float cos_10 = std::cos(phi1-phi0);
        float sin_20 = std::sin(phi2-phi0);
        float cos_20 = std::cos(phi2-phi0);

        J(0,0) = -L1 * sin_10;
        J(0,1) = -L2 * sin_20;
        J(1,0) = L1/PendulumLength * cos_10;
        J(1,1) = L2/PendulumLength * cos_20;

        // 计算转置
        JT = J.transpose();
    }
};

#endif //VMCSOLVER_HPP
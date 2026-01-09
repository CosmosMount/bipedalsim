#pragma once

#include "kalmanfilter.hpp"
#include <cmath>
#include <iostream>

/**
 * @brief 基于四元数的扩展卡尔曼滤波器 (Quaternion-based EKF)
 * 状态量: [q0, q1, q2, q3, gyro_bias_x, gyro_bias_y] (6维)
 * 观测量: [acc_x, acc_y, acc_z] (3维)
 */
class QuaternionEKF : public DynamicKalmanFilter
{
public:
    // 物理参数与阈值
    const double Q1 = 10.0;     // 四元数过程噪声
    const double Q2 = 0.001;    // 陀螺仪偏置过程噪声
    const double ChiSquareTestThreshold = 1e-6; 
    const double lambda = 1.0;  // 衰减因子
    
    // 状态缓存
    double yaw = 0, pitch = 0, roll = 0;
    double yaw_dot = 0, pitch_dot = 0, roll_dot = 0;
    double total_yaw = 0, prev_yaw = 0;
    int16_t yaw_round_count = 0;
    
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    bool ConvergeFlag = false;
    bool StableFlag = false;
    uint64_t UpdateCount = 0;
    uint64_t ErrorCount = 0;

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity(); // 当前四元数

    QuaternionEKF() : DynamicKalmanFilter(6, 0, 3) 
    {
        // 1. 初始化状态量 (单位四元数)
        xhat << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        
        // 2. 初始化协方差矩阵 P
        P.setIdentity();
        P.diagonal() << 100000, 100000, 100000, 100000, 100, 100;

        // 3. 初始化观测噪声 R
        R = Eigen::MatrixXd::Identity(3, 3) * 100000.0;
        
        // 关闭自动维度调整，因为 EKF 结构相对固定
        useAutoAdjustment = false;
    }

    /**
     * @brief 核心更新函数
     * @param gx, gy, gz 陀螺仪数据 (rad/s)
     * @param ax, ay, az 加速度计数据 (m/s^2 或 重力单位)
     * @param dt 时间步长
     */
    void UpdateEKF(double gx, double gy, double gz, double dt)
    {
        // 移除偏置后的角速度
        double wx = gx - xhat[4];
        double wy = gy - xhat[5];
        double wz = gz - gyro_bias[2]; // 假设 z 轴偏置由外部维护或设为0

        roll_dot  = wx;
        pitch_dot = wy;
        yaw_dot = gz;

        // --- 1. 预测步 (Prediction) ---
        
        // 更新状态转移矩阵 F (线性化)
        double half_dt = 0.5 * dt;
        F.setIdentity();
        F(0, 1) = -half_dt * wx; F(0, 2) = -half_dt * wy; F(0, 3) = -half_dt * wz;
        F(1, 0) =  half_dt * wx; F(1, 2) =  half_dt * wz; F(1, 3) = -half_dt * wy;
        F(2, 0) =  half_dt * wy; F(2, 1) = -half_dt * wz; F(2, 3) =  half_dt * wx;
        F(3, 0) =  half_dt * wz; F(3, 1) =  half_dt * wy; F(3, 2) = -half_dt * wx;
        
        // 注入偏置对四元数的影响到 F 矩阵 (对应原代码中的自定义部分)
        F(0, 4) =  half_dt * xhat[1]; F(0, 5) =  half_dt * xhat[2];
        F(1, 4) = -half_dt * xhat[0]; F(1, 5) =  half_dt * xhat[3];
        F(2, 4) = -half_dt * xhat[3]; F(2, 5) = -half_dt * xhat[0];
        F(3, 4) =  half_dt * xhat[2]; F(3, 5) = -half_dt * xhat[1];

        // 更新过程噪声 Q
        Q.setZero();
        Q.diagonal() << Q1*dt, Q1*dt, Q1*dt, Q1*dt, Q2*dt, Q2*dt;

        // 执行基础预测阶段 (x- = F*x, P- = F*P*F' + Q)
        xhatMinus = F * xhat;
        xhatMinus.segment<4>(0).normalize(); // 四元数归一化
        
        Pminus = F * P * F.transpose() + Q;
        
        // 衰减因子处理 (Fading Memory)
        Pminus(4,4) /= lambda;
        Pminus(5,5) /= lambda;
    }

    /**
     * @brief 观测更新步
     */
    void ObserveEKF(double ax, double ay, double az)
    {
        // 归一化加速度计测量值
        double norm = std::sqrt(ax*ax + ay*ay + az*az);
        if(norm < 1e-6) return;
        measuredVector << ax/norm, ay/norm, az/norm;

        // --- 2. 测量步 (Measurement) ---
        
        // 计算观测矩阵 H (h(x) 对 q 的雅可比矩阵)
        double q0 = xhatMinus[0], q1 = xhatMinus[1], q2 = xhatMinus[2], q3 = xhatMinus[3];
        H.setZero();
        H(0, 0) = -2*q2;  H(0, 1) =  2*q3;  H(0, 2) = -2*q0;  H(0, 3) =  2*q1;
        H(1, 0) =  2*q1;  H(1, 1) =  2*q0;  H(1, 2) =  2*q3;  H(1, 3) =  2*q2;
        H(2, 0) =  2*q0;  H(2, 1) = -2*q1;  H(2, 2) = -2*q2;  H(2, 3) =  2*q3;

        // 计算预测测量值 h(x-)
        Eigen::Vector3d h_x;
        h_x[0] = 2 * (q1*q3 - q0*q2);
        h_x[1] = 2 * (q0*q1 + q2*q3);
        h_x[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // 残差
        Eigen::Vector3d y = measuredVector - h_x;

        // 计算 S 矩阵及其逆
        Eigen::Matrix3d S = H * Pminus * H.transpose() + R;
        Eigen::Matrix3d S_inv = S.inverse();

        // --- 3. 统计检测与自适应增益 ---
        double chiSquare = y.transpose() * S_inv * y;
        
        if (chiSquare < 0.5 * ChiSquareTestThreshold) ConvergeFlag = true;

        double adaptiveGain = 1.0;
        if (chiSquare > ChiSquareTestThreshold && ConvergeFlag) {
            if (++ErrorCount > 50) {
                ConvergeFlag = false; // 发生发散，强制复位
            } else {
                return; // 拒绝本次更新 (Outlier Rejection)
            }
        } else {
            ErrorCount = 0;
            if (chiSquare > 0.1 * ChiSquareTestThreshold && ConvergeFlag) {
                adaptiveGain = (ChiSquareTestThreshold - chiSquare) / (0.9 * ChiSquareTestThreshold);
            }
        }

        // --- 4. 修正步 (Correction) ---
        K = Pminus * H.transpose() * S_inv;
        K *= adaptiveGain; // 应用自适应增益

        // 计算修正量并限制 Bias 的修正幅度
        Eigen::VectorXd correction = K * y;
        correction[3] = 0; // 强制不对 Yaw 轴四元数分量进行加速度计补偿
        
        xhat = xhatMinus + correction;
        xhat.segment<4>(0).normalize();

        // 更新协方差
        P = (Eigen::MatrixXd::Identity(6, 6) - K * H) * Pminus;

        // --- 5. 欧拉角转换 ---
        UpdateEulerAngles();
        UpdateCount++;
    }

private:

    void UpdateEulerAngles() 
    {
        q = Eigen::Quaternionf(xhat[0], xhat[1], xhat[2], xhat[3]);

        float q0 = xhat[0], q1 = xhat[1], q2 = xhat[2], q3 = xhat[3];
        yaw   = std::atan2(2.0 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3);
        pitch = -std::asin(2.0 * (q1*q3 - q0*q2));
        roll  = std::atan2(2.0 * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3);

        // Yaw 连续化处理
        if (yaw - prev_yaw > M_PI)       
            yaw_round_count--;
        else if (yaw - prev_yaw < -M_PI) 
            yaw_round_count++;
        
        total_yaw = M_PI*2*yaw_round_count + yaw;
        prev_yaw = yaw;
    }
};
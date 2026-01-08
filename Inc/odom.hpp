#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "kalmanfilter.hpp"

#define t  0.001f
#define t2 0.000001f
#define t3 0.000000001f
#define t4 0.000000000001f
#define t5 0.000000000000001f

struct odometry_info_t
{
    float x;
    float v;
    float a_z;
};

class cVelFusionKF
{
protected:
    const float qq = 5.0f;
    const float rv = 0.1f;
    const float ra = 50.0f;

    DynamicKalmanFilter kf;

public:
    cVelFusionKF() : kf(3, 0, 2)  // 3 states, 0 control inputs, 2 measurements
    {
        // 状态转移矩阵 F
        kf.F << 1, t, t2 / 2,
                0, 1, t,
                0, 0, 1;

        // 过程噪声协方差 Q
        kf.Q << t5 / 20 * qq, t4 / 8 * qq, t3 / 6 * qq,
                t4 / 8 * qq, t3 / 3 * qq, t2 / 2 * qq,
                t3 / 6 * qq, t2 / 2 * qq, t * qq;

        // 测量矩阵 H (2x3)
        kf.H << 0, 1, 0,   // 测量速度
                0, 0, 1;   // 测量加速度

        // 测量噪声协方差 R
        kf.R << rv, 0,
                0, ra;

        // 初始状态协方差 P
        kf.P << 10, 0, 0,
                0, 10, 0,
                0, 0, 10;

        // 初始化状态向量
        kf.xhat.setZero();
        
        // 关闭自动调整,因为我们有固定的测量维度
        kf.useAutoAdjustment = false;
    }

    void ResetKF()
    {
        kf.xhat.setZero();
        kf.xhatMinus.setZero();
        kf.P.setIdentity();
        kf.P *= 10;
    }

    void UpdateKalman(float velocity, float accelerationX)
    {
        // 设置测量向量
        kf.measuredVector[0] = velocity;
        kf.measuredVector[1] = accelerationX;
        
        // 执行卡尔曼滤波更新
        kf.Update();
    }

    float GetXhat()
    {
        return kf.xhat(0);
    }

    float GetVhat()
    {
        return kf.xhat(1);
    }

    float GetAhat()
    {
        return kf.xhat(2);
    }
};

class Odometry
{
private:
    odometry_info_t odom_data_;
    cVelFusionKF vel_fusion_kf_;

public:
   
    /**
     * @brief odometry update function
     * @note all the params must be homography
     * @param _quaternion [w, x, y, z] format
     * @param _acc [0, ax, ay, az] quaternion format for acceleration
     * @param _vel velocity measurement
     * @param _yaw in degree
     * @return odometry_info
     */
    void Odometry_Update(Eigen::Quaternionf q, Eigen::Vector3f acc, float _vel, float _yaw)
    {
        // 将加速度从机体坐标系转换到世界坐标系
        Eigen::Vector3f a_world = q*acc;

        // 计算 x 方向的加速度分量
        float yaw_rad = _yaw * M_PI / 180.0f;
        float a_x = std::sqrt(a_world.x() * a_world.x() + a_world.y() * a_world.y()) *
                    std::cos(std::atan2(a_world.y(), a_world.x()) - yaw_rad);

        // 更新卡尔曼滤波器
        vel_fusion_kf_.UpdateKalman(_vel, a_x);

        // 获取滤波后的结果
        odom_data_.v = vel_fusion_kf_.GetVhat();
        odom_data_.x = vel_fusion_kf_.GetXhat();
        odom_data_.a_z = a_world.z();
    }

    // 重置里程计
    void Reset()
    {
        vel_fusion_kf_.ResetKF();
        odom_data_ = {0.0f, 0.0f, 0.0f};
    }

    // 获取当前里程计数据
    odometry_info_t GetOdomData() const
    {
        return odom_data_;
    }
};
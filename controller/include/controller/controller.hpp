#pragma once

// constexpr float PI = 3.14159265358979323846f;
constexpr float RAD2DEG = 57.29577951308232f;
constexpr float DEG2RAD = 0.017453292519943295f;
constexpr float WHEEL_RADIUS = 0.076f;  // 轮子半径，单位：米

struct motor_fdb_t
{
    float pos_fdb;
    float spd_fdb;
    float tor_fdb;
};

struct IMU
{
    float qx;
    float qy;
    float qz;
    float qw;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

float FloatConstrain(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}


void QuaternionToEuler(Eigen::Quaternionf q, Eigen::Vector3f& euler)
{
    float roll, pitch, yaw;
    float qw = q.w();
    float qx = q.x();
    float qy = q.y();
    float qz = q.z();

    // 归一化
    float norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    qw /= norm; qx /= norm; qy /= norm; qz /= norm;

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90° if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    euler << roll, pitch, yaw;
}

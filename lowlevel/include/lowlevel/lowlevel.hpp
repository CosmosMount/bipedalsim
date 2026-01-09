#ifndef LOWLEVEL_HPP
#define LOWLEVEL_HPP

#include <rclcpp/rclcpp.hpp>

struct MOTOR
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

#endif // LOWLEVEL_HPP

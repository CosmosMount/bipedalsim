#pragma once

#include <Eigen/Dense>
#include <vector>
#include <functional>
#include <algorithm>

/**
 * @brief 动态卡尔曼滤波器，支持自动调整H、R、K维度
 */
class DynamicKalmanFilter
{
public:
    int xhatSize;
    int uSize;
    int zSize;

    Eigen::VectorXd xhat;        // 状态 x(k|k)
    Eigen::VectorXd xhatMinus;   // 预测状态 x(k|k-1)
    Eigen::VectorXd z;           // 测量向量
    Eigen::VectorXd u;           // 控制向量
    Eigen::MatrixXd P;           // 状态协方差 P(k|k)
    Eigen::MatrixXd Pminus;      // 预测协方差 P(k|k-1)
    Eigen::MatrixXd F;           // 状态转移矩阵
    Eigen::MatrixXd B;           // 控制矩阵
    Eigen::MatrixXd H;           // 测量矩阵
    Eigen::MatrixXd R;           // 测量噪声
    Eigen::MatrixXd Q;           // 过程噪声
    Eigen::MatrixXd K;           // 卡尔曼增益

    Eigen::VectorXd measuredVector;
    Eigen::VectorXd controlVector;
    Eigen::VectorXd stateMinVariance;

    bool useAutoAdjustment = true;

    // 用户自定义回调函数，可替代 KF 五个环节
    std::function<void(DynamicKalmanFilter &)> User_Func[7];

public:
    DynamicKalmanFilter(int xSize, int uSize_ = 0, int zSize_ = 0)
        : xhatSize(xSize), uSize(uSize_), zSize(zSize_)
    {
        xhat = Eigen::VectorXd::Zero(xhatSize);
        xhatMinus = Eigen::VectorXd::Zero(xhatSize);
        P = Eigen::MatrixXd::Identity(xhatSize, xhatSize);
        Pminus = Eigen::MatrixXd::Identity(xhatSize, xhatSize);
        F = Eigen::MatrixXd::Identity(xhatSize, xhatSize);
        Q = Eigen::MatrixXd::Identity(xhatSize, xhatSize) * 0.01;
        K = Eigen::MatrixXd::Zero(xhatSize, zSize);

        if (zSize > 0)
        {
            H = Eigen::MatrixXd::Zero(zSize, xhatSize);
            R = Eigen::MatrixXd::Identity(zSize, zSize) * 0.1;
            z = Eigen::VectorXd::Zero(zSize);
            measuredVector = Eigen::VectorXd::Zero(zSize);
        }

        if (uSize > 0)
        {
            B = Eigen::MatrixXd::Zero(xhatSize, uSize);
            u = Eigen::VectorXd::Zero(uSize);
            controlVector = Eigen::VectorXd::Zero(uSize);
        }

        stateMinVariance = Eigen::VectorXd::Zero(xhatSize);
    }

    // 更新测量向量和控制向量
    void Measure()
    {
        if (useAutoAdjustment)
        {
            Adjust_H_R_K();
        }
        else
        {
            z = measuredVector;
            measuredVector.setZero();
        }

        if (uSize > 0)
            u = controlVector;
    }

    // 执行卡尔曼滤波
    void Update()
    {
        Measure();

        // 1. xhatMinus = F*xhat + B*u
        if (uSize > 0)
            xhatMinus = F * xhat + B * u;
        else
            xhatMinus = F * xhat;

        // 2. Pminus = F*P*F^T + Q
        Pminus = F * P * F.transpose() + Q;

        bool shouldUpdate = false;
        if (useAutoAdjustment) {
            shouldUpdate = (z.size() > 0);
        } else {
            shouldUpdate = (zSize > 0);
        }

        if (shouldUpdate)
        {
            // 3. K = Pminus * H^T * (H*Pminus*H^T + R)^-1
            Eigen::MatrixXd S = H * Pminus * H.transpose() + R;
            K = Pminus * H.transpose() * S.inverse();
            if (User_Func[3]) User_Func[3](*this);

            // 4. xhat = xhatMinus + K*(z - H*xhatMinus)
            xhat = xhatMinus + K * (z - H * xhatMinus);
            if (User_Func[4]) User_Func[4](*this);

            // 5. P = Pminus - K*H*Pminus
            P = Pminus - K * H * Pminus;
        }
        else
        {
            xhat = xhatMinus;
            P = Pminus;
        }


        // 限制最小方差
        for (int i = 0; i < xhatSize; ++i)
            if (P(i, i) < stateMinVariance[i])
                P(i, i) = stateMinVariance[i];

    }

private:
    void Adjust_H_R_K()
    {
        // 根据 measuredVector 动态调整 H, R, K
        int validCount = 0;
        std::vector<int> idxMap;
        for (int i = 0; i < zSize; ++i)
        {
            if (measuredVector[i] != 0)
            {
                z[validCount] = measuredVector[i];
                idxMap.push_back(i);
                ++validCount;
            }
        }
        z.conservativeResize(validCount);
        H.conservativeResize(validCount, xhatSize);
        R.conservativeResize(validCount, validCount);
        K.conservativeResize(xhatSize, validCount);

        // 清零矩阵
        H.setZero();
        R.setZero();
        K.setZero();
    }
};
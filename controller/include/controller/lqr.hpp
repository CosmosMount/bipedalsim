#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <array>

/**
 * @brief LQR 控制器配置
 */
struct LQRConfig
{
    static constexpr int STATE_DIM = 6;      // 状态维度
    static constexpr int CONTROL_DIM = 2;    // 控制维度
    static constexpr int K_NUM = 46;         // K矩阵数量
    // static constexpr int K_NUM = 23;         // K矩阵数量
    static constexpr float MIN_LEN = 0.12f;  // 最小腿长
    static constexpr float MAX_LEN = 0.32f;  // 最大腿长
    static constexpr float LEN_RESOLUTION = 0.01f; // 腿长分辨率
};

/**
 * @brief 现代化 LQR 控制器
 * @tparam T 数据类型 (float 或 double)
 */
template<typename T = float>
class LQRController
{
public:
    using StateVector = Eigen::Matrix<T, LQRConfig::STATE_DIM, 1>;
    using ControlVector = Eigen::Matrix<T, LQRConfig::CONTROL_DIM, 1>;
    using GainMatrix = Eigen::Matrix<T, LQRConfig::CONTROL_DIM, LQRConfig::STATE_DIM>;

private:
    // LQR 增益矩阵查找表 (预计算的 -K 矩阵)
    // 格式: [Normal, OffGround] 成对,对应不同腿长
    static constexpr std::array<std::array<T, 12>, LQRConfig::K_NUM> K_TABLE = {{
        /* Normal -K    L=0.120000      R00=6.00        R11=3.00 */
        {83.57012, 23.866755, 0.845941, 5.015903, 224.624844, 31.066638, -60.385721, -20.786651, -0.744650, -4.404229, 452.109940, 60.441487},
        /* OffGround -K L=0.120000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -60.385721, -20.786651, 0, 0, 0, 0},
        /* Normal -K    L=0.130000      R00=6.00        R11=3.00 */
        {90.29306, 25.946770, 0.918174, 5.342502, 217.073627, 30.047880, -62.680623, -21.543376, -0.770477, -4.472538, 459.001278, 61.300215},
        /* OffGround -K L=0.130000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -62.680623, -21.543376, 0, 0, 0, 0},
        /* Normal -K    L=0.140000      R00=6.00        R11=3.00 */
        {96.45219, 27.849661, 0.983854, 5.629623, 209.716223, 29.055106, -64.271412, -22.061509, -0.787723, -4.497228, 465.401213, 62.102714},
        /* OffGround -K L=0.140000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -64.271412, -22.061509, 0, 0, 0, 0},
        /* Normal -K    L=0.150000      R00=6.00        R11=3.00 */
        {102.07986, 29.586009, 1.043390, 5.881562, 202.569583, 28.090606, -65.266965, -22.379026, -0.797779, -4.487311, 471.332055, 62.850322},
        /* OffGround -K L=0.150000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -65.266965, -22.379026, 0, 0, 0, 0},
        /* Normal -K    L=0.160000      R00=6.00        R11=3.00 */
        {107.21705, 31.169266, 1.097289, 6.102600, 195.645776, 27.156050, -65.768670, -22.530950, -0.801921, -4.450498, 476.818901, 63.545039},
        /* OffGround -K L=0.160000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -65.768670, -22.530950, 0, 0, 0, 0},
        /* Normal -K    L=0.170000      R00=6.00        R11=3.00 */
        {111.90743, 32.613721, 1.146083, 6.296720, 188.952601, 26.252532, -65.866418, -22.548106, -0.801271, -4.393170, 481.888456, 64.189333},
        /* OffGround -K L=0.170000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -65.866418, -22.548106, 0, 0, 0, 0},
        /* Normal -K    L=0.180000      R00=6.00        R11=3.00 */
        {116.19390, 33.933352, 1.190293, 6.467491, 182.494265, 25.380653, -65.637550, -22.456851, -0.796783, -4.320499, 486.568058, 64.785967},
        /* OffGround -K L=0.180000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -65.637550, -22.456851, 0, 0, 0, 0},
        /* Normal -K    L=0.190000      R00=6.00        R11=3.00 */
        {120.11680, 35.141239, 1.230402, 6.618041, 176.272021, 24.540602, -65.147361, -22.279315, -0.789257, -4.236622, 490.884921, 65.337861},
        /* OffGround -K L=0.190000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -65.147361, -22.279315, 0, 0, 0, 0},
        /* Normal -K    L=0.200000      R00=6.00        R11=3.00 */
        {123.71311, 36.249317, 1.266852, 6.751074, 170.284754, 23.732237, -64.450309, -22.033861, -0.779356, -4.144812, 494.865590, 65.847982},
        /* OffGround -K L=0.200000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -64.450309, -22.033861, 0, 0, 0, 0},
        /* Normal -K    L=0.210000      R00=6.00        R11=3.00 */
        {127.01624, 37.268304, 1.300038, 6.868916, 164.529474, 22.955151, -63.591441, -21.735610, -0.767624, -4.047647, 498.535563, 66.319262},
        /* OffGround -K L=0.210000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -63.591441, -21.735610, 0, 0, 0, 0},
        /* Normal -K    L=0.220000      R00=6.00        R11=3.00 */
        {130.05608, 38.207742, 1.330311, 6.973556, 159.001738, 22.208737, -62.607821, -21.396947, -0.754504, -3.947138, 501.919049, 66.754551},
        /* OffGround -K L=0.220000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -62.607821, -21.396947, 0, 0, 0, 0},
        /* Normal -K    L=0.230000      R00=6.00        R11=3.00 */
        {132.85923, 39.076081, 1.357982, 7.066697, 153.696004, 21.492237, -61.529818, -21.027975, -0.740359, -3.844854, 505.038833, 67.156572},
        /* OffGround -K L=0.230000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -61.529818, -21.027975, 0, 0, 0, 0},
        /* Normal -K    L=0.240000      R00=6.00        R11=3.00 */
        {135.44934, 39.880786, 1.383325, 7.149797, 148.605919, 20.804778, -60.382244, -20.636914, -0.725482, -3.742005, 507.916202, 67.527905},
        /* OffGround -K L=0.240000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -60.382244, -20.636914, 0, 0, 0, 0},
        /* Normal -K    L=0.250000      R00=6.00        R11=3.00 */
        {137.84736, 40.628440, 1.406583, 7.224106, 143.724557, 20.145415, -59.185318, -20.230433, -0.710109, -3.639521, 510.570944, 67.870970},
        /* OffGround -K L=0.250000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -59.185318, -20.230433, 0, 0, 0, 0},
        /* Normal -K    L=0.260000      R00=6.00        R11=3.00 */
        {140.07188, 41.324854, 1.427968, 7.290697, 139.044609, 19.513148, -57.955471, -19.813931, -0.694433, -3.538111, 513.021368, 68.188026},
        /* OffGround -K L=0.260000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -57.955471, -19.813931, 0, 0, 0, 0},
        /* Normal -K    L=0.270000      R00=6.00        R11=3.00 */
        {142.13938, 41.975158, 1.447669, 7.350494, 134.558541, 18.906950, -56.706022, -19.391762, -0.678606, -3.438308, 515.284359, 68.481169},
        /* OffGround -K L=0.270000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -56.706022, -19.391762, 0, 0, 0, 0},
        /* Normal -K    L=0.280000      R00=6.00        R11=3.00 */
        {144.06453, 42.583888, 1.465852, 7.404296, 130.258715, 18.325781, -55.447729, -18.967429, -0.662751, -3.340507, 517.375447, 68.752338},
        /* OffGround -K L=0.280000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -55.447729, -18.967429, 0, 0, 0, 0},
        /* Normal -K    L=0.290000      R00=6.00        R11=3.00 */
        {145.86037, 43.155060, 1.482665, 7.452794, 126.137492, 17.768604, -54.189249, -18.543734, -0.646966, -3.244994, 519.308887, 69.003321},
        /* OffGround -K L=0.290000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -54.189249, -18.543734, 0, 0, 0, 0},
        /* Normal -K    L=0.300000      R00=6.00        R11=3.00 */
        {147.53852, 43.692236, 1.498237, 7.496588, 122.187299, 17.234391, -52.937510, -18.122910, -0.631330, -3.151969, 521.097740, 69.235761},
        /* OffGround -K L=0.300000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -52.937510, -18.122910, 0, 0, 0, 0},
        /* Normal -K    L=0.310000      R00=6.00        R11=3.00 */
        {149.10936, 44.198582, 1.512685, 7.536201, 118.400698, 16.722135, -51.698022, -17.706722, -0.615901, -3.061565, 522.753958, 69.451167},
        /* OffGround -K L=0.310000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -51.698022, -17.706722, 0, 0, 0, 0},
        /* Normal -K    L=0.320000      R00=6.00        R11=3.00 */
        {150.58217, 44.676918, 1.526112, 7.572090, 114.770424, 16.230854, -50.475128, -17.296549, -0.600728, -2.973863, 524.288471, 69.650923},
        /* OffGround -K L=0.320000      R00=6.00        R11=3.00 */
        {0, 0, 0, 0, 0, 0, -50.475128, -17.296549, 0, 0, 0, 0}
    }};

    GainMatrix K_;              // 当前使用的增益矩阵
    StateVector x_ref_;         // 参考状态
    StateVector x_obs_;         // 观测状态
    ControlVector u_;           // 控制输出
    T current_leg_length_;      // 当前腿长
    bool is_flying_;            // 是否离地

public:
    LQRController()
        : current_leg_length_(LQRConfig::MIN_LEN)
        , is_flying_(false)
    {
        K_.setZero();
        x_ref_.setZero();
        x_obs_.setZero();
        u_.setZero();
        UpdateGainMatrix(current_leg_length_, is_flying_);
    }

    /**
     * @brief 设置参考状态
     * @param x_ref 参考状态向量 [6x1]
     */
    void SetReferenceState(const StateVector& x_ref)
    {
        x_ref_ = x_ref;
    }

    /**
     * @brief 设置观测状态
     * @param x_obs 观测状态向量 [6x1]
     */
    void SetObservedState(const StateVector& x_obs)
    {
        x_obs_ = x_obs;
    }

    /**
     * @brief 兼容指针接口 - 设置参考状态
     */
    void SetReferenceState(const T* x_ref)
    {
        x_ref_ = Eigen::Map<const StateVector>(x_ref);
    }

    /**
     * @brief 兼容指针接口 - 设置观测状态
     */
    void SetObservedState(const T* x_obs)
    {
        x_obs_ = Eigen::Map<const StateVector>(x_obs);
    }

    /**
     * @brief 更新增益矩阵
     * @param leg_length 当前腿长
     * @param is_flying 是否离地
     */
    void UpdateGainMatrix(T leg_length, bool is_flying)
    {
        // 限制腿长范围
        leg_length = std::clamp(leg_length, 
                                LQRConfig::MIN_LEN, 
                                LQRConfig::MAX_LEN);
        
        current_leg_length_ = leg_length;
        is_flying_ = is_flying;

        // 计算查找表索引
        int index = static_cast<int>(std::round(
            (leg_length - LQRConfig::MIN_LEN) / LQRConfig::LEN_RESOLUTION
        ));
        
        // 选择对应的 K 矩阵 (Normal 或 OffGround)
        int k_index = 2 * index + (is_flying ? 1 : 0);
        // int k_index = index;
        
        // 加载 K 矩阵 (注意: K_TABLE 存储的是展平的行优先矩阵)
        const auto& k_data = K_TABLE[k_index];
        K_ = Eigen::Map<const Eigen::Matrix<T, 2, 6, Eigen::RowMajor>>(k_data.data());
    }

    /**
     * @brief 计算 LQR 控制输出
     * @return 控制向量 [T, Tp]
     * 
     * 计算公式: u = -K * (x_obs - x_ref)
     */
    ControlVector Update(StateVector x_obs, StateVector x_ref)
    {
        StateVector error = x_obs - x_ref;
        u_ = K_ * error;  // K 已经是负的,所以直接相乘
        return u_;
    }

    /**
     * @brief 计算并返回到指针
     * @param u_out 输出控制量 [T, Tp]
     */
    void Compute(T* u_out)
    {
        ControlVector u = Compute();
        u_out[0] = u(0);
        u_out[1] = u(1);
    }

    /**
     * @brief 一步完成:更新矩阵+计算控制
     * @param leg_length 腿长
     * @param is_flying 是否离地
     * @return 控制输出
     */
    ControlVector ComputeWithUpdate(T leg_length, bool is_flying)
    {
        UpdateGainMatrix(leg_length, is_flying);
        return Compute();
    }

    // Getter 方法
    const GainMatrix& GetGainMatrix() const { return K_; }
    const StateVector& GetReferenceState() const { return x_ref_; }
    const StateVector& GetObservedState() const { return x_obs_; }
    const ControlVector& GetControlOutput() const { return u_; }
    T GetCurrentLegLength() const { return current_leg_length_; }
    bool IsFlying() const { return is_flying_; }

    /**
     * @brief 重置控制器
     */
    void Reset()
    {
        x_ref_.setZero();
        x_obs_.setZero();
        u_.setZero();
    }
};

// 类型别名
using LQRControllerf = LQRController<float>;
using LQRControllerd = LQRController<double>;
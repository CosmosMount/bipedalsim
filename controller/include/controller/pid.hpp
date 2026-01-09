#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

class PDController
{
    protected:
        float _kp;
        float _kd;
        float _dt;
        float _max;
        float _min;
        float _ref;
        float _x_fdb;
        float _x_dot_fdb;
        float _out;
    public:
        void SetParam(float kp, float kd, float max, float min) {
            _kp = kp;
            _kd = kd;
            _max = max;
            _min = min;
        }

        void SetRef(float ref) {
            _ref = ref;
        }

        float Out() {
            return _out;
        }

        float Update(float x, float x_dot) {
            _x_fdb = x;
            _x_dot_fdb = x_dot;
            float out_val;
            out_val = _kp * (_ref - _x_fdb) + _x_dot_fdb * _kd ;
            if (out_val > _max) {
                out_val = _max;
            } else if (out_val < _min) {
                out_val = _min;
            }
            _out = out_val;
            return out_val;
        }

        float Clear() {
            _x_fdb = 0.0f;
            _x_dot_fdb = 0.0f;
            _ref = 0.0f;
            _out = 0.0f;
            return _out;
        }

        float GetKp() {
            return _kp;
        }

        float GetKd() {
            return _kd;
        }
};

/**
 * @brief 简化版 PID 控制器类
 * @tparam T 数据类型 (float 或 double)
 */
template<typename T = float>
class PIDController
{
private:
    // PID 参数
    T kp_;                    // 比例增益
    T ki_;                    // 积分增益
    T kd_;                    // 微分增益
    T max_output_;            // 输出上限
    T min_output_;            // 输出下限
    T max_integral_;          // 积分上限
    T min_integral_;          // 积分下限
    T deadzone_;              // 死区
    T dt_;                    // 采样时间(秒)
    
    // 状态变量
    T setpoint_;              // 目标值
    T error_;                 // 当前误差
    T last_error_;            // 上次误差
    T integral_;              // 积分累积
    T derivative_;            // 微分项
    T output_;                // 输出值
    bool first_run_;          // 首次运行标志

public:
    /**
     * @brief 构造函数
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     * @param dt 采样时间(默认 0.001s = 1ms)
     */
    explicit PIDController(T kp = 0.0, T ki = 0.0, T kd = 0.0, T dt = 0.001)
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , max_output_(std::numeric_limits<T>::max())
        , min_output_(std::numeric_limits<T>::lowest())
        , max_integral_(std::numeric_limits<T>::max())
        , min_integral_(std::numeric_limits<T>::lowest())
        , deadzone_(0.0)
        , dt_(dt)
        , setpoint_(0.0)
        , error_(0.0)
        , last_error_(0.0)
        , integral_(0.0)
        , derivative_(0.0)
        , output_(0.0)
        , first_run_(true)
    {
    }

    /**
     * @brief 设置 PID 增益
     */
    void SetGains(T kp, T ki, T kd, T max_out, T min_out) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        min_output_ = min_out;
        max_output_ = max_out;
    }

    /**
     * @brief 设置单个增益
     */
    void SetKp(T kp) { kp_ = kp; }
    void SetKi(T ki) { ki_ = ki; }
    void SetKd(T kd) { kd_ = kd; }

    /**
     * @brief 设置目标值
     */
    void SetRef(T setpoint) { setpoint_ = setpoint; }

    /**
     * @brief 设置采样时间
     */
    void SetDt(T dt) { dt_ = dt; }

    /**
     * @brief 设置输出限幅
     */
    void SetOutputLimits(T min_output, T max_output) {
        min_output_ = min_output;
        max_output_ = max_output;
    }

    /**
     * @brief 设置积分限幅
     */
    void SetIntegralLimits(T min_integral, T max_integral) {
        min_integral_ = min_integral;
        max_integral_ = max_integral;
    }

    /**
     * @brief 设置死区
     */
    void SetDeadzone(T deadzone) { deadzone_ = deadzone; }

    /**
     * @brief 更新 PID 控制器
     * @param measurement 当前测量值
     * @return 控制输出
     */
    T Update(T measurement)
    {
        // 计算误差
        error_ = setpoint_ - measurement;

        // 死区处理
        if (std::abs(error_) < deadzone_) {
            error_ = 0.0;
        }

        // 比例项
        T p_term = kp_ * error_;

        // 积分项
        integral_ += error_ * dt_;
        integral_ = Clamp(integral_, min_integral_, max_integral_);
        T i_term = ki_ * integral_;

        // 微分项
        if (first_run_) {
            derivative_ = 0.0;
            first_run_ = false;
        } else {
            derivative_ = (error_ - last_error_) / dt_;
        }
        T d_term = kd_ * derivative_;

        // 计算总输出
        output_ = p_term + i_term + d_term;
        output_ = Clamp(output_, min_output_, max_output_);

        // 保存当前误差
        last_error_ = error_;

        return output_;
    }

    /**
     * @brief 带前馈的更新
     * @param measurement 当前测量值
     * @param feedforward 前馈项
     * @return 控制输出
     */
    T Update(T measurement, T feedforward)
    {
        T pid_output = Update(measurement);
        T total_output = pid_output + feedforward;
        return Clamp(total_output, min_output_, max_output_);
    }

    /**
     * @brief 重置 PID 状态
     */
    void Reset()
    {
        error_ = 0.0;
        last_error_ = 0.0;
        integral_ = 0.0;
        derivative_ = 0.0;
        output_ = 0.0;
        first_run_ = true;
    }

    // Getter 方法
    T GetError() const { return error_; }
    T GetIntegral() const { return integral_; }
    T GetDerivative() const { return derivative_; }
    T GetOutput() const { return output_; }
    T GetSetpoint() const { return setpoint_; }
    T GetKp() const { return kp_; }
    T GetKi() const { return ki_; }
    T GetKd() const { return kd_; }
    T GetDt() const { return dt_; }

private:
    /**
     * @brief 限幅函数
     */
    T Clamp(T value, T min_val, T max_val) const
    {
        return std::max(min_val, std::min(value, max_val));
    }
};


/**
 * @brief 级联 PID 控制器(双环控制)
 * @tparam T 数据类型
 */
template<typename T = float>
class CascadePIDController
{
private:
    PIDController<T> outer_pid_;  // 外环(位置环)
    PIDController<T> inner_pid_;  // 内环(速度环)

public:
    /**
     * @brief 构造函数
     * @param outer_kp 外环比例增益
     * @param outer_ki 外环积分增益
     * @param outer_kd 外环微分增益
     * @param inner_kp 内环比例增益
     * @param inner_ki 内环积分增益
     * @param inner_kd 内环微分增益
     * @param dt 采样时间
     */
    CascadePIDController(T outer_kp = 0.0, T outer_ki = 0.0, T outer_kd = 0.0,
                        T inner_kp = 0.0, T inner_ki = 0.0, T inner_kd = 0.0,
                        T dt = 0.001)
        : outer_pid_(outer_kp, outer_ki, outer_kd, dt)
        , inner_pid_(inner_kp, inner_ki, inner_kd, dt)
    {
    }

    /**
     * @brief 更新级联 PID
     * @param position_setpoint 位置目标
     * @param position_measurement 位置测量
     * @param velocity_measurement 速度测量
     * @return 控制输出
     */
    T Update(T position_setpoint, T position_measurement, T velocity_measurement)
    {
        // 外环输出作为内环目标
        outer_pid_.SetSetpoint(position_setpoint);
        T velocity_setpoint = outer_pid_.Update(position_measurement);

        // 内环控制
        inner_pid_.SetSetpoint(velocity_setpoint);
        return inner_pid_.Update(velocity_measurement);
    }

    void Reset()
    {
        outer_pid_.Reset();
        inner_pid_.Reset();
    }

    PIDController<T>& GetOuterPID() { return outer_pid_; }
    PIDController<T>& GetInnerPID() { return inner_pid_; }
};


/**
 * @brief 向量 PID 控制器(用于多维控制)
 */
template<int N, typename T = float>
class VectorPIDController
{
private:
    using VectorNT = Eigen::Matrix<T, N, 1>;
    
    T kp_, ki_, kd_;
    T max_output_, min_output_;
    T max_integral_, min_integral_;
    T deadzone_;
    T dt_;
    
    VectorNT setpoint_;
    VectorNT error_;
    VectorNT last_error_;
    VectorNT integral_;
    VectorNT derivative_;
    VectorNT output_;
    bool first_run_;

public:
    explicit VectorPIDController(T kp = 0.0, T ki = 0.0, T kd = 0.0, T dt = 0.001)
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , max_output_(std::numeric_limits<T>::max())
        , min_output_(std::numeric_limits<T>::lowest())
        , max_integral_(std::numeric_limits<T>::max())
        , min_integral_(std::numeric_limits<T>::lowest())
        , deadzone_(0.0)
        , dt_(dt)
        , first_run_(true)
    {
        setpoint_.setZero();
        error_.setZero();
        last_error_.setZero();
        integral_.setZero();
        derivative_.setZero();
        output_.setZero();
    }

    void SetGains(T kp, T ki, T kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void SetSetpoint(const VectorNT& setpoint) { setpoint_ = setpoint; }
    
    void SetOutputLimits(T min_output, T max_output) {
        min_output_ = min_output;
        max_output_ = max_output;
    }

    VectorNT Update(const VectorNT& measurement)
    {
        // 计算误差
        error_ = setpoint_ - measurement;

        // 死区处理
        for (int i = 0; i < N; ++i) {
            if (std::abs(error_(i)) < deadzone_) {
                error_(i) = 0.0;
            }
        }

        // 比例项
        VectorNT p_term = kp_ * error_;

        // 积分项
        integral_ += error_ * dt_;
        for (int i = 0; i < N; ++i) {
            integral_(i) = std::clamp(integral_(i), min_integral_, max_integral_);
        }
        VectorNT i_term = ki_ * integral_;

        // 微分项
        if (first_run_) {
            derivative_.setZero();
            first_run_ = false;
        } else {
            derivative_ = (error_ - last_error_) / dt_;
        }
        VectorNT d_term = kd_ * derivative_;

        // 计算输出
        output_ = p_term + i_term + d_term;
        for (int i = 0; i < N; ++i) {
            output_(i) = std::clamp(output_(i), min_output_, max_output_);
        }

        last_error_ = error_;
        return output_;
    }

    void Reset()
    {
        error_.setZero();
        last_error_.setZero();
        integral_.setZero();
        derivative_.setZero();
        output_.setZero();
        first_run_ = true;
    }

    const VectorNT& GetOutput() const { return output_; }
    const VectorNT& GetError() const { return error_; }
};

// 常用类型别名
using PIDControllerf = PIDController<float>;
using PIDControllerd = PIDController<double>;
using CascadePIDControllerf = CascadePIDController<float>;
using CascadePIDControllerd = CascadePIDController<double>;
using VectorPID2f = VectorPIDController<2, float>;
using VectorPID3f = VectorPIDController<3, float>;
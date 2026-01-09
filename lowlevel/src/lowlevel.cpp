#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lowlevel/msg/robot_fdb.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using sensor_msgs::msg::JointState;
using sensor_msgs::msg::Imu;
using namespace message_filters;

class FeedbackReceiver : public rclcpp::Node
{
public:
    FeedbackReceiver() : Node("feedback_receiver")
    {
        // publisher
        publisher_ = this->create_publisher<lowlevel::msg::RobotFdb>("/robot_feedback", 10);

        pub_timer = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&FeedbackReceiver::publishfdb, this));

        // message_filters subscribers
        js_sub_.subscribe(this, "/joint_states");
        imu_sub_.subscribe(this, "/imu/data");

        // 近似时间同步器
        typedef sync_policies::ApproximateTime<JointState, Imu> MySyncPolicy;
        sync_.reset(new Synchronizer<MySyncPolicy>(MySyncPolicy(10), js_sub_, imu_sub_));
        sync_->registerCallback(&FeedbackReceiver::synced_callback, this);
    }

private:
    rclcpp::TimerBase::SharedPtr pub_timer;
    rclcpp::Publisher<lowlevel::msg::RobotFdb>::SharedPtr publisher_;
    message_filters::Subscriber<JointState> js_sub_;
    message_filters::Subscriber<Imu> imu_sub_;
    std::shared_ptr<Synchronizer<sync_policies::ApproximateTime<JointState, Imu>>> sync_;
    lowlevel::msg::RobotFdb message;

    void synced_callback(const JointState::SharedPtr js, const Imu::SharedPtr imu)
    {
        
        // 填充 JointState
        for (size_t i = 0; i < js->name.size(); ++i)
        {
            if (js->name[i] == "Lbig_joint") {
                message.l_big.pos_fdb = js->position[i];
                message.l_big.spd_fdb = js->velocity[i];
                message.l_big.tor_fdb = js->effort[i];
            } else if (js->name[i] == "Lsmall_joint") {
                message.l_small.pos_fdb = js->position[i];
                message.l_small.spd_fdb = js->velocity[i];
                message.l_small.tor_fdb = js->effort[i];
            } else if (js->name[i] == "Lwheel_joint") {
                message.l_wheel.pos_fdb = js->position[i];
                message.l_wheel.spd_fdb = js->velocity[i];
                message.l_wheel.tor_fdb = js->effort[i];
            } else if (js->name[i] == "Rbig_joint") {
                message.r_big.pos_fdb = js->position[i];
                message.r_big.spd_fdb = js->velocity[i];
                message.r_big.tor_fdb = js->effort[i];
            } else if (js->name[i] == "Rsmall_joint") {
                message.r_small.pos_fdb = js->position[i];
                message.r_small.spd_fdb = js->velocity[i];
                message.r_small.tor_fdb = js->effort[i];
            } else if (js->name[i] == "Rwheel_joint") {
                message.r_wheel.pos_fdb = js->position[i];
                message.r_wheel.spd_fdb = js->velocity[i];
                message.r_wheel.tor_fdb = js->effort[i];
            }
        }

        // 填充 IMU
        message.imu.qx = imu->orientation.x;
        message.imu.qy = imu->orientation.y;
        message.imu.qz = imu->orientation.z;
        message.imu.qw = imu->orientation.w;

        message.imu.acc_x = imu->linear_acceleration.x;
        message.imu.acc_y = imu->linear_acceleration.y;
        message.imu.acc_z = imu->linear_acceleration.z;

        message.imu.gyro_x = imu->angular_velocity.x;
        message.imu.gyro_y = imu->angular_velocity.y;
        message.imu.gyro_z = imu->angular_velocity.z;
    }

    void publishfdb()
    {
        publisher_->publish(message);
        // RCLCPP_INFO(this->get_logger(), "Published Robot Feedback");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeedbackReceiver>());
    rclcpp::shutdown();
    return 0;
}

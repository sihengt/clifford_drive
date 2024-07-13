#ifndef CLIFFORD_DRIVE_NODE_H_
#define CLIFFORD_DRIVE_NODE_H_

#include "ros/ros.h"
#include "CppMaestro/CppMaestro.hpp"
#include "clifford_drive/CliffordDriveCommand.h"
#include "clifford_drive/throttle_controller.hpp"
#include "vesc_msgs/VescCommand.h"

#include <cmath>
#include <algorithm>

class CliffordDriveNode
{
public:
    CliffordDriveNode();
    
private:
    ros::NodeHandle nh_;

    // Hardware connections
    std::unique_ptr<CppMaestro> servo_driver_;
    // TODO: VESC motor_driver_; 

    // Subscribers
    ros::Subscriber command_sub_;
    ros::Publisher  vesc_pub_;
    ros::Publisher  status_pub_;

    // Timers
    ros::Timer command_timer_;
    ros::Timer reconnect_timer_;

    // Callbacks    
    void commandCallback(const clifford_drive::CliffordDriveCommand::ConstPtr& msg);
    
    void reconnectCallback(const ros::TimerEvent& event);
    void sendCommand(const ros::TimerEvent& event);

    // Functions
    bool getParameters();
    bool connectSteer();
    bool connectVESC();

    template <typename T>
    T clip(const T& data, const T& lower_limit, const T& upper_limit) 
    { 
        return std::max(lower_limit, std::min(upper_limit, data));
    }

    float limitAcceleration(float accel_cmd, float prev_cmd, const std::vector<float>& limit_cmd);

    // Center position of Maestro server
    static const int MAESTRO_CENTER_POSITION_ = 6000;

    ThrottleController front_throttle_controller_;
    ThrottleController rear_throttle_controller_;
    
    struct MotorParams
    {
        std::string vesc_serial_port_;
        int front_vesc_id_;
        int rear_vesc_id_;
        int comm_vesc_id_;
        std::vector<float> front_throttle_limit_;
        std::vector<float> rear_throttle_limit_;
        std::vector<float> front_accel_limit_;
        std::vector<float> rear_accel_limit_;    
        float front_throttle_scale_;
        float rear_throttle_scale_;
        float e_ratio_;
        float gear_ratio_;
        int num_slots_;
    };
    struct ServoParams
    {
        std::string steer_serial_port_;
        int front_steer_id_;
        int rear_steer_id_;
        int front_steer_scale_;
        int rear_steer_scale_;
        float front_steer_trim_;
        float rear_steer_trim_;
        std::vector<float> front_steer_limit_;
        std::vector<float> rear_steer_limit_;
    };
    struct CliffordCommands
    {
        float prev_front_throttle_ = 0.0f;
        float prev_rear_throttle_ = 0.0f;
        float cmd_front_throttle_ = 0.0f;
        float cmd_rear_throttle_ = 0.0f;
        float cmd_front_steer_ = 0.0f;;
        float cmd_rear_steer_ = 0.0f;
        ros::Time last_cmd_time_;
    };

    MotorParams motor_params_;
    ServoParams servo_params_;
    CliffordCommands clifford_commands_;

    // Status flags
    bool drive_connected_ = false;
    bool steer_connected_ = false;

    // Latest commands


    // Timers
    float command_period_;
    float reconnect_period_;
};

#endif // CLIFFORD_DRIVE_NODE_H_
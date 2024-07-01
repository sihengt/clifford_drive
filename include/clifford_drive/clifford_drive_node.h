#ifndef CLIFFORD_DRIVE_NODE_H_
#define CLIFFORD_DRIVE_NODE_H_

#include "ros/ros.h"
#include "CppMaestro/CppMaestro.hpp"
#include <algorithm>

class CliffordDriveNode
{
public:
    CliffordDriveNode(const std::string& port);
    
private:
    ros::NodeHandle nh_;

    // Hardware connections
    std::unique_ptr<CppMaestro> servo_driver_;
    // TODO: VESC motor_driver_;

    // Subscribers
    ros::Subscriber command_sub_;

    // Callbacks    
    void commandCallback(const CliffordDriveCommand::ConstPtr& msg);
    void sendCommand();

    // Functions
    bool getParameters();
    bool connectSteer();

    template <typename T>
    T clip(const T& data, const T& lower_limit, const T& upper_limit) 
    { 
        return std::max(lower_limit, std::min(upper_limit, data));
    }

    // Center position of Maestro server
    static const int MAESTRO_CENTER_POSITION_ = 6000;

    // Motor variables
    std::string vesc_serial_port_;
    int front_vesc_id_;
    int rear_vesc_id_;
    std::vector<float> front_throttle_limit_;
    std::vector<float> rear_throttle_limit_;
    std::vector<float> front_accel_limit_;
    std::vector<float> rear_accel_limit_;    
    float front_throttle_scale_;
    float rear_throttle_scale_;
    int comm_vesc_id_;
    float e_ratio_;

    // Steering variables
    std::string steer_serial_port_;
    int front_steer_id_;
    int rear_steer_id_;
    float front_steer_scale_;
    float rear_steer_scale_;
    float front_steer_trim_;
    float rear_steer_trim_;
    std::vector<float> front_steer_limit_;
    std::vector<float> rear_steer_limit_;

    // Status flags
    bool drive_connected_ = false;
    bool steer_connected_ = false;

    // Latest commands
    float prev_front_throttle_;
    float prev_rear_throttle_;
    float cmd_front_throttle_;
    float cmd_rear_throttle_;
    float cmd_front_steer_;
    float cmd_rear_steer_;
    ros::Time last_cmd_time_;
    
};

#endif // CLIFFORD_DRIVE_NODE_H_
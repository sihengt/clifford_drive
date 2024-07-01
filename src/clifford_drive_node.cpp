#include "clifford_drive/clifford_drive_node.h"

CliffordDriveNode::CliffordDriveNode(const std::string& port)
    : nh_()
{
    if(!getParameters())
    {
        ROS_ERROR("Failed to load all parameters.");
        ros::shutdown();
    };

    //connectDrive();
    steer_connected_ = connectSteer();
    command_sub_ = nh.subscribe('/motor_controller/command', CliffordDriveCommand, &CliffordDriveNode::commandCallback)
}

bool CliffordDriveNode::getParameters()
{
    bool success = true;

    // lambda for getting parameters and logging failures
    bool getParamWithLogging = [this](const std::string& param_name, auto& param_var)
    {
        if (!nh_.getParam(param_name, param_var))
        {
            ROS_ERROR("Failed to get param: %s", param_name.c_str());
            return false;
        }
        return true;
    };
    
    float gear_ratio;
    int num_slots;
    success &= getParamWithLogging("~gear_ratio", gear_ratio);
    success &= getParamWithLogging("~num_slots", num_slots);
    e_ratio_ = gear_ratio * num_slots;
    
    success &= getParamWithLogging("~comm_vesc_id", comm_vesc_id_);
    success &= getParamWithLogging("~vesc_serial_port", vesc_serial_port_);
    success &= getParamWithLogging("~front_vesc_id", front_vesc_id_);
    success &= getParamWithLogging("~rear_vesc_id", rear_vesc_id_);
    success &= getParamWithLogging("~front_throttle_limit", front_throttle_limit_);
    success &= getParamWithLogging("~rear_throttle_limit", rear_throttle_limit_);
    success &= getParamWithLogging("~front_throttle_scale", front_throttle_scale_);
    success &= getParamWithLogging("~rear_throttle_scale", rear_throttle_scale_);
    success &= getParamWithLogging("~front_accel_limit", front_accel_limit_);
    success &= getParamWithLogging("~rear_accel_limit", rear_accel_limit_);
    success &= getParamWithLogging("~steer_serial_port", steer_serial_port_);
    success &= getParamWithLogging("~front_steer_id", front_steer_id_);
    success &= getParamWithLogging("~rear_steer_id", rear_steer_id_);
    success &= getParamWithLogging("~front_steer_scale", front_steer_scale_);
    success &= getParamWithLogging("~front_steer_trim", front_steer_trim_);
    success &= getParamWithLogging("~rear_steer_trim", rear_steer_trim_);
    success &= getParamWithLogging("~front_steer_limit", front_steer_limit_);
    success &= getParamWithLogging("~rear_steer_limit", rear_steer_limit_);

    return true;
}

bool CliffordDriveNode::connectSteer()
{
    ROS_INFO("Connecting steer servos");
    servo_driver_ = std::make_unique<CppMaestro>(steer_serial_port_);
    if (servo_driver_)
    {
        servo_driver_->setTarget(front_steer_id_, MAESTRO_CENTER_POSITION_);
        servo_driver_->setTarget(rear_steer_id_, MAESTRO_CENTER_POSITION_);
        return true;
    }
    ROS_WARN("Unable to connect to steer servos.");
    return false;
}

void CliffordDriveNode::commandCallback(const CliffordDriveCommand::ConstPtr& msg)
{
    cmd_front_throttle_ = clip(msg->throttle, front_throttle_limit_[0], front_throttle_limit_[1]);
    if (msg->separate_throttle)
        cmd_rear_throttle_ = clip(msg->rear_throttle, rear_throttle_limit_[0], rear_throttle_limit_[1]);
    else
        cmd_rear_throttle_ = cmd_front_throttle_;
    cmd_front_steer_ = clip(msg->front_steering, front_steer_limit[0], front_steer_limit_[1]);
    cmd_rear_steer_ = clip(msg->rear_steering, rear_steer_limit_[0], rear_steer_limit_[1]);
    last_cmd_time_ = rospy.Time.now()
}

// TODO: send command off to motor every ___ time.
// 1. Get ROS to do this every self.control_timeout seconds (this is a parameter from command_config, Sean used 0.25)
// 2. 
void CliffordDriveNode::sendCommand()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clifford_drive_node");
    ros::NodeHandle nh;

    ros::spin();
    return 0;
}
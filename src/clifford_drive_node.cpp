#include "clifford_drive/clifford_drive_node.h"

CliffordDriveNode::CliffordDriveNode()
    : nh_("~")
{
    if(!getParameters())
    {
        ROS_ERROR("Failed to load all parameters.");
        ros::shutdown();
    };

    //connectDrive();
    steer_connected_ = connectSteer();    
    command_sub_ = nh_.subscribe("/motor_controller/command", 1, &CliffordDriveNode::commandCallback, this);
    command_timer_ = nh_.createTimer(ros::Duration(command_period_), &CliffordDriveNode::sendCommand, this);
    reconnect_timer_ = nh_.createTimer(ros::Duration(reconnect_period_), &CliffordDriveNode::reconnectCallback, this);
}

bool CliffordDriveNode::getParameters()
{
    bool success = true;

    // lambda for getting parameters and logging failures
    auto getParamWithLogging = [this](const std::string& param_name, auto& param_var)
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
    success &= getParamWithLogging("gear_ratio", gear_ratio);
    success &= getParamWithLogging("num_slots", num_slots);
    e_ratio_ = gear_ratio * num_slots;
    
    success &= getParamWithLogging("comm_vesc_id", comm_vesc_id_);
    success &= getParamWithLogging("vesc_serial_port", vesc_serial_port_);
    success &= getParamWithLogging("front_vesc_id", front_vesc_id_);
    success &= getParamWithLogging("rear_vesc_id", rear_vesc_id_);
    success &= getParamWithLogging("front_throttle_limit", front_throttle_limit_);
    success &= getParamWithLogging("rear_throttle_limit", rear_throttle_limit_);
    success &= getParamWithLogging("front_throttle_scale", front_throttle_scale_);
    success &= getParamWithLogging("rear_throttle_scale", rear_throttle_scale_);
    success &= getParamWithLogging("front_accel_limit", front_accel_limit_);
    success &= getParamWithLogging("rear_accel_limit", rear_accel_limit_);
    
    // Steering
    success &= getParamWithLogging("steer_serial_port", steer_serial_port_);
    success &= getParamWithLogging("front_steer_id", front_steer_id_);
    success &= getParamWithLogging("front_steer_scale", front_steer_scale_);
    success &= getParamWithLogging("front_steer_trim", front_steer_trim_);
    success &= getParamWithLogging("front_steer_limit", front_steer_limit_);
    success &= getParamWithLogging("rear_steer_id", rear_steer_id_);
    success &= getParamWithLogging("rear_steer_scale", rear_steer_scale_);
    success &= getParamWithLogging("rear_steer_trim", rear_steer_trim_);
    success &= getParamWithLogging("rear_steer_limit", rear_steer_limit_);

    // ROS Timers
    success &= getParamWithLogging("command_period", command_period_);
    success &= getParamWithLogging("reconnect_period", reconnect_period_);

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

bool CliffordDriveNode::connectVESC()
{}

void CliffordDriveNode::commandCallback(const clifford_drive::CliffordDriveCommand::ConstPtr& msg)
{
    // ROS_INFO("[commandCallback] Command received.");
    cmd_front_throttle_ = clip(msg->throttle, front_throttle_limit_[0], front_throttle_limit_[1]);
    if (msg->separate_throttle)
        cmd_rear_throttle_ = clip(msg->rear_throttle, rear_throttle_limit_[0], rear_throttle_limit_[1]);
    else
        cmd_rear_throttle_ = cmd_front_throttle_;
    cmd_front_steer_ = clip(msg->front_steering, front_steer_limit_[0], front_steer_limit_[1]);
    cmd_rear_steer_ = clip(msg->rear_steering, rear_steer_limit_[0], rear_steer_limit_[1]);
    last_cmd_time_ = ros::Time::now();
}

void CliffordDriveNode::reconnectCallback(const ros::TimerEvent& event)
{
    // TODO: link VESC up here
    // if (!drive_connected_)
    // {
    //     drive_connected_ = connectVESC();
    //     ROS_WARN("Reconnecting VESC.");
    // }
    if (!servo_driver_->isConnected())
    {
        ROS_WARN("Reconnecting steering.");
        // steer_connected_ = connectSteer();
    }
}

// TODO: send command off to motor every ___ time.
void CliffordDriveNode::sendCommand(const ros::TimerEvent& event)
{
    if (!steer_connected_)
    {
        prev_front_throttle_ = 0;
        prev_rear_throttle_ = 0;
        return;
    }

    ROS_DEBUG("Front cmd: %f", cmd_front_steer_);
    ROS_DEBUG("Front steer scale: %d", front_steer_scale_);
    ROS_DEBUG("Front steer value: %f", front_steer_scale_ * cmd_front_steer_ + rear_steer_trim_);
    ROS_DEBUG("Setting front servo to %d", static_cast<int>(MAESTRO_CENTER_POSITION_ + front_steer_scale_ * cmd_front_steer_ + front_steer_trim_));

    ROS_DEBUG("Rear cmd: %f", cmd_rear_steer_);
    ROS_DEBUG("Rear steer scale: %d", rear_steer_scale_);
    ROS_DEBUG("Rear steer value: %f", rear_steer_scale_ * cmd_rear_steer_ + rear_steer_trim_);
    ROS_DEBUG("Setting rear servo to %d", static_cast<int>(MAESTRO_CENTER_POSITION_ + rear_steer_scale_ * cmd_rear_steer_ + rear_steer_trim_));

    servo_driver_->setTarget(front_steer_id_, static_cast<int>(MAESTRO_CENTER_POSITION_ + front_steer_scale_ * cmd_front_steer_ + front_steer_trim_));
    servo_driver_->setTarget(rear_steer_id_, static_cast<int>(MAESTRO_CENTER_POSITION_ + rear_steer_scale_ * cmd_rear_steer_ + rear_steer_trim_));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clifford_drive_node");
    CliffordDriveNode clifford_drive;
    ros::spin();
    return 0;
}
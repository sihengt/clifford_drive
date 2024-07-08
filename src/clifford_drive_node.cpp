#include "clifford_drive/clifford_drive_node.hpp"

CliffordDriveNode::CliffordDriveNode()
    : nh_("~"), front_throttle_controller_(), rear_throttle_controller_()
{
    if(!getParameters())
    {
        ROS_ERROR("Failed to load all parameters.");
        ros::shutdown();
    };

    // TODO: clean this up
    if (motor_params_.front_vesc_id_ == motor_params_.comm_vesc_id_)
        motor_params_.front_vesc_id_ = 0;
    if (motor_params_.rear_vesc_id_ == motor_params_.comm_vesc_id_)
        motor_params_.rear_vesc_id_ = 0;

    //connectDrive();
    steer_connected_ = connectSteer();    
    command_sub_ = nh_.subscribe("/motor_controller/command", 1, &CliffordDriveNode::commandCallback, this);
    vesc_pub_ = nh_.advertise<vesc_msgs::VescCommand>("/commands/motor/speed", 1000);
    command_timer_ = nh_.createTimer(ros::Duration(command_period_), &CliffordDriveNode::sendCommand, this);
    reconnect_timer_ = nh_.createTimer(ros::Duration(reconnect_period_), &CliffordDriveNode::reconnectCallback, this);
    front_throttle_controller_.setParams(800.0f, motor_params_.front_throttle_scale_);
    rear_throttle_controller_.setParams(800.0f, motor_params_.rear_throttle_scale_);   
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
    
    success &= getParamWithLogging("gear_ratio", motor_params_.gear_ratio_);
    success &= getParamWithLogging("num_slots", motor_params_.num_slots_);
    motor_params_.e_ratio_ = motor_params_.gear_ratio_ * motor_params_.num_slots_;
    
    success &= getParamWithLogging("comm_vesc_id",          motor_params_.comm_vesc_id_);
    success &= getParamWithLogging("vesc_serial_port",      motor_params_.vesc_serial_port_);
    success &= getParamWithLogging("front_vesc_id",         motor_params_.front_vesc_id_);
    success &= getParamWithLogging("rear_vesc_id",          motor_params_.rear_vesc_id_);
    success &= getParamWithLogging("front_throttle_limit",  motor_params_.front_throttle_limit_);
    success &= getParamWithLogging("rear_throttle_limit",   motor_params_.rear_throttle_limit_);
    success &= getParamWithLogging("front_throttle_scale",  motor_params_.front_throttle_scale_);
    success &= getParamWithLogging("rear_throttle_scale",   motor_params_.rear_throttle_scale_);
    success &= getParamWithLogging("front_accel_limit",     motor_params_.front_accel_limit_);
    success &= getParamWithLogging("rear_accel_limit",      motor_params_.rear_accel_limit_);
    
    // Steering
    success &= getParamWithLogging("steer_serial_port",     servo_params_.steer_serial_port_);
    success &= getParamWithLogging("front_steer_id",        servo_params_.front_steer_id_);
    success &= getParamWithLogging("front_steer_scale",     servo_params_.front_steer_scale_);
    success &= getParamWithLogging("front_steer_trim",      servo_params_.front_steer_trim_);
    success &= getParamWithLogging("front_steer_limit",     servo_params_.front_steer_limit_);
    success &= getParamWithLogging("rear_steer_id",         servo_params_.rear_steer_id_);
    success &= getParamWithLogging("rear_steer_scale",      servo_params_.rear_steer_scale_);
    success &= getParamWithLogging("rear_steer_trim",       servo_params_.rear_steer_trim_);
    success &= getParamWithLogging("rear_steer_limit",      servo_params_.rear_steer_limit_);

    // ROS Timers
    success &= getParamWithLogging("command_period", command_period_);
    success &= getParamWithLogging("reconnect_period", reconnect_period_);

    return true;
}

bool CliffordDriveNode::connectSteer()
{
    ROS_INFO("Connecting steer servos");
    servo_driver_ = std::make_unique<CppMaestro>(servo_params_.steer_serial_port_);
    if (servo_driver_)
    {
        servo_driver_->setTarget(servo_params_.front_steer_id_, MAESTRO_CENTER_POSITION_);
        servo_driver_->setTarget(servo_params_.rear_steer_id_, MAESTRO_CENTER_POSITION_);
        return true;
    }
    ROS_WARN("Unable to connect to steer servos.");
    return false;
}

float CliffordDriveNode::limitAcceleration(float accel_cmd, float prev_cmd, const std::vector<float>& limit_cmd)
{
    // ROS_INFO("limit from %f to %f", limit_cmd[0], limit_cmd[1]);
    ROS_INFO("current_cmd=%f \t previous_cmd=%f", accel_cmd, prev_cmd);

    float maxChange;
    float change = accel_cmd - prev_cmd;
    
    if (std::abs(accel_cmd - prev_cmd) < 1e-6)
    {
        ROS_INFO("Command change within tolerance, returning previous command: %f", prev_cmd);
        return prev_cmd;
    }
    if (std::signbit(accel_cmd) != std::signbit(prev_cmd)) // Direction change
    {
        maxChange = std::min(limit_cmd[1], std::abs(prev_cmd) + (1 - std::abs(prev_cmd) / limit_cmd[1]) * limit_cmd[0]);
    }
    else if (std::abs(accel_cmd) > std::abs(prev_cmd)) // Accelerating
    {
        maxChange = limit_cmd[0];
    }
    else // Decelerating or maintaining speed
    {
        maxChange = limit_cmd[1];
    }

    // Correct handling of sign and limit the change to the maximum allowed
    float limited_change = std::copysign(std::min(std::abs(change), maxChange), change);

    // Apply the limited change to the previous command
    accel_cmd = prev_cmd + limited_change;
    
    ROS_INFO("accel calculated: %f", accel_cmd);
    return accel_cmd;
}

void CliffordDriveNode::commandCallback(const clifford_drive::CliffordDriveCommand::ConstPtr& msg)
{
    // ROS_INFO("[commandCallback] Command received.");
    clifford_commands_.cmd_front_throttle_ = clip(msg->throttle,
        motor_params_.front_throttle_limit_[0],
        motor_params_.front_throttle_limit_[1]);
    if (msg->separate_throttle)
        clifford_commands_.cmd_rear_throttle_ = clip(msg->rear_throttle,
            motor_params_.rear_throttle_limit_[0],
            motor_params_.rear_throttle_limit_[1]);
    else
        clifford_commands_.cmd_rear_throttle_ = clifford_commands_.cmd_front_throttle_;
    clifford_commands_.cmd_front_steer_ = clip(msg->front_steering,
        servo_params_.front_steer_limit_[0],
        servo_params_.front_steer_limit_[1]);
    clifford_commands_.cmd_rear_steer_ = clip(msg->rear_steering,
        servo_params_.rear_steer_limit_[0],
        servo_params_.rear_steer_limit_[1]);
    clifford_commands_.last_cmd_time_ = ros::Time::now();
}

void CliffordDriveNode::reconnectCallback(const ros::TimerEvent& event)
{
    // TODO: Check if VESC is connected - is there a function in the library you are using?
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

// TODO: send command off to motor every command_period_.
void CliffordDriveNode::sendCommand(const ros::TimerEvent& event)
{
    if (!steer_connected_)
    {
        clifford_commands_.prev_front_throttle_ = 0;
        clifford_commands_.prev_rear_throttle_ = 0;
        return;
    }

    ROS_DEBUG("Front cmd: %f", clifford_commands_.cmd_front_steer_);
    ROS_DEBUG("Front steer scale: %d", servo_params_.front_steer_scale_);
    ROS_DEBUG("Front steer value: %f", servo_params_.front_steer_scale_ * clifford_commands_.cmd_front_steer_ + servo_params_.rear_steer_trim_);
    ROS_DEBUG("Setting front servo to %d", static_cast<int>(MAESTRO_CENTER_POSITION_ \
        + servo_params_.front_steer_scale_ * clifford_commands_.cmd_front_steer_ + servo_params_.front_steer_trim_));

    ROS_DEBUG("Rear cmd: %f", clifford_commands_.cmd_rear_steer_);
    ROS_DEBUG("Rear steer scale: %d", servo_params_.rear_steer_scale_);
    ROS_DEBUG("Rear steer value: %f", servo_params_.rear_steer_scale_ * clifford_commands_.cmd_rear_steer_ + servo_params_.rear_steer_trim_);
    ROS_DEBUG("Setting rear servo to %d", static_cast<int>(MAESTRO_CENTER_POSITION_ \
        + servo_params_.rear_steer_scale_ * clifford_commands_.cmd_rear_steer_ + servo_params_.rear_steer_trim_));

    servo_driver_->setTarget(servo_params_.front_steer_id_, static_cast<int>(MAESTRO_CENTER_POSITION_ \
        + servo_params_.front_steer_scale_ * clifford_commands_.cmd_front_steer_ + servo_params_.front_steer_trim_));
    servo_driver_->setTarget(servo_params_.rear_steer_id_, static_cast<int>(MAESTRO_CENTER_POSITION_ \
        + servo_params_.rear_steer_scale_ * clifford_commands_.cmd_rear_steer_ + servo_params_.rear_steer_trim_));

    clifford_commands_.prev_front_throttle_ = front_throttle_controller_.setThrottle(clifford_commands_.cmd_front_throttle_);
    clifford_commands_.prev_rear_throttle_ = rear_throttle_controller_.setThrottle(clifford_commands_.cmd_rear_throttle_);
    
    vesc_msgs::VescCommand msg_front;
    vesc_msgs::VescCommand msg_rear;

    ROS_DEBUG("Front throttle: %f \tRear throttle: %f", clifford_commands_.prev_front_throttle_, clifford_commands_.prev_rear_throttle_);
    msg_front.can_id = motor_params_.front_vesc_id_;
    msg_front.command = clifford_commands_.prev_front_throttle_;
    vesc_pub_.publish(msg_front);

    msg_rear.can_id = motor_params_.rear_vesc_id_;
    msg_rear.command = clifford_commands_.prev_rear_throttle_;
    vesc_pub_.publish(msg_rear);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clifford_drive_node");
    CliffordDriveNode clifford_drive;
    ros::spin();
    return 0;
}
#pragma once
#include "ck_ros_msgs_node/HMI_Signals.h"

class AutoBase
{
public:
    virtual ck_ros_msgs_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) = 0;
    virtual ~AutoBase() = default;
};
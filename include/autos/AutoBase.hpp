#pragma once
#include "hmi_agent_node/HMI_Signals.h"

class AutoBase
{
public:
    virtual hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted) = 0;
    virtual ~AutoBase() = default;
};
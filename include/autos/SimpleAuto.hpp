#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"

class SimpleAuto : public AutoBase
{
public:
    enum class SimpleAutoStates : int
    {
        BEGIN,
        DRIVE_PATH_1,
        GET_BALL_1,
        SHOOT_1,
        END
    };

    hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted) override;

private:
    SimpleAutoStates mAutoState {SimpleAutoStates::BEGIN};
    SimpleAutoStates mNextState {SimpleAutoStates::BEGIN};
};
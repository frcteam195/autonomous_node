#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"

class Auto1BallHub : public AutoBase
{
public:
    enum class AutoStates : int
    {
        BEGIN_PATH,
        DRIVE_PATH,
        GET_OPPONENT_BALL,
        SHOOT,
        END
    };

    hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    AutoStates mAutoState{AutoStates::BEGIN_PATH};
    AutoStates mNextState{AutoStates::BEGIN_PATH};
};
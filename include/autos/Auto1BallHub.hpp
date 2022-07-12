#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"

class Auto1BallHub : public AutoBase
{
public:
    enum class AutoStates : int
    {
        BEGIN_INITIAL_PATH,
        DRIVE_INITIAL_PATH,
        SHOOT,
        INTAKE_OPPONENT_BALL,
        BEGIN_STASH_PATH,
        DRIVE_STASH_PATH,
        STASH,
        END
    };

    hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    AutoStates mAutoState{AutoStates::BEGIN_INITIAL_PATH};
    AutoStates mNextState{AutoStates::BEGIN_INITIAL_PATH};
};
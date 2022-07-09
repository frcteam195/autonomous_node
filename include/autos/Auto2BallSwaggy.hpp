#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"

class Auto2BallSwaggy : public AutoBase
{
public:
    enum class AutoStates : int
    {
        BEGIN_INITIAL_PATH,
        DRIVE_INITIAL_PATH,
        SHOOT,
        BEGIN_OPONENT_PATH,
        DRIVE_OPONENT_PATH,
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

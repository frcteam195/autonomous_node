#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"

class AutoFollowPath : public AutoBase
{
public:
    enum class AutoFollowPathStates : int
    {
        BEGIN_PATH_1,
        DRIVE_PATH_1,
        BEGIN_PATH_2,
        DRIVE_PATH_2,
        BEGIN_PATH_3,
        DRIVE_PATH_3,
        BEGIN_PATH_4,
        DRIVE_PATH_4,
        END
    };

    hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    AutoFollowPathStates mAutoState {AutoFollowPathStates::BEGIN_PATH_1};
    AutoFollowPathStates mNextState {AutoFollowPathStates::BEGIN_PATH_1};
};
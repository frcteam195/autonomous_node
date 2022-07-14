#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"
#include "ros/ros.h"

class Auto1BallHub : public AutoBase
{
public:
    enum class AutoStates : int
    {
        BEGIN_INITIAL_PATH,
        DRIVE_INITIAL_PATH,
        SHOOT,
        BEGIN_STASH_PATH,
        DRIVE_STASH_PATH,
        STASH,
        END
    };

    hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    AutoStates mAutoState{AutoStates::SHOOT};
    AutoStates mNextState{AutoStates::SHOOT};
    bool loop_run_once = false;
};
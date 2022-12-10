#pragma once
#include "autos/AutoBase.hpp"
#include "ck_ros_msgs_node/HMI_Signals.h"
#include "ros/ros.h"

class Auto1BallHangar : public AutoBase
{
public:
    enum class AutoStates : int
    {
        SHOOT,
        BEGIN_HANGAR_PATH,
        DRIVE_HANGAR_PATH,
        BEGIN_STASH_PATH,
        DRIVE_STASH_PATH,
        STASH,
        END
    };

    ck_ros_msgs_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    AutoStates mAutoState{AutoStates::SHOOT};
    AutoStates mNextState{AutoStates::SHOOT};
    bool loop_run_once = false;
};
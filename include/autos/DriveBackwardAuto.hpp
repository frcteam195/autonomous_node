#pragma once
#include "autos/AutoBase.hpp"
#include "ck_ros_msgs_node/HMI_Signals.h"

class DriveBackwardAuto : public AutoBase
{
public:
    enum class DriveBackwardAutoStates : int
    {
        BEGIN,
        DRIVE_BACKWARDS,
        END
    };

    ck_ros_msgs_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    DriveBackwardAutoStates mAutoState {DriveBackwardAutoStates::BEGIN};
    DriveBackwardAutoStates mNextState {DriveBackwardAutoStates::BEGIN};
};
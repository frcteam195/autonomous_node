#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"

class DriveBackwardAuto : public AutoBase
{
public:
    enum class DriveBackwardAutoStates : int
    {
        BEGIN,
        DRIVE_BACKWARDS,
        END
    };

    hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted) override;

private:
    DriveBackwardAutoStates mAutoState {DriveBackwardAutoStates::BEGIN};
    DriveBackwardAutoStates mNextState {DriveBackwardAutoStates::BEGIN};
};
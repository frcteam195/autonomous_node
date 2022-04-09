#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"

class AutoMode3_1ball : public AutoBase
{
public:
    enum class AutoMode3States : int
    {
        BEGIN_PATH_1,   //Drive path id 3
        DRIVE_PATH_1,   //Drive path id 3
        SHOOT_1,        //Shoot preload
        BEGIN_PATH_2,   //Drive path id 4
        DRIVE_PATH_2,   //Drive path id 4
        END
    };

    hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted) override;

private:
    AutoMode3States mAutoState {AutoMode3States::BEGIN_PATH_1};
    AutoMode3States mNextState {AutoMode3States::BEGIN_PATH_1};
};
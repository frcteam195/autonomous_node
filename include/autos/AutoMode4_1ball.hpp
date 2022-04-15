#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"

class AutoMode4_1ball : public AutoBase
{
public:
    enum class AutoMode4States : int
    {
        BEGIN_PATH_1,   //Drive path id 3
        DRIVE_PATH_1,   //Drive path id 3
        SHOOT_1,        //Shoot preload
        BEGIN_PATH_2,   //Drive path id 4
        DRIVE_PATH_2,   //Drive path id 4
        END
    };

    hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    AutoMode4States mAutoState {AutoMode4States::BEGIN_PATH_1};
    AutoMode4States mNextState {AutoMode4States::BEGIN_PATH_1};
};
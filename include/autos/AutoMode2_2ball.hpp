#pragma once
#include "autos/AutoBase.hpp"
#include "hmi_agent_node/HMI_Signals.h"

class AutoMode2_2ball : public AutoBase
{
public:
    enum class AutoMode2States : int
    {
        BEGIN_PATH_1,   //Drive path id 2
        DRIVE_PATH_1,   //Drive path id 2
        GET_BALL_1, //Intake ball 1
        SHOOT_1,    //Shoot preload and ball 1
        END
    };

    hmi_agent_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    AutoMode2States mAutoState {AutoMode2States::BEGIN_PATH_1};
    AutoMode2States mNextState {AutoMode2States::BEGIN_PATH_1};
};
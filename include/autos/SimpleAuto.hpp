#pragma once
#include "autos/AutoBase.hpp"
#include "ck_ros_msgs_node/HMI_Signals.h"

class SimpleAuto : public AutoBase
{
public:
    enum class SimpleAutoStates : int
    {
        BEGIN,
        DRIVE_PATH_1,
        GET_BALL_1,
        SHOOT_1,
        END
    };

    ck_ros_msgs_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    SimpleAutoStates mAutoState {SimpleAutoStates::BEGIN};
    SimpleAutoStates mNextState {SimpleAutoStates::BEGIN};
};
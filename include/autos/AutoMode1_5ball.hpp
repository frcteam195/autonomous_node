#pragma once
#include "autos/AutoBase.hpp"
#include "ck_ros_msgs_node/HMI_Signals.h"

class AutoMode1_5ball : public AutoBase
{
public:
    enum class AutoMode1States : int
    {
        BEGIN_PATH_1,   //Drive path id 0
        DRIVE_PATH_1,   //Drive path id 0
        GET_BALL_3, //Intake ball 3
        SHOOT_1,    //Shoot preload and ball 3
        BEGIN_PATH_2,   //Drive path id 1
        DRIVE_PATH_2,   //Drive path id 1
        GET_BALL_2, //Intake Ball 2
        GET_BALL_7, //Intake Ball 7
        SHOOT_2,    //Shoot Ball 2 and 7
        GET_BALL_FEED,  //Intake fed ball
        SHOOT_3,    //Shoot last ball
        END
    };

    ck_ros_msgs_node::HMI_Signals stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id) override;

private:
    AutoMode1States mAutoState {AutoMode1States::BEGIN_PATH_1};
    AutoMode1States mNextState {AutoMode1States::BEGIN_PATH_1};
};
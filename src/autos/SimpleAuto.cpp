#include "autos/SimpleAuto.hpp"
#include "AutonomousHelper.hpp"
#include <string>

std::string PATH_1_POINT = "red_ball_3";

hmi_agent_node::HMI_Signals SimpleAuto::stepStateMachine(bool trajRunning, bool trajCompleted)
{
    hmi_agent_node::HMI_Signals autoHMISignals;
    autoHMISignals.allow_shoot = true;
    autoHMISignals.intake_rollers = true;


	static ros::Time time_state_entered = ros::Time::now();
	if(mNextState != mAutoState)
	{
		time_state_entered = ros::Time::now();
	}

    mAutoState = mNextState;

    switch (mAutoState)
    {
        case SimpleAutoStates::BEGIN:
        {
            AutonomousHelper::getInstance().move_to_position(PATH_1_POINT);
            mNextState = SimpleAutoStates::DRIVE_PATH_1;
            break;
        }
        case SimpleAutoStates::DRIVE_PATH_1:
        {
            if (!trajRunning && trajCompleted)
            {
                mNextState = SimpleAutoStates::GET_BALL_1;
            }
            break;
        }
        case SimpleAutoStates::GET_BALL_1:
        {
            if ((ros::Time::now() - time_state_entered) > ros::Duration(0.5))
            {
                mNextState = SimpleAutoStates::SHOOT_1;
            }
            break;
        }
        case SimpleAutoStates::SHOOT_1:
        {
            mNextState = SimpleAutoStates::END;
            break;
        }
        case SimpleAutoStates::END:
        {
            autoHMISignals.intake_rollers = false;
            break;
        }
    }
    return autoHMISignals;
}
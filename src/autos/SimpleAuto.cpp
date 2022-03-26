#include "autos/SimpleAuto.hpp"
#include "AutonomousHelper.hpp"
#include <string>

// std::string PATH_1_POINT = "red_ball_3";
std::string PATH_1_POINT = "red_ball_3";
std::string PATH_2_POINT = "red_ball_2";
std::string PATH_3_POINT = "red_ball_1";

#define AUTO_ENABLED

hmi_agent_node::HMI_Signals SimpleAuto::stepStateMachine(bool trajRunning, bool trajCompleted)
{
    (void) trajRunning;
    (void) trajCompleted;
    
    hmi_agent_node::HMI_Signals autoHMISignals;
#ifdef AUTO_ENABLED
    autoHMISignals.allow_shoot = true;
    autoHMISignals.intake_rollers = true;
#endif


	static ros::Time time_state_entered = ros::Time::now();
	if(mNextState != mAutoState)
	{
		time_state_entered = ros::Time::now();
	}

    mAutoState = mNextState;

#ifdef AUTO_ENABLED

    switch (mAutoState)
    {
        case SimpleAutoStates::BEGIN:
        {
            std::vector<std::pair<std::string, double>> v {{PATH_1_POINT, 90}, {PATH_2_POINT, 0}, {PATH_3_POINT, -90}};
            AutonomousHelper::getInstance().drive_trajectory_points(v);
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
#endif
    return autoHMISignals;
}
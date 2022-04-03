#include "autos/AutoMode2_2ball.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals AutoMode2_2ball::stepStateMachine(bool trajRunning, bool trajCompleted)
{
    (void) trajRunning;
    (void) trajCompleted;
    
    hmi_agent_node::HMI_Signals autoHMISignals;
    autoHMISignals.retract_intake = true;

	static ros::Time time_state_entered = ros::Time::now();
	if(mNextState != mAutoState)
	{
        ROS_INFO("Running state: %d", (int)mAutoState);
		time_state_entered = ros::Time::now();
	}

    mAutoState = mNextState;

    switch (mAutoState)
    {
        case AutoMode2States::BEGIN_PATH_1:
        {
            AutonomousHelper::getInstance().drive_trajectory(2);
            ROS_INFO("Started trajectory id: %d", 2);
            mNextState = AutoMode2States::DRIVE_PATH_1;
            break;
        }
        case AutoMode2States::DRIVE_PATH_1:
        {
            if((ros::Time::now() - time_state_entered) > ros::Duration(0.35))
            {
                autoHMISignals.intake_rollers = true;
            }
            
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoMode2States::GET_BALL_1;
            }
            break;
        }
        case AutoMode2States::GET_BALL_1:
        {
            autoHMISignals.intake_rollers = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1))
            {
                mNextState = AutoMode2States::SHOOT_1;
            }
            break;
        }
        case AutoMode2States::SHOOT_1:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(5))
            {
                mNextState = AutoMode2States::END;
            }
            break;
        }
        case AutoMode2States::END:
        {
            mNextState = AutoMode2States::END;  //Wait
            break;
        }
    }
    return autoHMISignals;
}
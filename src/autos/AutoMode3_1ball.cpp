#include "autos/AutoMode3_1ball.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals AutoMode3_1ball::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
{
    (void) trajRunning;
    (void) trajCompleted;
    (void) traj_id;
    
    hmi_agent_node::HMI_Signals autoHMISignals = {};

	static ros::Time time_state_entered = ros::Time::now();
	if(mNextState != mAutoState)
	{
        ROS_INFO("Running state: %d", (int)mAutoState);
		time_state_entered = ros::Time::now();
	}

    mAutoState = mNextState;

    switch (mAutoState)
    {
        case AutoMode3States::BEGIN_PATH_1:
        {
            AutonomousHelper::getInstance().drive_trajectory(3);
            ROS_INFO("Started trajectory id: %d", 3);
            mNextState = AutoMode3States::DRIVE_PATH_1;
            break;
        }
        case AutoMode3States::DRIVE_PATH_1:
        {   
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoMode3States::SHOOT_1;
            }
            break;
        }
        case AutoMode3States::SHOOT_1:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(3))
            {
                mNextState = AutoMode3States::END;
            }
            break;
        }
        case AutoMode3States::BEGIN_PATH_2:
        {
            AutonomousHelper::getInstance().drive_trajectory(4);
            ROS_INFO("Started trajectory id: %d", 4);
            mNextState = AutoMode3States::DRIVE_PATH_2;
            break;
        }
        case AutoMode3States::DRIVE_PATH_2:
        {
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoMode3States::END;
            }
            break;
        }
        case AutoMode3States::END:
        {
            mNextState = AutoMode3States::END;  //Wait
            break;
        }
    }
    return autoHMISignals;
}
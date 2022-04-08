#include "autos/AutoMode4_1ball.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals AutoMode4_1ball::stepStateMachine(bool trajRunning, bool trajCompleted)
{
    (void) trajRunning;
    (void) trajCompleted;
    
    hmi_agent_node::HMI_Signals autoHMISignals;

	static ros::Time time_state_entered = ros::Time::now();
    static int intake_counter = 0;
	if(mNextState != mAutoState)
	{
        ROS_INFO("Running state: %d", (int)mAutoState);
		time_state_entered = ros::Time::now();
	}

    mAutoState = mNextState;

    switch (mAutoState)
    {
        case AutoMode4States::BEGIN_PATH_1:
        {
            AutonomousHelper::getInstance().drive_trajectory(5);
            ROS_INFO("Started trajectory id: %d", 5);
            mNextState = AutoMode4States::DRIVE_PATH_1;
            break;
        }
        case AutoMode4States::DRIVE_PATH_1:
        {   
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoMode4States::SHOOT_1;
            }
            break;
        }
        case AutoMode4States::SHOOT_1:
        {
            autoHMISignals.retract_intake = true;
            if (intake_counter++ < 20)
            {
                autoHMISignals.intake_rollers = true;
            }
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(2))
            {
                mNextState = AutoMode4States::BEGIN_PATH_2;
            }
            break;
        }
        case AutoMode4States::BEGIN_PATH_2:
        {
            autoHMISignals.retract_intake = true;
            AutonomousHelper::getInstance().drive_trajectory(6);
            ROS_INFO("Started trajectory id: %d", 6);
            mNextState = AutoMode4States::DRIVE_PATH_2;
            break;
        }
        case AutoMode4States::DRIVE_PATH_2:
        {
            autoHMISignals.retract_intake = true;
            autoHMISignals.intake_rollers = true;
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoMode4States::END;
            }
            break;
        }
        case AutoMode4States::END:
        {
            mNextState = AutoMode4States::END;  //Wait
            break;
        }
    }
    return autoHMISignals;
}
#include "autos/AutoMode1_5ball.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals AutoMode1_5ball::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
{
    (void) trajRunning;
    (void) trajCompleted;
    (void) traj_id;
    
    hmi_agent_node::HMI_Signals autoHMISignals = {};
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
        case AutoMode1States::BEGIN_PATH_1:
        {
            AutonomousHelper::getInstance().drive_trajectory(0);
            ROS_INFO("Started trajectory id: %d", 0);
            mNextState = AutoMode1States::DRIVE_PATH_1;
            break;
        }
        case AutoMode1States::DRIVE_PATH_1:
        {
            if((ros::Time::now() - time_state_entered) > ros::Duration(0.35))
            {
                autoHMISignals.intake_rollers = true;
            }
            
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoMode1States::GET_BALL_3;
            }
            break;
        }
        case AutoMode1States::GET_BALL_3:
        {
            autoHMISignals.intake_rollers = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1))
            {
                mNextState = AutoMode1States::SHOOT_1;
            }
            break;
        }
        case AutoMode1States::SHOOT_1:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(2))
            {
                mNextState = AutoMode1States::BEGIN_PATH_2;
            }
            break;
        }
        case AutoMode1States::BEGIN_PATH_2:
        {
            autoHMISignals.intake_rollers = true;
            AutonomousHelper::getInstance().drive_trajectory(1);
            ROS_INFO("Started trajectory id: %d", 1);
            mNextState = AutoMode1States::DRIVE_PATH_2;
            break;
        }
        case AutoMode1States::DRIVE_PATH_2:
        {
            autoHMISignals.intake_rollers = true;
            mNextState = AutoMode1States::GET_BALL_2;
            break;
        }
        case AutoMode1States::GET_BALL_2:
        {
            autoHMISignals.intake_rollers = true;
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoMode1States::GET_BALL_7;
            }
            break;
        }
        case AutoMode1States::GET_BALL_7:
        {
            autoHMISignals.intake_rollers = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(2.5))
            {
                mNextState = AutoMode1States::SHOOT_2;
            }
            break;
        }
        case AutoMode1States::SHOOT_2:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(2))
            {
                mNextState = AutoMode1States::GET_BALL_FEED;
            }
            break;
        }
        case AutoMode1States::GET_BALL_FEED:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(2.5))
            {
                mNextState = AutoMode1States::SHOOT_3;
            }
            break;
        }
        case AutoMode1States::SHOOT_3:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(6))
            {
                mNextState = AutoMode1States::END;
            }
            break;
        }
        case AutoMode1States::END:
        {
            mNextState = AutoMode1States::END;  //Wait
            break;
        }
    }
    return autoHMISignals;
}
#include "autos/AutoMode5_5ball.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals AutoMode5_5ball::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
{    
    hmi_agent_node::HMI_Signals autoHMISignals = {};
    // memset(&autoHMISignals, 0, sizeof(hmi_agent_node::HMI_Signals));

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
        case AutoMode5States::BEGIN_PATH_1:
        {
            // AutonomousHelper::getInstance().drive_trajectory(7);
            AutonomousHelper::getInstance().drive_trajectory(12);
            ROS_INFO("Started trajectory id: %d", 12);
            mNextState = AutoMode5States::DRIVE_PATH_1;
            break;
        }
        case AutoMode5States::DRIVE_PATH_1:
        {
            if((ros::Time::now() - time_state_entered) > ros::Duration(0.35))
            {
                autoHMISignals.intake_rollers = true;
            }
            
            if (!trajRunning && trajCompleted && traj_id == 12)
            {
                mNextState = AutoMode5States::GET_BALL_3;
            }
            break;
        }
        case AutoMode5States::GET_BALL_3:
        {
            autoHMISignals.intake_rollers = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(0.25))
            {
                mNextState = AutoMode5States::SHOOT_1;
            }
            break;
        }
        case AutoMode5States::SHOOT_1:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1.75))
            {
                mNextState = AutoMode5States::BEGIN_PATH_2;
            }
            break;
        }
        case AutoMode5States::BEGIN_PATH_2:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = false;
            // AutonomousHelper::getInstance().drive_trajectory(8);
            AutonomousHelper::getInstance().drive_trajectory(13);
            ROS_INFO("Started trajectory id: %d", 8);
            mNextState = AutoMode5States::DRIVE_PATH_2;
            break;
        }
        case AutoMode5States::DRIVE_PATH_2:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = false;
            mNextState = AutoMode5States::GET_BALL_2;
            break;
        }
        case AutoMode5States::GET_BALL_2:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = false;
            if (!trajRunning && trajCompleted && traj_id == 13)
            {
                mNextState = AutoMode5States::SHOOT_2;
            }
            break;
        }
        case AutoMode5States::SHOOT_2:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1.5))
            {
                mNextState = AutoMode5States::BEGIN_PATH_3;
            }
            break;
        }
        case AutoMode5States::BEGIN_PATH_3:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = false;

            // AutonomousHelper::getInstance().drive_trajectory(9);
            AutonomousHelper::getInstance().drive_trajectory(14);
            ROS_INFO("Started trajectory id: %d", 9);
            mNextState = AutoMode5States::DRIVE_PATH_3;
            break;
        }
        case AutoMode5States::DRIVE_PATH_3:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = false;

            if (!trajRunning && trajCompleted && traj_id == 14)
            {
                mNextState = AutoMode5States::GET_BALL_7;
            }
            break;
        }
        case AutoMode5States::GET_BALL_7:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = false;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(0.25))
            {
                mNextState = AutoMode5States::BEGIN_PATH_4;
            }
            break;
        }
        case AutoMode5States::BEGIN_PATH_4:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = false;

            // AutonomousHelper::getInstance().drive_trajectory(10);
            AutonomousHelper::getInstance().drive_trajectory(15);
            ROS_INFO("Started trajectory id: %d", 10);
            mNextState = AutoMode5States::DRIVE_PATH_4;
            break;
        }
        case AutoMode5States::DRIVE_PATH_4:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = false;

            if (!trajRunning && trajCompleted && traj_id == 15)
            {
                mNextState = AutoMode5States::GET_BALL_FEED;
            }
            break;
        }
        case AutoMode5States::GET_BALL_FEED:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = false;

            if ((ros::Time::now() - time_state_entered) > ros::Duration(0.5))
            {
                mNextState = AutoMode5States::SHOOT_3;
            }
            break;
        }
        case AutoMode5States::SHOOT_3:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(8))
            {
                mNextState = AutoMode5States::END;
            }
            break;
        }
        case AutoMode5States::END:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            mNextState = AutoMode5States::END;  //Wait
            break;
        }
    }
    return autoHMISignals;
}
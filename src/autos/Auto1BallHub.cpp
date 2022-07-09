#include "autos/Auto1BallHub.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals Auto1BallHub::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
{   
    (void) traj_id; // Unused

    hmi_agent_node::HMI_Signals autoHMISignals;
    memset(&autoHMISignals, 0, sizeof(hmi_agent_node::HMI_Signals));
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
        case AutoStates::BEGIN_INITIAL_PATH:
        {
            AutonomousHelper::getInstance().drive_trajectory(40);
            ROS_INFO("Started trajectory id: %d", 40);
            mNextState = AutoStates::DRIVE_INITIAL_PATH;
            break;
        }
        case AutoStates::DRIVE_INITIAL_PATH:
        {
            if((ros::Time::now() - time_state_entered) > ros::Duration(0.35))
            {
                autoHMISignals.intake_rollers = true;
            }
            
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoStates::GET_OPPONENT_BALL;
            }
            break;
        }
        case AutoStates::GET_OPPONENT_BALL:
        {
            autoHMISignals.intake_rollers = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1))
            {
                mNextState = AutoStates::SHOOT;
            }
            break;
        }
        case AutoStates::SHOOT:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(2))
            {
                mNextState = AutoStates::BEGIN_HANGAR_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_HANGAR_PATH:
        {
            AutonomousHelper::getInstance().drive_trajectory(41);
            ROS_INFO("Started trajectory id: %d", 41);
            mNextState = AutoStates::DRIVE_HANGAR_PATH;
            break;
        }
        case AutoStates::DRIVE_HANGAR_PATH:
        {
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoStates::END;
            }
            break;
        }
        case AutoStates::END:
        {
            mNextState = AutoStates::END;
            break;
        }
    }

    return autoHMISignals;
}
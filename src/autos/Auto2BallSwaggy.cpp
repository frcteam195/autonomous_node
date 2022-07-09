#include "autos/Auto2BallSwaggy.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals Auto2BallSwaggy::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
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
            AutonomousHelper::getInstance().drive_trajectory(50);
            mNextState = AutoStates::DRIVE_INITIAL_PATH;
            break;
        }
        case AutoStates::DRIVE_INITIAL_PATH:
        {   
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoStates::SHOOT;
            }
            break;
        }
        case AutoStates::SHOOT:
        {
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(2))
            {
                mNextState = AutoStates::END;
            }
            break;
        }
        case AutoStates::INTAKE_OPPONENT_BALL:
        {
            autoHMISignals.intake_rollers = true;

            if ((ros::Time::now() - time_state_entered) > ros::Duration(0.5))
            {
                mNextState = AutoStates::END;
            }
            break;
        }
        case AutoStates::BEGIN_HANGAR_PATH:
        {
            autoHMISignals.intake_rollers = true;
            AutonomousHelper::getInstance().drive_trajectory(51);
            mNextState = AutoStates::DRIVE_INITIAL_PATH;
            break;
        }
        case AutoStates::DRIVE_HANGAR_PATH:
        {   
            autoHMISignals.intake_rollers = true;

            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoStates::SHOOT;
            }
            break;
        }
        case AutoStates::BEGIN_STASH_PATH:
        {
            AutonomousHelper::getInstance().drive_trajectory(52);
            mNextState = AutoStates::DRIVE_INITIAL_PATH;
            break;
        }
        case AutoStates::DRIVE_STASH_PATH:
        {  
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoStates::STASH;
            }
            break;
        }
        case AutoStates::STASH:
        {
            if ((ros::Time::now() - time_state_entered) > ros::Duration(2))
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

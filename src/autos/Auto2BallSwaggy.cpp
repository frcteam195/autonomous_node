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
            AutonomousHelper::getInstance().drive_trajectory(60);
            mNextState = AutoStates::DRIVE_INITIAL_PATH;
            autoHMISignals.intake_rollers = true;
            break;
        }
        case AutoStates::DRIVE_INITIAL_PATH:
        {
            if (!trajRunning && trajCompleted && traj_id == 60)
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
                mNextState = AutoStates::BEGIN_OPONENT_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_OPONENT_PATH:
        {
            AutonomousHelper::getInstance().drive_trajectory(61);
            autoHMISignals.allow_shoot = false;
            autoHMISignals.intake_rollers = true;

            if ((ros::Time::now() - time_state_entered) > ros::Duration(0.5))
            {
                mNextState = AutoStates::DRIVE_OPONENT_PATH;
            }
            break;
        }
        case AutoStates::DRIVE_OPONENT_PATH:
        {
            if (!trajRunning && trajCompleted && traj_id == 61)
            {
                mNextState = AutoStates::BEGIN_STASH_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_STASH_PATH:
        {
            AutonomousHelper::getInstance().drive_trajectory(62);
            mNextState = AutoStates::DRIVE_STASH_PATH;
            break;
        }
        case AutoStates::DRIVE_STASH_PATH:
        {  
            if (!trajRunning && trajCompleted && traj_id == 62)
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

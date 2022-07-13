#include "autos/Auto1BallHub.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals Auto1BallHub::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
{
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
        case AutoStates::SHOOT:
        {
            autoHMISignals.intake_rollers = true;
            autoHMISignals.intake_do_not_eject = true;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(4.0))
            {
                mNextState = AutoStates::BEGIN_INITIAL_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_INITIAL_PATH:
        {
            AutonomousHelper::getInstance().drive_trajectory(40);
            mNextState = AutoStates::DRIVE_INITIAL_PATH;
            break;
        }
        case AutoStates::DRIVE_INITIAL_PATH:
        {
            if((ros::Time::now() - time_state_entered) > ros::Duration(0.35))
            {
                autoHMISignals.intake_rollers = true;
                autoHMISignals.intake_do_not_eject = true;
            }
            
            if (!trajRunning && trajCompleted && traj_id == 40)
            {
                mNextState = AutoStates::BEGIN_STASH_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_STASH_PATH:
        {
            autoHMISignals.intake_do_not_eject = true;
            AutonomousHelper::getInstance().drive_trajectory(41);
            mNextState = AutoStates::DRIVE_STASH_PATH;
            break;
        }
        case AutoStates::DRIVE_STASH_PATH:
        {  
            autoHMISignals.intake_do_not_eject = true;
            if (!trajRunning && trajCompleted && traj_id == 41)
            {
                mNextState = AutoStates::STASH;
            }
            break;
        }
        case AutoStates::STASH:
        {
            autoHMISignals.manual_outake_back = true;
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
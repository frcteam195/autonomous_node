#include "autos/Auto1BallHangar.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals Auto1BallHangar::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
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
            autoHMISignals.retract_intake = false;
            AutonomousHelper::getInstance().drive_trajectory(50);
            mNextState = AutoStates::DRIVE_INITIAL_PATH;
            break;
        }
        case AutoStates::DRIVE_INITIAL_PATH:
        {
            autoHMISignals.retract_intake = false;
            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoStates::SHOOT;
            }
            break;
        }
        case AutoStates::SHOOT:
        {
            autoHMISignals.retract_intake = false;
            autoHMISignals.allow_shoot = true;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(2))
            {
                mNextState = AutoStates::INTAKE_OPPONENT_BALL;
            }
            break;
        }
        case AutoStates::INTAKE_OPPONENT_BALL:
        {
            autoHMISignals.intake_do_not_eject = false;
            autoHMISignals.intake_rollers = true;

            if ((ros::Time::now() - time_state_entered) > ros::Duration(0.5))
            {
                mNextState = AutoStates::BEGIN_HANGAR_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_HANGAR_PATH:
        {
            autoHMISignals.intake_do_not_eject = false;
            autoHMISignals.intake_rollers = true;

            AutonomousHelper::getInstance().drive_trajectory(51);
            mNextState = AutoStates::DRIVE_HANGAR_PATH;
            break;
        }
        case AutoStates::DRIVE_HANGAR_PATH:
        {   
            autoHMISignals.intake_do_not_eject = false;
            autoHMISignals.intake_rollers = true;

            if (!trajRunning && trajCompleted)
            {
                mNextState = AutoStates::BEGIN_STASH_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_STASH_PATH:
        {
            autoHMISignals.intake_do_not_eject = false;
            AutonomousHelper::getInstance().drive_trajectory(52);
            mNextState = AutoStates::DRIVE_STASH_PATH;
            break;
        }
        case AutoStates::DRIVE_STASH_PATH:
        {  
            autoHMISignals.intake_do_not_eject = false;
            if (!trajRunning && trajCompleted)
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
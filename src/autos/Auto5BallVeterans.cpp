#include "autos/Auto5BallVeterans.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals Auto5BallVeterans::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
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
        case AutoStates::BEGIN_INITIAL_PATH:
        {
            autoHMISignals.retract_intake = false;
            AutonomousHelper::getInstance().drive_trajectory(70);
            mNextState = AutoStates::DRIVE_INITIAL_PATH;
            break;
        }
        case AutoStates::DRIVE_INITIAL_PATH:
        {   
            if((ros::Time::now() - time_state_entered) > ros::Duration(0.35))
            {
                autoHMISignals.intake_rollers = true;
            }

            if (!trajRunning && trajCompleted && traj_id == 70)
            {
                mNextState = AutoStates::SHOOT;
            }
            break;
        }
        case AutoStates::SHOOT:
        {
            autoHMISignals.allow_shoot = true;
            autoHMISignals.intake_rollers = true;
            autoHMISignals.shoot_3ball = true;

            if ((ros::Time::now() - time_state_entered) > ros::Duration(3))
            {
                mNextState = AutoStates::BEGIN_HUMAN_PATH;
            }
            break;
        }
        
        case AutoStates::BEGIN_HUMAN_PATH:
        {
            AutonomousHelper::getInstance().drive_trajectory(71);
            mNextState = AutoStates::DRIVE_HUMAN_PATH;
            break;
        }
        case AutoStates::DRIVE_HUMAN_PATH:
        {   
            autoHMISignals.intake_rollers = true;

            if (!trajRunning && trajCompleted && traj_id == 71)
            {
                mNextState = AutoStates::BEGIN_RETURN_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_RETURN_PATH:
        {
            AutonomousHelper::getInstance().drive_trajectory(72);
            mNextState = AutoStates::DRIVE_RETURN_PATH;
            break;
        }
        case AutoStates::DRIVE_RETURN_PATH:
        {  
            autoHMISignals.intake_rollers = true;

            if (!trajRunning && trajCompleted && traj_id == 72)
            {
                mNextState = AutoStates::SHOOT2;
            }
            break;
        }
        case AutoStates::SHOOT2:
        {
            autoHMISignals.allow_shoot = true;
            autoHMISignals.intake_rollers = true;

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
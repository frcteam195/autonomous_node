#include "autos/Auto1BallHangar.hpp"
#include "AutonomousHelper.hpp"
#include <string>

ck_ros_msgs_node::HMI_Signals Auto1BallHangar::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
{
    ck_ros_msgs_node::HMI_Signals autoHMISignals {};  //NEED TO CHECK IF VALUE INIT WORKS PROPERLY AND DOES NOT REUSE VALUES
    // memset(&autoHMISignals, 0, sizeof(hmi_agent_node::HMI_Signals));
    autoHMISignals.retract_intake = true;

	static ros::Time time_state_entered = ros::Time::now();
    if (!loop_run_once)
    {
        time_state_entered = ros::Time::now();
        loop_run_once = true;
    }

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
            autoHMISignals.allow_shoot = true;

            if((ros::Time::now() - time_state_entered) > ros::Duration(0.35))
            {
                autoHMISignals.intake_rollers = true;
            }

            if ((ros::Time::now() - time_state_entered) > ros::Duration(2.5))
            {
                mNextState = AutoStates::BEGIN_HANGAR_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_HANGAR_PATH:
        {
            autoHMISignals.intake_do_not_eject = true;
            autoHMISignals.intake_rollers = true;

            AutonomousHelper::getInstance().drive_trajectory(50);
            mNextState = AutoStates::DRIVE_HANGAR_PATH;
            break;
        }
        case AutoStates::DRIVE_HANGAR_PATH:
        {   
            autoHMISignals.intake_do_not_eject = true;

            if ((ros::Time::now() - time_state_entered) < ros::Duration(1.5) ||
                (ros::Time::now() - time_state_entered) > ros::Duration(4.0))
            {
                autoHMISignals.intake_rollers = true;
            }

            if (!trajRunning && trajCompleted && traj_id == 50)
            {
                mNextState = AutoStates::BEGIN_STASH_PATH;
            }
            break;
        }
        case AutoStates::BEGIN_STASH_PATH:
        {
            autoHMISignals.intake_do_not_eject = true;

            AutonomousHelper::getInstance().drive_trajectory(51);
            mNextState = AutoStates::DRIVE_STASH_PATH;
            break;
        }
        case AutoStates::DRIVE_STASH_PATH:
        {  
            autoHMISignals.intake_do_not_eject = true;

            if (!trajRunning && trajCompleted && traj_id == 51)
            {
                mNextState = AutoStates::STASH;
            }
            break;
        }
        case AutoStates::STASH:
        {
            autoHMISignals.manual_outake_back = true;

            if ((ros::Time::now() - time_state_entered) > ros::Duration(4.0))
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
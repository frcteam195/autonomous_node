#include "autos/AutoFollowPath.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals AutoFollowPath::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
{    
    // bool test = !trajRunning && trajCompleted;

    // ROS_ERROR("Running: %d Completed: %d ID: %d", trajRunning, trajCompleted, traj_id);
    // ROS_ERROR("Is finished: %d\n\n", test);

    hmi_agent_node::HMI_Signals autoHMISignals = {};
    // memset(&autoHMISignals, 0, sizeof(hmi_agent_node::HMI_Signals));

    autoHMISignals.retract_intake = false;

	static ros::Time time_state_entered = ros::Time::now();
	if(mNextState != mAutoState)
	{
        // ROS_ERROR("Running state: %d", (int)mAutoState);
		time_state_entered = ros::Time::now();
	}

    mAutoState = mNextState;

    switch (mAutoState)
    {
        case AutoFollowPathStates::BEGIN_PATH_1:
        {
            AutonomousHelper::getInstance().drive_trajectory(12);
            ROS_ERROR("Started trajectory id: %d", 12);
            mNextState = AutoFollowPathStates::DRIVE_PATH_1;
            break;
        }
        case AutoFollowPathStates::DRIVE_PATH_1:
        {
            if (!trajRunning && trajCompleted && traj_id == 12)
            // if ((ros::Time::now() - time_state_entered) > ros::Duration(5))
            {
                ROS_ERROR("Finished trajectory id: %d\n", 12);
                mNextState = AutoFollowPathStates::BEGIN_PATH_2;
            }
            break;
        }
        case AutoFollowPathStates::BEGIN_PATH_2:
        {
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1)) {
                AutonomousHelper::getInstance().drive_trajectory(13);
                ROS_ERROR("Started trajectory id: %d", 13);
                mNextState = AutoFollowPathStates::DRIVE_PATH_2;
            }
            break;
        }
        case AutoFollowPathStates::DRIVE_PATH_2:
        {
            if (!trajRunning && trajCompleted && traj_id == 13)
            // if ((ros::Time::now() - time_state_entered) > ros::Duration(5))
            {
                ROS_ERROR("Finished trajectory id: %d\n", 13);
                mNextState = AutoFollowPathStates::BEGIN_PATH_3;
            }
            break;
        }

        case AutoFollowPathStates::BEGIN_PATH_3:
        {
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1)) {
                AutonomousHelper::getInstance().drive_trajectory(14);
                ROS_ERROR("Started trajectory id: %d", 14);
                mNextState = AutoFollowPathStates::DRIVE_PATH_3;
            }
            break;
        }
        case AutoFollowPathStates::DRIVE_PATH_3:
        {
            if (!trajRunning && trajCompleted && traj_id == 14)
            // if ((ros::Time::now() - time_state_entered) > ros::Duration(5))
            {
                ROS_ERROR("Finished trajectory id: %d\n", 14);
                mNextState = AutoFollowPathStates::BEGIN_PATH_4;
            }
            break;
        }
        case AutoFollowPathStates::BEGIN_PATH_4:
        {
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1)) {
                AutonomousHelper::getInstance().drive_trajectory(15);
                ROS_ERROR("Started trajectory id: %d", 15);
                mNextState = AutoFollowPathStates::DRIVE_PATH_4;
            }
            break;
        }
        case AutoFollowPathStates::DRIVE_PATH_4:
        {
            if (!trajRunning && trajCompleted && traj_id == 15) {
                ROS_ERROR("Finished trajectory id: %d\n", 15);
                mNextState = AutoFollowPathStates::END;
            }
            break;
        }
        case AutoFollowPathStates::END:
        {
            break;
        }
    }
    return autoHMISignals;
}
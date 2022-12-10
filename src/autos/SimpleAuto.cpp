#include "autos/SimpleAuto.hpp"
#include "AutonomousHelper.hpp"
#include <string>

#define AUTO_ENABLED

ck_ros_msgs_node::HMI_Signals SimpleAuto::stepStateMachine(bool trajRunning, bool trajCompleted, int traj_id)
{
    (void) trajRunning;
    (void) trajCompleted;
    (void) traj_id;
    
    ck_ros_msgs_node::HMI_Signals autoHMISignals = {};
#ifdef AUTO_ENABLED
    autoHMISignals.allow_shoot = true;
    autoHMISignals.intake_rollers = true;
#endif


	static ros::Time time_state_entered = ros::Time::now();
	if(mNextState != mAutoState)
	{
		time_state_entered = ros::Time::now();
	}

    mAutoState = mNextState;

#ifdef AUTO_ENABLED
    ROS_INFO("Running state: %d", (int)mAutoState);
    switch (mAutoState)
    {
        case SimpleAutoStates::BEGIN:
        {
            AutonomousHelper::getInstance().drive_trajectory(0);
            ROS_INFO("Started trajectory id: %d", 0);
            mNextState = SimpleAutoStates::DRIVE_PATH_1;
            break;
        }
        case SimpleAutoStates::DRIVE_PATH_1:
        {
            if (!trajRunning && trajCompleted)
            {
                mNextState = SimpleAutoStates::GET_BALL_1;
            }
            break;
        }
        case SimpleAutoStates::GET_BALL_1:
        {
            AutonomousHelper::getInstance().drive_trajectory(1);
            ROS_INFO("Started trajectory id: %d", 1);
            mNextState = SimpleAutoStates::SHOOT_1;
            // if ((ros::Time::now() - time_state_entered) > ros::Duration(0.5))
            // {
            //     mNextState = SimpleAutoStates::SHOOT_1;
            // }
            break;
        }
        case SimpleAutoStates::SHOOT_1:
        {
            if (!trajRunning && trajCompleted)
            {
                mNextState = SimpleAutoStates::END;
            }
            // mNextState = SimpleAutoStates::END;
            break;
        }
        case SimpleAutoStates::END:
        {
            autoHMISignals.intake_rollers = false;
            break;
        }
    }
#endif
    return autoHMISignals;
}
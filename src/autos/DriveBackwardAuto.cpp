#include "autos/DriveBackwardAuto.hpp"
#include "AutonomousHelper.hpp"
#include <string>

hmi_agent_node::HMI_Signals DriveBackwardAuto::stepStateMachine(bool trajRunning, bool trajCompleted)
{
    (void) trajRunning;
    (void) trajCompleted;
    
    hmi_agent_node::HMI_Signals autoHMISignals;
    autoHMISignals.allow_shoot = true;
    autoHMISignals.intake_rollers = true;
    autoHMISignals.retract_intake = true;

	static ros::Time time_state_entered = ros::Time::now();
	if(mNextState != mAutoState)
	{
		time_state_entered = ros::Time::now();
	}

    mAutoState = mNextState;

    switch (mAutoState)
    {
        case DriveBackwardAutoStates::BEGIN:
        {
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1.0))
            {
                mNextState = DriveBackwardAutoStates::DRIVE_BACKWARDS;
            }
            break;
        }
        case DriveBackwardAutoStates::DRIVE_BACKWARDS:
        {
            autoHMISignals.drivetrain_fwd_back = -0.3;
            if ((ros::Time::now() - time_state_entered) > ros::Duration(1.0))
            {
                mNextState = DriveBackwardAutoStates::END;
            }
            break;
        }
        case DriveBackwardAutoStates::END:
        {
            autoHMISignals.drivetrain_fwd_back = 0;
            break;
        }
    }

    return autoHMISignals;
}
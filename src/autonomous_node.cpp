#include "autonomous_node.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include <action_helper/action_helper.hpp>

#include <local_planner_node/PlanReq.h>
#include <local_planner_node/TrajectoryFollowCue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <robot_localization/SetPose.h>
#include "rio_control_node/Robot_Status.h"
#include "autos/AutoBase.hpp"
#include "autos/SimpleAuto.hpp"
#include "autos/DriveBackwardAuto.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include "AutonomousHelper.hpp"
#include <atomic>
#include <quesadilla_auto_node/Planner_Output.h>


#define RATE (100)

ros::NodeHandle* node = nullptr;
std::atomic_int32_t selected_auto_mode {0};
std::atomic_bool traj_follow_active {false};
std::atomic_bool traj_follow_complete {false};

void planner_callback (const quesadilla_auto_node::Planner_Output &msg)
{
    traj_follow_active = msg.trajectory_active;
    traj_follow_complete = msg.trajectory_completed;
}

void robot_status_callback (const rio_control_node::Robot_Status &msg)
{
    AutonomousHelper::getInstance().setRobotState((RobotState) msg.robot_state);
    AutonomousHelper::getInstance().setAllianceColor((AllianceColor) msg.alliance);
    selected_auto_mode = msg.selected_auto;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "autonomous_node");
	ros::NodeHandle n;
    ros::Rate rate(RATE);
	node = &n;

    static ros::Subscriber robot_status_subscriber = node->subscribe("/RobotStatus", 1, robot_status_callback);
    static ros::Subscriber q_planner_subscriber = node->subscribe("/QuesadillaPlannerOutput", 1, planner_callback);
    static ros::Publisher auto_hmi_publisher = node->advertise<hmi_agent_node::HMI_Signals>("/HMISignals", 1);
    (void)AutonomousHelper::getInstance();

    AutoBase* autoModePrg = nullptr;
    RobotState last_robot_state = RobotState::DISABLED;
    hmi_agent_node::HMI_Signals auto_hmi_signals;
    while( ros::ok() )
    {
        if(AutonomousHelper::getInstance().getRobotState() == RobotState::AUTONOMOUS && last_robot_state == RobotState::DISABLED && !autoModePrg)
        {
            AutonomousHelper::getInstance().initialize_position();
            traj_follow_active = false;
            traj_follow_complete = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            switch (selected_auto_mode)
            {
                case 0:
                {
                    autoModePrg = new SimpleAuto();
                    break;
                }
                case 1:
                {
                    autoModePrg = nullptr;
                    break;
                }
                case 2:
                {
                    autoModePrg = nullptr;
                    break;
                }
                default:
                {
                    autoModePrg = nullptr;
                    break;
                }
            }
        }
        last_robot_state = AutonomousHelper::getInstance().getRobotState();

        if(AutonomousHelper::getInstance().getRobotState() != RobotState::AUTONOMOUS)
        {
            if (autoModePrg)
            {
                delete autoModePrg;
                autoModePrg = nullptr;
            }
        }

        if (autoModePrg != nullptr)
        {
            auto_hmi_signals = autoModePrg->stepStateMachine(traj_follow_active, traj_follow_complete);
            auto_hmi_publisher.publish(auto_hmi_signals);
        }
        else if (traj_follow_active)
        {
            ROS_INFO("Force stopping trajectory!");
            AutonomousHelper::getInstance().stop_trajectory();
        }

        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}

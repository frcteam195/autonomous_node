#include "autonomous_node.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <robot_localization/SetPose.h>
#include "rio_control_node/Robot_Status.h"
#include "autos/AutoBase.hpp"
#include "autos/AutoMode1_5ball.hpp"
#include "autos/AutoMode5_5ball.hpp"
#include "autos/AutoMode1_5ball_Alt.hpp"
#include "autos/AutoMode2_2ball.hpp"
#include "autos/AutoMode3_1ball.hpp"
#include "autos/AutoMode4_1ball.hpp"
#include "autos/AutoFollowPath.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include "AutonomousHelper.hpp"
#include <atomic>
#include <quesadilla_auto_node/Planner_Output.h>


#define RATE (100)

ros::NodeHandle* node = nullptr;
std::atomic_int32_t selected_auto_mode {0};
std::atomic_bool traj_follow_active {false};
std::atomic_bool traj_follow_complete {false};
std::atomic_int32_t traj_id {-1};

void planner_callback (const quesadilla_auto_node::Planner_Output &msg)
{
    traj_follow_active = msg.trajectory_active;
    traj_follow_complete = msg.trajectory_completed;
    traj_id = msg.trajectory_id;
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

    static ros::Subscriber robot_status_subscriber = node->subscribe("/RobotStatus", 1, robot_status_callback, ros::TransportHints().tcpNoDelay());
    static ros::Subscriber q_planner_subscriber = node->subscribe("/QuesadillaPlannerOutput", 1, planner_callback, ros::TransportHints().tcpNoDelay());
    static ros::Publisher auto_hmi_publisher = node->advertise<hmi_agent_node::HMI_Signals>("/HMISignals", 1);
    (void)AutonomousHelper::getInstance();

    AutoBase* autoModePrg = nullptr;
    RobotState last_robot_state = RobotState::DISABLED;
    hmi_agent_node::HMI_Signals auto_hmi_signals;
    while( ros::ok() )
    {
        if(AutonomousHelper::getInstance().getRobotState() == RobotState::AUTONOMOUS && last_robot_state == RobotState::DISABLED && !autoModePrg)
        {
            traj_follow_active = false;
            traj_follow_complete = false;
            traj_id = -1;
            switch (selected_auto_mode)
            {
                case 0:
                {
                    AutonomousHelper::getInstance().initialize_position(1);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    // autoModePrg = new AutoMode1_5ball_Alt();
                    autoModePrg = new AutoMode5_5ball();
                    // autoModePrg = new AutoFollowPath();
                    break;
                }
                case 1:
                {
                    AutonomousHelper::getInstance().initialize_position(2);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    autoModePrg = new AutoMode2_2ball();
                    break;
                }
                case 2:
                {
                    AutonomousHelper::getInstance().initialize_position(3);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    autoModePrg = new AutoMode3_1ball();
                    break;
                }
                case 3:
                {
                    AutonomousHelper::getInstance().initialize_position(3);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    autoModePrg = new AutoMode4_1ball();
                    // autoModePrg = new AutoMode5_5ball();
                    break;
                }
                default:
                {
                    autoModePrg = nullptr;
                    break;
                }
            }
        }

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
            auto_hmi_signals = autoModePrg->stepStateMachine(traj_follow_active, traj_follow_complete, traj_id);
            auto_hmi_publisher.publish(auto_hmi_signals);
        }
        
        if (AutonomousHelper::getInstance().getRobotState() != RobotState::AUTONOMOUS && last_robot_state == RobotState::AUTONOMOUS)
        {
            ROS_ERROR("Force stopping trajectory!");
            AutonomousHelper::getInstance().stop_trajectory();
        }

        last_robot_state = AutonomousHelper::getInstance().getRobotState();
        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}

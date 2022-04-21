#include "AutonomousHelper.hpp"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <sstream>

#include <action_helper/action_helper.hpp>

#include <local_planner_node/PlanReq.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <robot_localization/SetPose.h>
#include "rio_control_node/Robot_Status.h"

#include <tf2/LinearMath/Quaternion.h>
#include "ck_utilities/CKMath.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <quesadilla_auto_node/Planner_Input.h>

static tf2_ros::TransformListener *tfListener;
static tf2_ros::Buffer tfBuffer;

void AutonomousHelper::initialize_position(int auto_id)
{
    static ros::ServiceClient position_reset_service = node->serviceClient<robot_localization::SetPose>("/set_pose");
    
    robot_localization::SetPose initial_pose;
    initial_pose.request.pose.header.stamp = ros::Time::now();
    // std::string alliance_color_str = alliance_color == AllianceColor::RED ? "red" : "blue";
    // std::stringstream ss_auto;
    // ss_auto << "auto_" << auto_id << "_" << alliance_color_str << "_link" << std::ends;
    // initial_pose.request.pose.header.frame_id = ss_auto.str();

    switch (auto_id)
    {
        case 1:
        {
            initial_pose.request.pose.header.frame_id = alliance_color == AllianceColor::RED ? "auto_1_red_link" : "auto_1_blue_link";
            break;
        }
        case 2:
        {
            initial_pose.request.pose.header.frame_id = alliance_color == AllianceColor::RED ? "auto_2_red_link" : "auto_2_blue_link";
            break;
        }
        case 3:
        {
            initial_pose.request.pose.header.frame_id = alliance_color == AllianceColor::RED ? "auto_3_red_link" : "auto_3_blue_link";
            break;
        }
        case 4:
        {
            initial_pose.request.pose.header.frame_id = alliance_color == AllianceColor::RED ? "auto_3_red_link" : "auto_3_blue_link";
            break;
        }
        default:
        {
            break;
        }
    }
    
    initial_pose.request.pose.pose.pose.position.x = 0;
    initial_pose.request.pose.pose.pose.position.y = 0;
    initial_pose.request.pose.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0,0,0);
    initial_pose.request.pose.pose.pose.orientation.w = q.getW();
    initial_pose.request.pose.pose.pose.orientation.x = q.getX();
    initial_pose.request.pose.pose.pose.orientation.y = q.getY();
    initial_pose.request.pose.pose.pose.orientation.z = q.getZ();

    initial_pose.request.pose.pose.covariance =
	   { 0.00001, 0.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.00001, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.00001, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.00001, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.00001, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0, 0.00001,};

    if(position_reset_service.call(initial_pose))
    {
        ROS_INFO ("Resetting position to %s", initial_pose.request.pose.header.frame_id.c_str());
    }
    else
    {
        ROS_ERROR("FAILED TO SET INITIAL POSITION COMING OUT OF DISABLED!");
    }
}

void AutonomousHelper::drive_trajectory(int trajectory_id)
{
    static ros::ServiceClient quesadilla_service = node->serviceClient<quesadilla_auto_node::Planner_Input>("quesadilla_planner_input");
    quesadilla_auto_node::Planner_Input msg;
    msg.request.begin_trajectory = true;
    msg.request.force_stop = false;
    msg.request.trajectory_id = trajectory_id;
    if (!quesadilla_service.call(msg))
    {
        ROS_ERROR("Failed to start trajectory");
    }
}

void AutonomousHelper::stop_trajectory()
{
    static ros::ServiceClient quesadilla_service = node->serviceClient<quesadilla_auto_node::Planner_Input>("quesadilla_planner_input");
    quesadilla_auto_node::Planner_Input msg;
    msg.request.begin_trajectory = false;
    msg.request.force_stop = true;
    msg.request.trajectory_id = -1;
    if (!quesadilla_service.call(msg))
    {
        ROS_ERROR("Failed to stop trajectory");
    }
}

void AutonomousHelper::setRobotState(RobotState robot_state)
{
    this->robot_state = robot_state;
}

void AutonomousHelper::setAllianceColor(AllianceColor alliance_color)
{
    this->alliance_color = alliance_color;
}

RobotState AutonomousHelper::getRobotState()
{
    return robot_state;
}

AllianceColor AutonomousHelper::getAllianceColor()
{
    return alliance_color;
}

AutonomousHelper::AutonomousHelper()
{
    tfListener = new tf2_ros::TransformListener(tfBuffer);
}
AutonomousHelper::~AutonomousHelper() {}

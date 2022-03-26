#include "AutonomousHelper.hpp"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

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

static tf2_ros::TransformListener *tfListener;
static tf2_ros::Buffer tfBuffer;

void AutonomousHelper::initialize_position()
{
    static ros::ServiceClient position_reset_service = node->serviceClient<robot_localization::SetPose>("/set_pose");
    
    robot_localization::SetPose initial_pose;
    initial_pose.request.pose.header.stamp = ros::Time::now();
    initial_pose.request.pose.header.frame_id = alliance_color == AllianceColor::RED ? "auto_1_red_link" : "auto_1_blue_link";
    
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

void AutonomousHelper::drive_trajectory_points(std::vector<std::pair<std::string, double>>& points)
{
    state = STATE::IDLE;
    timer = 0;

    local_planner_node::PlanReq req;
    req.request.plan = std::vector<geometry_msgs::PoseStamped>();
    req.request.frame = local_planner_node::PlanReq::Request::FRAME_BASE;
    
    if (points.size() <= 0)
    {
        return;
    }

    std::string firstPointLinkStr = points[0].first;

    for (const std::pair<std::string, double>& p : points)
    {
        // tf2::Stamped<tf2::Transform> point_to_first_point;
        // tf2::convert(tfBuffer.lookupTransform(firstPointLinkStr, p.first, ros::Time(0)), point_to_first_point);

        geometry_msgs::PoseStamped point;
        point.header.stamp = ros::Time::now();
        point.header.frame_id = p.first;
        point.pose.position.x = 0;
        point.pose.position.y = 0;
        point.pose.position.z = 0;
        // point.pose.position.x = point_to_first_point.getOrigin().getX();
        // point.pose.position.y = point_to_first_point.getOrigin().getY();
        // point.pose.position.z = point_to_first_point.getOrigin().getZ();
        // ROS_INFO("Point %s x, y, z: %lf, %lf, %lf", p.first.c_str(), point.pose.position.x, point.pose.position.y, point.pose.position.z);


        //TODO: Align transforms using quaternion from point_to_first_point
        tf2::Quaternion Up;
        Up.setRPY(0,0,ck::math::deg2rad(p.second));
        point.pose.orientation.w = Up.getW();
        point.pose.orientation.x = Up.getX();
        point.pose.orientation.y = Up.getY();
        point.pose.orientation.z = Up.getZ();

        req.request.plan.push_back(point);
    }

    ros::ServiceClient& lPlanReq = getLocalPlannerReqService();
    if (lPlanReq)
    {
        lPlanReq.call( req );
    }
}

void AutonomousHelper::move_to_position(std::string position, double yaw_rotation_deg)
{
    std::vector<std::pair<std::string, double>> v {{position, yaw_rotation_deg}};
    drive_trajectory_points(v);
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
    (void)getLocalPlannerReqService();
    tfListener = new tf2_ros::TransformListener(tfBuffer);
}
AutonomousHelper::~AutonomousHelper() {}

ros::ServiceClient& AutonomousHelper::getLocalPlannerReqService()
{
    if (!local_planner_req_client)
	{
		local_planner_req_client = node->serviceClient<local_planner_node::PlanReq>("/local_plan_request", true);
	}
	return local_planner_req_client;
}
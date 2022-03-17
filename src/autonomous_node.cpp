#include "ros/ros.h"
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

#define RATE (100)

ros::NodeHandle* node;

enum RobotState
{
    DISABLED = 0,
    TELEOP = 1,
    AUTONOMOUS = 2,
    TEST = 3,
};

static RobotState robot_state;

// ActionHelper* action_helper;

// ros::ServiceClient local_planner_req_client;


// enum STATE
// {
//     IDLE,
//     MOVING_FORWARD
// };

// enum STATE state;
// int timer = 0;

// void move_forward()
// {
//     std::cout << "move fprward\n";
//     state = STATE::IDLE;
//     timer = 0;

//     local_planner_node::PlanReq req;
//     req.request.plan = std::vector<geometry_msgs::PoseStamped>();
//     req.request.frame = local_planner_node::PlanReq::Request::FRAME_BASE;

//     geometry_msgs::PoseStamped point;
//     point.pose.position.x = 1.0;
//     point.pose.position.y = 0.0;
//     point.pose.position.z = 0.0;

//     tf2::Quaternion Up;
//     Up.setRPY(0,0,0);
//     point.pose.orientation.w = Up.getW();
//     point.pose.orientation.x = Up.getX();
//     point.pose.orientation.y = Up.getY();
//     point.pose.orientation.z = Up.getZ();

//     req.request.plan.push_back(point);

//     local_planner_req_client.call( req );

// }

void initialize_position()
{
    static ros::ServiceClient position_reset_service = node->serviceClient<robot_localization::SetPose>("/set_pose");
    
    robot_localization::SetPose initial_pose;
    initial_pose.request.pose.header.stamp = ros::Time::now();
    initial_pose.request.pose.header.frame_id = "auto_1_red_link";
    
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

void robot_status_callback (const rio_control_node::Robot_Status &msg)
{
    robot_state = (RobotState) msg.robot_state;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "autonomous_node");
	ros::NodeHandle n;
    ros::Rate rate(RATE);
	node = &n;

    static ros::Subscriber robot_status_subscriber = node->subscribe("/RobotStatus", 1, robot_status_callback);

    // // listen to move actions
    // action_helper = new ActionHelper(node);
    // local_planner_req_client =  node->serviceClient<local_planner_node::PlanReq>("/local_plan_request");

    // state = STATE::IDLE;

    while( ros::ok() )
    {
        static RobotState last_robot_state = RobotState::DISABLED;
        if(robot_state == RobotState::AUTONOMOUS && last_robot_state == RobotState::DISABLED)
        {
            initialize_position();
        }
        last_robot_state = robot_state;

        //action_helper->step();

        // if( state == STATE::MOVING_FORWARD )
        // {
        //     timer += 1;

        //     if( timer > 500 )
        //     {
        //         action_helper->update_action( "MoveForward",
        //                                       ActionHelper::ACTION_STATUS::COMPLETE );
        //        state = STATE::IDLE;
        //     }
        // }
        // else
        // {
        //     if( action_helper->check_action( "MoveForward" ) )
        //     {
        //         action_helper->update_action( "MoveForward",
        //                                       ActionHelper::ACTION_STATUS::EXECUTING );
        //         move_forward();
        //         state = STATE::MOVING_FORWARD;
        //     }
        // }

        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include <action_helper/action_helper.hpp>

#include <local_planner_node/PlanReq.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/LinearMath/Quaternion.h>

#define RATE (100)

ros::NodeHandle* node;
ActionHelper* action_helper;

ros::ServiceClient local_planner_req_client;


enum STATE
{
    IDLE,
    MOVING_FORWARD
};

enum STATE state;
int timer = 0;

void move_forward()
{
    std::cout << "move fprward\n";
    state = STATE::IDLE;
    timer = 0;

    local_planner_node::PlanReq req;
    req.request.plan = std::vector<geometry_msgs::PoseStamped>();
    req.request.frame = local_planner_node::PlanReq::Request::FRAME_BASE;

    geometry_msgs::PoseStamped point;
    point.pose.position.x = 1.0;
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;

    tf2::Quaternion Up;
    Up.setRPY(0,0,0);
    point.pose.orientation.w = Up.getW();
    point.pose.orientation.x = Up.getX();
    point.pose.orientation.y = Up.getY();
    point.pose.orientation.z = Up.getZ();

    req.request.plan.push_back(point);

    local_planner_req_client.call( req );

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "autonomous_node");
	ros::NodeHandle n;
    ros::Rate rate(RATE);
	node = &n;

    // listen to move actions
    action_helper = new ActionHelper(node);
    local_planner_req_client =  node->serviceClient<local_planner_node::PlanReq>("/local_plan_request");

    state = STATE::IDLE;

    while( ros::ok() )
    {
        //action_helper->step();

        if( state == STATE::MOVING_FORWARD )
        {
            timer += 1;

            if( timer > 500 )
            {
                action_helper->update_action( "MoveForward",
                                              ActionHelper::ACTION_STATUS::COMPLETE );
               state = STATE::IDLE;
            }
        }
        else
        {
            if( action_helper->check_action( "MoveForward" ) )
            {
                action_helper->update_action( "MoveForward",
                                              ActionHelper::ACTION_STATUS::EXECUTING );
                move_forward();
                state = STATE::MOVING_FORWARD;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}

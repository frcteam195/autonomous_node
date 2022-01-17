#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include <../../action_processor_node/include/helper/action_helper.hpp>

#define RATE (100)

ros::NodeHandle* node;
ActionHelper* action_helper;

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
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "autonomous_node");
	ros::NodeHandle n;
    ros::Rate rate(RATE);
	node = &n;

    // listen to move actions
    action_helper = new ActionHelper(node);

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
               std::cout << "asdasdasd\n\n";
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

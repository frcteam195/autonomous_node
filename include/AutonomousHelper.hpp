#pragma once
#include "ck_utilities/Singleton.hpp"
#include <atomic>
#include "ros/ros.h"
#include <local_planner_node/PlanReq.h>

extern ros::NodeHandle* node;

enum RobotState : int
{
    DISABLED = 0,
    TELEOP = 1,
    AUTONOMOUS = 2,
    TEST = 3,
};

enum AllianceColor : int
{
    RED = 0,
    BLUE = 1,
};

enum STATE : int
{
    IDLE,
    MOVING_FORWARD
};


class AutonomousHelper : public Singleton<AutonomousHelper>
{
    friend Singleton;
public:
    void drive_trajectory_points(std::vector<std::pair<std::string, double>>& points);
    void move_to_position(std::string position, double yaw_rotation_deg = 0);
    void initialize_position();

    void setRobotState(RobotState robot_state);
    void setAllianceColor(AllianceColor alliance_color);
    RobotState getRobotState();
    AllianceColor getAllianceColor();


private:
    ros::ServiceClient local_planner_req_client;
    std::atomic<RobotState> robot_state {RobotState::DISABLED};
    std::atomic<AllianceColor> alliance_color {AllianceColor::RED};
    //ActionHelper* action_helper = nullptr;
    std::atomic<STATE> state {STATE::IDLE};
    ros::ServiceClient& getLocalPlannerReqService();
    int timer = 0;

    AutonomousHelper();
    ~AutonomousHelper();
};
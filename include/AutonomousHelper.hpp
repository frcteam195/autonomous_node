#pragma once
#include "ck_utilities/Singleton.hpp"
#include <atomic>
#include "ros/ros.h"

extern ros::NodeHandle* node;

enum StartPosition : int
{
    POS_1,  // x: 297.98, y: -253.32, theta: -88.5
    POS_2,  // x: 238.72, y:  120.23, theta:  136.5
    POS_3,  // x: 268.00, y: -220.00, theta: -156.0
    POS_4,  // x: 238.69, y: -173.29, theta: -156.0
    POS_5,  // x: 346.78, y: -252.07, theta: -88.5
};

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
    void drive_trajectory(int trajectory_id);
    void stop_trajectory();
    void initialize_position(StartPosition start_pos);

    void setRobotState(RobotState robot_state);
    void setAllianceColor(AllianceColor alliance_color);
    RobotState getRobotState();
    AllianceColor getAllianceColor();


private:
    std::atomic<RobotState> robot_state {RobotState::DISABLED};
    std::atomic<AllianceColor> alliance_color {AllianceColor::RED};
    std::atomic<STATE> state {STATE::IDLE};
    int timer = 0;

    AutonomousHelper();
    ~AutonomousHelper();
};
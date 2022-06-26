#pragma once
#include "ck_utilities/Singleton.hpp"
#include <atomic>
#include "ros/ros.h"

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
    void drive_trajectory(int trajectory_id);
    void stop_trajectory();
    void initialize_position(int auto_id);

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
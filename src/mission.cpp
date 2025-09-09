#include "mission.h"

Mission::Mission() : Node("navigation")
{
    // Publisher to /mission/goals
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/mission/goals", 10);

    // Wait 200 ms, then call sendGoals()
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&Mission::sendGoals, this));
}

void Mission::sendGoals()
{
    if (goal_pub_->get_subscription_count() == 0) {
        std::cout<<"Waiting for subscribers..."<<std::endl;
        return; 
    }

    // Example goals
    geometry_msgs::msg::Pose p1;
    p1.position.x = 3.0;
    p1.position.y = 2.0;
    p1.position.z = 2.0;

    geometry_msgs::msg::Pose p2;
    p2.position.x = 0.0;
    p2.position.y = 0.0;
    p2.position.z = 2.0;

    goal_array.poses.push_back(p1);
    goal_array.poses.push_back(p2);

    goal_pub_->publish(goal_array);
    std::cout<<"NAV GOALS SET"<<std::endl;

    timer_->cancel();
}
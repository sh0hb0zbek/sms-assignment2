#include "actionlib/server/simple_action_server.h"
#include "assignment2/Navigate2DAction.h"
#include "std_msgs/Float32.h"
#include <cmath>

typedef actionlib::SimpleActionServer<assignment2::Navigate2DAction> NavServer;


ros::Publisher goal_publisher;
float DIST_THRESHOLD, SPEED;

geometry_msgs::Point robot_current_position;

void navCallback(const assignment2::Navigate2DGoal::ConstPtr& goal_point,
                 NavServer* nav_server)
{
    ros::NodeHandle nodehandle, nodehandle_goal;
    assignment2::Navigate2DFeedback feedback_msg;
    assignment2::Navigate2DResult   result_msg;

    goal_publisher = nodehandle_goal.advertise<std_msgs::Float32>("goal", 10);
    ros::Rate goal_rate(10);
    std_msgs::Float32 msg;
    msg.data = goal_point->Point.x;
    goal_publisher.publish(msg);
    goal_rate.sleep();

    ros::Rate feedback_rate(2);

    double goal_received_time = ros::Time::now().toSec();
    ROS_INFO("Goal Recevied");

    float dist = abs(robot_current_position.x - goal_point->Point.x);

    if (nodehandle.getParam("dist_threshold", DIST_THRESHOLD))
    {
        if (nodehandle.getParam("speed", SPEED))
        {
            while (dist > DIST_THRESHOLD)
            {
                dist = abs(robot_current_position.x - goal_point->Point.x);

                feedback_msg.distance_to_point = dist;

                nav_server->publishFeedback(feedback_msg);

                if (robot_current_position.x < goal_point->Point.x) robot_current_position.x = robot_current_position.x + SPEED;
                else robot_current_position.x = robot_current_position.x - SPEED;
                
                feedback_rate.sleep();
            }
        }
        else std::cout << "\n[ERROR] speed is not set!" << std::endl;
    }
    else std::cout << "\n[ERROR] dist_threshold is not set!" << std::endl;

    double goal_reached_time = ros::Time::now().toSec();
    ROS_INFO("Goal Reached");

    double elapsed_time = goal_reached_time - goal_received_time;

    result_msg.elapsed_time = elapsed_time;

    nav_server->setSucceeded(result_msg);
}

void subscriberCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    robot_current_position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigate_2d_server_node");

    ros::NodeHandle node_handle;

    ros::Subscriber robot_position = node_handle.subscribe("robot_position", 10, subscriberCallback);

    NavServer server(node_handle, "navigate_2d", boost::bind(&navCallback, _1, &server), false);
    server.start();

    ros::spin();

    return 0;
}
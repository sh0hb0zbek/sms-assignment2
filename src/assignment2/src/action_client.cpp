#include "actionlib/client/simple_action_client.h"
#include "assignment2/Navigate2DAction.h"
#include "std_msgs/Float32.h"

typedef actionlib::SimpleActionClient<assignment2::Navigate2DAction> NavClient;

ros::Publisher feedback_pub, result_pub;

bool feedback_cb_ready_flag = false;

void activeCallback()
{
    std::cout << "\nGoal Activated" << std::endl;
    feedback_cb_ready_flag = true;
}

void feedbackCallback(const assignment2::Navigate2DFeedback::ConstPtr& feedback)
{
    ros::NodeHandle feedback_node;

    feedback_pub = feedback_node.advertise<std_msgs::Float32>("feedback", 32);
    
    if (feedback_cb_ready_flag)
    {
        std_msgs::Float32 msg;
        msg.data = feedback->distance_to_point;
        feedback_pub.publish(msg);

        std::cout << "\33[3K\r";
        std::cout << "Distance to Goal: " << feedback->distance_to_point;
        std::cout.flush();
    }
}

void resultCallback(const actionlib::SimpleClientGoalState& state,
                    const assignment2::Navigate2DResult::ConstPtr& result)
{
    ros::NodeHandle result_node;
    
    result_pub = result_node.advertise<std_msgs::Float32>("result", 32);
    std_msgs::Float32 msg;
    msg.data = result->elapsed_time;
    result_pub.publish(msg);

    std::cout << "\n\nFinished.\nTime Elapsed: " << result->elapsed_time << std::endl;
    feedback_cb_ready_flag = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigate_2d_client_node");

    NavClient client("navigate_2d", true);
    client.waitForServer();

    assignment2::Navigate2DGoal goal;
    goal.Point.z = 0;
    goal.Point.y = 0;

    while (ros::ok())
    {
        std::cout << "\nEnter the X-Coordinate: "; std::cin >> goal.Point.x;

        client.sendGoal(goal, &resultCallback, &activeCallback, &feedbackCallback);

        client.waitForResult();
    }

    return 0;
}
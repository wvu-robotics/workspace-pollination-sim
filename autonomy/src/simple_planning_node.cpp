#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

//#define AER_222
//#define AER_LOBBY
#define GREENHOUSE

#ifdef AER_222
#define NUM_GOALS 8
#endif // AER_222

#ifdef AER_LOBBY
#define NUM_GOALS 9
#endif // AER_LOBBY

#ifdef GREENHOUSE
#define NUM_GOALS 3
#endif // GREENHOUSE

struct robotPose
{
    double x; // m
    double y; // m
    double yaw; // rad
};

bool actionDone = false;

void setPoseGoal(move_base_msgs::MoveBaseGoal& poseGoal, double x, double y, double yaw); // m, m, rad
void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    actionDone = true;
    ROS_INFO("goal done");
}
void activeCallback()
{
    ROS_INFO("goal went active");
}
void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO("got feedback");
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

int main(int argc, char** argv)
{
    enum STATE_T {_init, _exec, _finish} state = _init;
    unsigned int goalNum = 0;
    robotPose goalSequence[NUM_GOALS];
#ifdef AER_222
    goalSequence[0] = robotPose{-0.131,-1.807,DEG2RAD*-90.0};
    goalSequence[1] = robotPose{-0.368,-3.699,DEG2RAD*-180.0};
    goalSequence[2] = robotPose{-2.997,-3.575,DEG2RAD*90.0};
    goalSequence[3] = robotPose{-2.706,-0.448,DEG2RAD*90.0};
    goalSequence[4] = robotPose{-0.476,0.962,DEG2RAD*90.0};
    goalSequence[5] = robotPose{-0.418,3.340,DEG2RAD*180.0};
    goalSequence[6] = robotPose{-2.971,3.203,DEG2RAD*-90.0};
    goalSequence[7] = robotPose{-2.499,-0.375,DEG2RAD*0.0};
#endif // AER_222
#ifdef AER_LOBBY
    goalSequence[0] = robotPose{1.897,-2.892,DEG2RAD*-90.0};
    goalSequence[1] = robotPose{1.954,-6.953,DEG2RAD*0.0};
    goalSequence[2] = robotPose{9.649,-7.007,DEG2RAD*0.0};
    goalSequence[3] = robotPose{3.772,-1.413,DEG2RAD*90.0};
    goalSequence[4] = robotPose{11.181,-0.914,DEG2RAD*-90.0};
    goalSequence[5] = robotPose{10.248,-6.956,DEG2RAD*180.0};
    goalSequence[6] = robotPose{4.475,-1.758,DEG2RAD*90.0};
    goalSequence[7] = robotPose{-8.371,-0.096,DEG2RAD*180.0};
    goalSequence[8] = robotPose{2.856,-1.248,DEG2RAD*0.0};
    
#endif // AER_LOBBY
#ifdef GREENHOUSE
    goalSequence[0] = robotPose{6.120,2.653,0.0*DEG2RAD};
    goalSequence[1] = robotPose{6.120,4.384,90.0*DEG2RAD};
    goalSequence[2] = robotPose{1.306,4.384,180.0*DEG2RAD};
#endif // GREENHOUSE
    move_base_msgs::MoveBaseGoal goal;
    ros::init(argc, argv, "simple_planning_node");
    ros::NodeHandle nh;
    ros::Rate loopRate(20);
    ROS_INFO("simple_planning_node starting...");
    Client client("move_base");
    ros::spinOnce();
    client.waitForServer();
    ROS_INFO("connected to move_base server");
    while(ros::ok())
    {
        switch(state)
        {
        case _init:
            ROS_INFO("sending goal %u",goalNum);
            setPoseGoal(goal,goalSequence[goalNum].x,goalSequence[goalNum].y,goalSequence[goalNum].yaw);
            client.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);
            client.waitForResult(ros::Duration(0.25));
            state = _exec;
            break;
        case _exec:
            if(actionDone)
            {
                actionDone = false;
                ROS_INFO("goal %u reached",goalNum);
                state = _finish;
            }
            else
            {
                ROS_INFO_THROTTLE(1,"goal %u not yet reached",goalNum);
                state = _exec;
            }
            break;
        case _finish:
            goalNum++;
            if(goalNum>=NUM_GOALS)
            {
                goalNum = 0;
            }
            state = _init;
            break;
        }
        ros::spinOnce();
        loopRate.sleep();
    }
    ROS_INFO("simple_planning_node ending...");
    return 0;
}

void setPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
{
    const double pitch = 0.0;
    const double roll = 0.0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    poseGoal.target_pose.header.frame_id = "greenhouse";
    poseGoal.target_pose.pose.position.x = x;
    poseGoal.target_pose.pose.position.y = y;
    poseGoal.target_pose.pose.position.z = 0.0;
    poseGoal.target_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    poseGoal.target_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    poseGoal.target_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    poseGoal.target_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;
}

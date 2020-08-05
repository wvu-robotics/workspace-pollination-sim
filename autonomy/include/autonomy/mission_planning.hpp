#ifndef MISSION_PLANNING_HPP
#define MISSION_PLANNING_HPP
#include <ros/ros.h>
#include <autonomy/plant_row_map.hpp>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <blackfly_s/BlackflyOut.h>
#include <std_msgs/Bool.h>
#include <manipulation_common/FlowerMap.h>
//#include <aruco_markers/StartPollination.h>

class MissionPlanning
{
public:
    // Types
    enum MISSION_STATE_T {_surveyChooseWaypoint, _surveyDrive, _surveyCheckFinished, _pollinationChooseWaypoint, _pollinationBlindTurnToDrive,
                          _pollinationDrive, _pollinationBlindTurnToPlant, _pollinationBlindForward, _pollinationArm, _pollinationBlindBack,
                          _pollinationCheckFinished};
    enum STATE_T {_init, _exec, _finish};
    // Methods
    MissionPlanning();
    void run();
    void setPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, geometry_msgs::Pose2D poseInput); // m, m, rad
    void driveDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    void driveActiveCallback();
    void driveFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void detectedFlowersCallback(const manipulation_common::FlowerMap::ConstPtr& msg);
    void armCallback(const  std_msgs::Bool::ConstPtr& msg);
    unsigned int findNumUnpollinatedCells();
    // Members
    ros::NodeHandle nh;
    ros::Rate loopRate;
    ros::Subscriber poseSub;
    ros::Subscriber detectedFlowersSub;
    ros::Subscriber armSub;
    ros::Publisher blindDriveTwistPub;
    ros::Publisher armPub;
    bool continueRunning;
    PlantRowMap plantRowMap;
    MISSION_STATE_T state;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;
    move_base_msgs::MoveBaseGoal driveGoal;
    std::vector<geometry_msgs::Pose2D> surveyWaypoints;
    geometry_msgs::Twist blindDriveTwist;
    geometry_msgs::Pose2D pollinationGoalPose;
    std_msgs::Bool armMsg;
    float pollinationBlindTurnToDriveHeadingGoal;
    float pollinationBlindTurnToPlantHeadingGoal;
    unsigned int surveyWaypointIndex;
    bool driveActionDone;
    unsigned int numSurveyWaypoints;
    bool minPollinationCostFirstPass;
    double candidateCost;
    double bestPollinationCost;
    unsigned int bestPollinationCellIndex;
    geometry_msgs::Pose2D robotPose;
    bool pollinationArmManeuversComplete;
    bool performingSurveyPass;
    double driveActionStartTime;
    double currentTime;
    geometry_msgs::Pose2D blindDriveInitPose;
    geometry_msgs::Quaternion blindRotateToDriveGoalQ;
    geometry_msgs::Quaternion blindRotateToPlantGoalQ;
    geometry_msgs::Quaternion robotPoseCurrentQ;
    float blindTurnSpeedSign;
    float blindTurnCrossProduct;
    const double numFlowersCostGain = 0.5;
    const double distanceCostGain = 0.5;
    const float maxFlowerDetectDistance = 1.0; // m
    const double driveActionTimeout = 20.0; // sec
    const double blindDriveTime = 1.0; // sec
    const float blindDriveSpeed = 0.2; // m/s
    const float blindTurnSpeed = 0.3; // rad/s
    const float blindDriveDistance = 0.1; // m
};

#endif // MISSION_PLANNING_HPP

#include <autonomy/mission_planning.hpp>

MissionPlanning::MissionPlanning()
    : moveBaseClient("move_base"), loopRate(50)
{
    ros::spinOnce();
    this->moveBaseClient.waitForServer();
    this->state = _surveyChooseWaypoint;
    this->surveyWaypointIndex = 0;
    this->driveActionDone = false;
    this->continueRunning = true;
    this->performingSurveyPass = true;
    this->poseSub = nh.subscribe<nav_msgs::Odometry>("/nav_filter/nav_filter/states",1,&MissionPlanning::poseCallback,this);
    this->blindDriveTwistPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    this->detectedFlowersSub = nh.subscribe<manipulation_common::FlowerMap>( "flower_map",1,&MissionPlanning::detectedFlowersCallback,this);
    this->armSub = nh.subscribe<std_msgs::Bool>("pollination_procedure_ended",1,&MissionPlanning::armCallback,this);
    this->armPub = nh.advertise<std_msgs::Bool>("start_pollination_procedures",1);
    this->numSurveyWaypoints = 3;
    this->surveyWaypoints.resize(this->numSurveyWaypoints);
    this->surveyWaypoints.at(0).x = 1.25;//6.120;
    this->surveyWaypoints.at(0).y = 10;//2.653;
    this->surveyWaypoints.at(0).theta = 0.0*DEG2RAD;
    this->surveyWaypoints.at(1).x = .5;//6.120;
    this->surveyWaypoints.at(1).y = 2.61;//4.384;
    this->surveyWaypoints.at(1).theta = 90.0*DEG2RAD;
    this->surveyWaypoints.at(2).x = 1.306;
    this->surveyWaypoints.at(2).y = 4.384;
    this->surveyWaypoints.at(2).theta = 180.0*DEG2RAD;
    /*this->surveyWaypoints.at(3).x = 1.306;
    this->surveyWaypoints.at(3).y = 6.477;
    this->surveyWaypoints.at(3).theta = 90.0*DEG2RAD;
    this->surveyWaypoints.at(4).x = 6.220;
    this->surveyWaypoints.at(4).y = 6.477;
    this->surveyWaypoints.at(4).theta = 0.0*DEG2RAD;*/

    // TEMPORARY!!!!!!!
    // artificially populate grid blocks with fake info assuming survey passed found flowers
    //this->plantRowMap.plantRowsBlocks.at(0).numFlowers = 90;
    this->plantRowMap.plantRowsBlocks.at(1).numFlowers = 50;
    this->plantRowMap.plantRowsBlocks.at(2).numFlowers = 10;
    //this->plantRowMap.plantRowsBlocks.at(8).numFlowers = 1;
    //this->plantRowMap.plantRowsBlocks.at(9).numFlowers = 6;
    // !!!!!!!!!!!!!!!!
}

void MissionPlanning::run()
{
            /*auto f1  = [this](const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& ptr){
                ROS_INFO("driveDoneCallback");
                driveDoneCallback(state, ptr);
            };
            auto f2  = [this](){
                ROS_INFO("driveActiveCallback");
                driveActiveCallback();
            };
            auto f3  = [this](const move_base_msgs::MoveBaseFeedbackConstPtr& ptr){
                ROS_INFO("driveFeedbackCallback");
                driveFeedbackCallback(ptr);
            };*/
    while(ros::ok() && continueRunning)
    {
        this->currentTime = ros::Time::now().toSec();
        switch(this->state)
        {
        case _surveyChooseWaypoint:
            ROS_INFO("sending survey drive waypoint %u",this->surveyWaypointIndex);
            setPoseGoal(this->driveGoal, this->surveyWaypoints.at(this->surveyWaypointIndex));
            this->driveActionStartTime = ros::Time::now().toSec();
            moveBaseClient.sendGoal(this->driveGoal,
                                    boost::bind(&MissionPlanning::driveDoneCallback, this, _1, _2),
                                    boost::bind(&MissionPlanning::driveActiveCallback, this),
                                    boost::bind(&MissionPlanning::driveFeedbackCallback, this, _1));
            this->state = _surveyDrive;
            break;
        case _surveyDrive:
            if(this->driveActionDone)
            {
                ROS_INFO("survey drive waypoint %u reached",this->surveyWaypointIndex);
                this->driveActionDone = false;
                this->surveyWaypointIndex++;
                this->state = _surveyCheckFinished;
            }
            else if((this->currentTime - this->driveActionStartTime) > this->driveActionTimeout)
            {
                ROS_WARN("survey drive maneuver timed out");
                moveBaseClient.cancelAllGoals();
                this->state = _surveyChooseWaypoint;
            }
            else
            {
                ROS_INFO_THROTTLE(1,"survey drive waypoint %u not yet reached",this->surveyWaypointIndex);
                this->state = _surveyDrive;
            }
            break;
        case _surveyCheckFinished:
            if(this->surveyWaypointIndex >= this->numSurveyWaypoints)
            {
                ROS_INFO("survey sequence complete");
                if(findNumUnpollinatedCells() > 0)
                {
                    this->state = _pollinationChooseWaypoint;
                    this->performingSurveyPass = false;
                }
                else
                {
                    this->continueRunning = false;
                    ROS_ERROR("no grid cells with flowers to pollinate found, terminating");
                }
            }
            else
            {
                this->state = _surveyChooseWaypoint;
            }
            break;
        case _pollinationChooseWaypoint:
            this->minPollinationCostFirstPass = true;
            for(int i=0; i < this->plantRowMap.plantRowsBlocks.size(); i++)
            {
                this->candidateCost = this->numFlowersCostGain/((double)this->plantRowMap.plantRowsBlocks.at(i).numFlowers)
                        + this->distanceCostGain*hypot(this->plantRowMap.plantRowsBlocks.at(i).pollinationPose.x - this->robotPose.x,
                                                       this->plantRowMap.plantRowsBlocks.at(i).pollinationPose.y - this->robotPose.y);
                if((this->minPollinationCostFirstPass || this->candidateCost < this->bestPollinationCost) && !this->plantRowMap.plantRowsBlocks.at(i).pollinationComplete)
                {
                    this->bestPollinationCost = this->candidateCost;
                    this->bestPollinationCellIndex = i;
                    this->minPollinationCostFirstPass = false;
                }
            }
            ROS_INFO("sending pollination drive waypoint, cell index %u",this->bestPollinationCellIndex);
            ROS_INFO("numFlowers = %u, goalPose = [%f, %f, %f]",this->plantRowMap.plantRowsBlocks.at(this->bestPollinationCellIndex).numFlowers,
                     this->plantRowMap.plantRowsBlocks.at(this->bestPollinationCellIndex).pollinationPose.x,
                     this->plantRowMap.plantRowsBlocks.at(this->bestPollinationCellIndex).pollinationPose.y,
                     this->plantRowMap.plantRowsBlocks.at(this->bestPollinationCellIndex).pollinationPose.theta*RAD2DEG);
            this->pollinationGoalPose = this->plantRowMap.plantRowsBlocks.at(this->bestPollinationCellIndex).pollinationPose;
            this->pollinationBlindTurnToPlantHeadingGoal = this->plantRowMap.plantRowsBlocks.at(this->bestPollinationCellIndex).pollinationPose.theta;
            this->blindRotateToPlantGoalQ.z = sin(0.5*this->pollinationBlindTurnToPlantHeadingGoal);
            this->blindRotateToPlantGoalQ.w = cos(0.5*this->pollinationBlindTurnToPlantHeadingGoal);
            if((this->robotPose.x - this->pollinationGoalPose.x) > 0.0)
            {
                if(fabs(this->robotPose.y - this->pollinationGoalPose.y) < 1.0) // Traveling within same row
                {
                    this->pollinationGoalPose.theta = 180.0*DEG2RAD;
                    this->pollinationBlindTurnToDriveHeadingGoal = 180.0*DEG2RAD;
                }
                else // Traveling to different row
                {
                    this->pollinationGoalPose.theta = 180.0*DEG2RAD;
                    this->pollinationBlindTurnToDriveHeadingGoal = 0.0*DEG2RAD;
                }
            }
            else
            {
                if(fabs(this->robotPose.y - this->pollinationGoalPose.y) < 1.0) // Traveling within same row
                {
                    this->pollinationGoalPose.theta = 0.0*DEG2RAD;
                    this->pollinationBlindTurnToDriveHeadingGoal = 0.0*DEG2RAD;
                }
                else // Traveling to different row
                {
                    this->pollinationGoalPose.theta = 0.0*DEG2RAD;
                    this->pollinationBlindTurnToDriveHeadingGoal = 180.0*DEG2RAD;
                }
            }
            this->blindRotateToDriveGoalQ.z = sin(0.5*this->pollinationBlindTurnToDriveHeadingGoal);
            this->blindRotateToDriveGoalQ.w = cos(0.5*this->pollinationBlindTurnToDriveHeadingGoal);
            this->driveActionStartTime = ros::Time::now().toSec();
            setPoseGoal(this->driveGoal, this->pollinationGoalPose);
            this->state = _pollinationBlindTurnToDrive;
            break;
        case _pollinationBlindTurnToDrive:
            if(fabs(this->blindRotateToDriveGoalQ.z - this->robotPoseCurrentQ.z) < 0.01 && fabs(this->blindRotateToDriveGoalQ.w - this->robotPoseCurrentQ.w) < 0.01)
            {
                ROS_INFO("pollination blind rotate to drive complete");
                this->blindDriveTwist.linear.x = 0.0;
                this->blindDriveTwist.angular.z = 0.0;
                this->blindDriveTwistPub.publish(this->blindDriveTwist);
                this->driveActionStartTime = ros::Time::now().toSec();
                this->blindDriveInitPose = this->robotPose;
                moveBaseClient.sendGoal(this->driveGoal,
                                              boost::bind(&MissionPlanning::driveDoneCallback, this, _1, _2),
                                              boost::bind(&MissionPlanning::driveActiveCallback, this),
                                              boost::bind(&MissionPlanning::driveFeedbackCallback, this, _1));
                moveBaseClient.waitForResult(ros::Duration(0.25));
                this->state = _pollinationDrive;
            }
            else
            {
                ROS_INFO_THROTTLE(1,"pollination blind rotate to drive executing");
                this->blindTurnCrossProduct = cos(this->robotPose.theta)*sin(this->pollinationBlindTurnToDriveHeadingGoal) -
                        cos(this->pollinationBlindTurnToDriveHeadingGoal)*sin(this->robotPose.theta);
                if(this->blindTurnCrossProduct > 1.0) this->blindTurnCrossProduct = 1.0;
                else if(this->blindTurnCrossProduct < -1.0) this->blindTurnCrossProduct = -1.0;
                if(asin(this->blindTurnCrossProduct) >= 0.0) this->blindTurnSpeedSign = 1.0;
                else this->blindTurnSpeedSign = -1.0;
                this->blindDriveTwist.linear.x = 0.0;
                this->blindDriveTwist.angular.z = this->blindTurnSpeed*this->blindTurnSpeedSign;
                this->blindDriveTwistPub.publish(this->blindDriveTwist);
                this->state = _pollinationBlindTurnToDrive;
            }
            break;
        case _pollinationDrive:
            if(this->driveActionDone)
            {
                ROS_INFO("pollination drive waypoint %u reached",this->surveyWaypointIndex);
                this->driveActionDone = false;
                this->surveyWaypointIndex++;
                this->driveActionStartTime = ros::Time::now().toSec();
                this->state = _pollinationBlindTurnToPlant;
            }
            else if((this->currentTime - this->driveActionStartTime) > this->driveActionTimeout)
            {
                ROS_WARN("pollination drive maneuver timed out");
                moveBaseClient.cancelAllGoals();
                this->state = _pollinationChooseWaypoint;
            }
            else
            {
                ROS_INFO_THROTTLE(1,"pollination drive waypoint %u not yet reached",this->surveyWaypointIndex);
                this->state = _pollinationDrive;
            }
            break;
        case _pollinationBlindTurnToPlant:
            if(fabs(this->blindRotateToPlantGoalQ.z - this->robotPoseCurrentQ.z) < 0.01 && fabs(this->blindRotateToPlantGoalQ.w - this->robotPoseCurrentQ.w) < 0.01)
            {
                ROS_INFO("pollination blind rotate to plant complete");
                this->blindDriveTwist.linear.x = 0.0;
                this->blindDriveTwist.angular.z = 0.0;
                this->blindDriveTwistPub.publish(this->blindDriveTwist);
                this->driveActionStartTime = ros::Time::now().toSec();
                this->blindDriveInitPose = this->robotPose;
                this->state = _pollinationBlindForward;
            }
            else
            {
                ROS_INFO_THROTTLE(1,"pollination blind rotate to plant executing");
                this->blindTurnCrossProduct = cos(this->robotPose.theta)*sin(this->pollinationBlindTurnToPlantHeadingGoal) -
                        cos(this->pollinationBlindTurnToPlantHeadingGoal)*sin(this->robotPose.theta);
                if(this->blindTurnCrossProduct > 1.0) this->blindTurnCrossProduct = 1.0;
                else if(this->blindTurnCrossProduct < -1.0) this->blindTurnCrossProduct = -1.0;
                if(asin(this->blindTurnCrossProduct) >= 0.0) this->blindTurnSpeedSign = 1.0;
                this->blindDriveTwist.linear.x = 0.0;
                this->blindDriveTwist.angular.z = this->blindTurnSpeed*this->blindTurnSpeedSign;
                this->blindDriveTwistPub.publish(this->blindDriveTwist);
                this->state = _pollinationBlindTurnToPlant;
            }
            break;
        case _pollinationBlindForward:
            if(hypot(this->robotPose.x - this->blindDriveInitPose.x, this->robotPose.y - this->blindDriveInitPose.y) >= this->blindDriveDistance)
            {
                ROS_INFO("pollination blind drive forward complete");
                this->blindDriveTwist.linear.x = 0.0;
                this->blindDriveTwist.angular.z = 0.0;
                this->blindDriveTwistPub.publish(this->blindDriveTwist);
                this->pollinationArmManeuversComplete = false;
                this->armMsg.data = true;
                this->armPub.publish(this->armMsg);
                this->state = _pollinationArm;
            }
            else
            {
                ROS_INFO_THROTTLE(1,"pollination blind drive forward executing");
                this->blindDriveTwist.linear.x = this->blindDriveSpeed;
                this->blindDriveTwist.angular.z = 0.0;
                this->blindDriveTwistPub.publish(this->blindDriveTwist);
                this->state = _pollinationBlindForward;
            }
            break;
        case _pollinationArm:
            if(this->pollinationArmManeuversComplete)
            {
                ROS_INFO("pollination arm proceedure complete");
                this->plantRowMap.plantRowsBlocks.at(this->bestPollinationCellIndex).pollinationComplete = true;
                this->driveActionStartTime = ros::Time::now().toSec();
                this->blindDriveInitPose = this->robotPose;
                this->state = _pollinationBlindBack;
            }
            else
            {
                ROS_INFO_THROTTLE(1,"pollination arm working");
                this->state = _pollinationArm;
            }
            break;
        case _pollinationBlindBack:
            if(hypot(this->robotPose.x - this->blindDriveInitPose.x, this->robotPose.y - this->blindDriveInitPose.y) >= this->blindDriveDistance)
            {
                ROS_INFO("pollination blind drive backward complete");
                this->blindDriveTwist.linear.x = 0.0;
                this->blindDriveTwistPub.publish(this->blindDriveTwist);
                this->state = _pollinationCheckFinished;
            }
            else
            {
                ROS_INFO_THROTTLE(1,"pollination blind drive backward executing");
                this->blindDriveTwist.linear.x = (-1.0)*this->blindDriveSpeed;
                this->blindDriveTwistPub.publish(this->blindDriveTwist);
                this->state = _pollinationBlindBack;
            }
            break;
        case _pollinationCheckFinished:
            if(findNumUnpollinatedCells() > 0)
            {
                this->state = _pollinationChooseWaypoint;
            }
            else
            {
                this->continueRunning = false;
                ROS_INFO("pollinated all flowers, ending mission");
            }
            break;
        }
        ros::spinOnce();
        this->loopRate.sleep();
    }
}

void MissionPlanning::setPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, geometry_msgs::Pose2D poseInput) // m, m, rad
{
ROS_INFO("Setting pose goal");
    const double pitch = 0.0;
    const double roll = 0.0;
    double cy = cos(poseInput.theta * 0.5);
    double sy = sin(poseInput.theta * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    ROS_INFO("%f %f %f %f %f %f", cy,sy,cr,sr,cp,sp);
    ROS_INFO("%f %f", poseInput.x, poseInput.y);

    poseGoal.target_pose.header.frame_id = "greenhouse"; //changed to greenhouse
    poseGoal.target_pose.pose.position.x = poseInput.x;
    poseGoal.target_pose.pose.position.y = poseInput.y;
    poseGoal.target_pose.pose.position.z = 0.0;
    poseGoal.target_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    poseGoal.target_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    poseGoal.target_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    poseGoal.target_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;
}

void MissionPlanning::driveDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    this->driveActionDone = true;
    ROS_DEBUG("drive goal done");
}
void MissionPlanning::driveActiveCallback()
{
    ROS_DEBUG("drive goal went active");
}
void MissionPlanning::driveFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_DEBUG("drive got feedback");
}

void MissionPlanning::poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    this->robotPose.x = msg->pose.pose.position.x;
    this->robotPose.y = msg->pose.pose.position.y;
    this->robotPose.theta = atan2(2.0*(msg->pose.pose.orientation.w*msg->pose.pose.orientation.z+msg->pose.pose.orientation.x*msg->pose.pose.orientation.y),
                                  1.0 - 2.0*(msg->pose.pose.orientation.y*msg->pose.pose.orientation.y+msg->pose.pose.orientation.z*msg->pose.pose.orientation.z));
    this->robotPoseCurrentQ.x = 0.0;
    this->robotPoseCurrentQ.y = 0.0;
    this->robotPoseCurrentQ.z = sin(0.5*this->robotPose.theta);
    this->robotPoseCurrentQ.w = cos(0.5*this->robotPose.theta);
}

void MissionPlanning::detectedFlowersCallback(const manipulation_common::FlowerMap::ConstPtr &msg)
{
    /*
    // Don't want to modify numFlowers if not performing survey pass
    if(this->performingSurveyPass)
    {
        // Find grid cells in range that can be updated and set all in-range grid cells' numFlowers to 0
        this->plantRowMap.setVisibleCells(this->robotPose, maxFlowerDetectDistance);
        // Loop over all detected flowers from vision and add to grid cell if grid cell is in range
        for(int i=0; i< msg->FlowerMap.size(); i++)
        {
            this->plantRowMap.addDetectedFlower(msg->FlowerMap(i).vec.vector, this->robotPose);
        }
    }
    for(int i=0; i < this->plantRowMap.plantRowsBlocks.size(); i++)
    {
        ROS_INFO("cell#: %i [%f,%f,%f], numFlowers: %u, polComplete: %i",i,this->plantRowMap.plantRowsBlocks.at(i).numFlowers, this->plantRowMap.plantRowsBlocks.at(i).pollinationPose.x, this->plantRowMap.plantRowsBlocks.at(i).y, this->plantRowMap.plantRowsBlocks.at(i).yNormalDir, this->plantRowMap.plantRowsBlocks.at(i).pollinationComplete);
    }*/
    
}

void MissionPlanning::armCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data==true)
    {
        this->pollinationArmManeuversComplete = true;
    }
}

unsigned int MissionPlanning::findNumUnpollinatedCells()
{
    unsigned int unpollinatedCount = 0;
    for(int i=0; i < this->plantRowMap.plantRowsBlocks.size(); i++)
    {
        //ROS_INFO("cell#: %i, numFlowers: %u, polComplete: %i",i,this->plantRowMap.plantRowsBlocks.at(i).numFlowers,this->plantRowMap.plantRowsBlocks.at(i).pollinationComplete);
        if(this->plantRowMap.plantRowsBlocks.at(i).numFlowers > 0 && this->plantRowMap.plantRowsBlocks.at(i).pollinationComplete==false)
        {
            unpollinatedCount++;
            //ROS_INFO("increment unpollinatedCount: %u",unpollinatedCount);
        }
    }
    return unpollinatedCount;
}

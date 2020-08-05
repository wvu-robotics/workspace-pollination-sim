#ifndef PLANT_ROW_MAP_HPP
#define PLANT_ROW_MAP_HPP
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h>

#define PI 3.14159265359
#define DEG2RAD PI/180.0
#define RAD2DEG 180.0/PI

#define NUM_ROWS 4

class PlantRowMap
{
public:
    // Types
    class RowBlock
    {
    public:
        // Methods
        RowBlock(float yInit, float xMinInit, float xMaxInit, float yPoseOffsetInit); // Constructor
        bool isInside(float xTest, float yTest, float robotY); // Return true if xMin <= xTest <= xMax, y==yTest (within small tolerance), and robot on visible row side
        // Members
        float y;
        float xMin;
        float xMax;
        float yNormalDir;
        unsigned int numFlowers;
        bool pollinationComplete;
        geometry_msgs::Pose2D pollinationPose;
    };
    // Methods
    PlantRowMap(); // Constructor
    void setVisibleCells(geometry_msgs::Pose2D robotPose, float maxRange);
    void addDetectedFlower(geometry_msgs::Vector3 flowerVector, geometry_msgs::Pose2D robotPose);
    // Members
    std::vector<PlantRowMap::RowBlock> plantRowsBlocks;
private:
    const float rowYValues[NUM_ROWS] = {3.24, 3.24, 5.41, 5.41}; // m
    const float parkingYOffset[NUM_ROWS] = {-1.2, 1.2, -1.2, 1.2}; //  m
    const float rowStartX = 2.09; // m
    const float rowEndX = 5.53; // m
    const float blockDeltaX = 0.688; // m
    const float cameraLocalXOffset = -0.315; // m
    std::vector<bool> cellsInRange;
};

#endif // PLANT_ROW_MAP_HPP

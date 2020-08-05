#include <autonomy/plant_row_map.hpp>

PlantRowMap::PlantRowMap()
{
    float blockStartX;
    int numBlocksPerRow = (int)floor((rowEndX-rowStartX)/blockDeltaX);
    // Initialize row blocks
    this->plantRowsBlocks.clear();
    for(int i=0; i<NUM_ROWS; i++)
    {
        blockStartX = rowStartX;
        for(int j=0; j<numBlocksPerRow; j++)
        {
            plantRowsBlocks.push_back(PlantRowMap::RowBlock(rowYValues[i],blockStartX,blockStartX+blockDeltaX,parkingYOffset[i]));
            blockStartX += blockDeltaX;
        }
    }
    cellsInRange.resize(this->plantRowsBlocks.size(), false);
}

void PlantRowMap::setVisibleCells(geometry_msgs::Pose2D robotPose, float maxRange)
{
    float cellCenterX;
    float cellCenterY;
    geometry_msgs::Pose2D cameraPose;
    cameraPose.x = robotPose.x + this->cameraLocalXOffset*cos(robotPose.theta);
    cameraPose.y = robotPose.y + this->cameraLocalXOffset*sin(robotPose.theta);
    cameraPose.theta = robotPose.theta;
    for(int i=0; i < this->plantRowsBlocks.size(); i++)
    {
        cellCenterX = (this->plantRowsBlocks.at(i).xMin + this->plantRowsBlocks.at(i).xMax)/2.0;
        cellCenterY = this->plantRowsBlocks.at(i).y;
        if(hypot(cellCenterX - cameraPose.x, cellCenterY - cameraPose.y) < maxRange)
        {
            this->cellsInRange.at(i) = true;
            this->plantRowsBlocks.at(i).numFlowers = 0;
        }
        else
        {
            this->cellsInRange.at(i) = false;
        }
    }
}

void PlantRowMap::addDetectedFlower(geometry_msgs::Vector3 flowerVector, geometry_msgs::Pose2D robotPose)
{
    float xIntersect;
    float yCandidateBlock;
    float slope;
    float bestCandidateYDiff = 10000.0; // Way more than is realistic to find initial best
    int bestCandidateIndex = 0;
    bool validIntersectionFound = false;
    std::vector<int> intersectingCandidateIndices;
    geometry_msgs::Pose2D cameraPose;
    cameraPose.x = robotPose.x + this->cameraLocalXOffset*cos(robotPose.theta);
    cameraPose.y = robotPose.y + this->cameraLocalXOffset*sin(robotPose.theta);
    cameraPose.theta = robotPose.theta;
    intersectingCandidateIndices.clear();
    float flowerLocalBearing = atan2(flowerVector.y, flowerVector.x);
    if(fabs(sin(flowerLocalBearing + cameraPose.theta)) > 0.0)
    {
        slope = tan(flowerLocalBearing + cameraPose.theta);
        for(int i=0; i < this->plantRowsBlocks.size(); i++)
        {
            yCandidateBlock = plantRowsBlocks.at(i).y;
            xIntersect = (yCandidateBlock - cameraPose.y)/slope + cameraPose.x;
            if(this->cellsInRange.at(i) && this->plantRowsBlocks.at(i).isInside(xIntersect, yCandidateBlock, cameraPose.y) &&
                    (yCandidateBlock - cameraPose.y)*sin(flowerLocalBearing + cameraPose.theta) > 0.0)
            {
                intersectingCandidateIndices.push_back(i);
            }
        }
        for(int i=0; i < intersectingCandidateIndices.size(); i++)
        {
            if(fabs(plantRowsBlocks.at(intersectingCandidateIndices.at(i)).y - cameraPose.y) < bestCandidateYDiff)
            {
                bestCandidateYDiff = fabs(plantRowsBlocks.at(intersectingCandidateIndices.at(i)).y - cameraPose.y);
                bestCandidateIndex = intersectingCandidateIndices.at(i);
                validIntersectionFound = true;
            }
        }
        if(validIntersectionFound)
        {
            this->plantRowsBlocks.at(bestCandidateIndex).numFlowers++;
        }
    }
    else
    {
        ROS_WARN("Flower detected parallel to plant rows");
    }
}

PlantRowMap::RowBlock::RowBlock(float yInit, float xMinInit, float xMaxInit, float yPoseOffsetInit)
{
    this->y = yInit;
    this->xMin = xMinInit;
    this->xMax = xMaxInit;
    this->numFlowers = 0;
    this->pollinationComplete = false;
    this->pollinationPose.x = (this->xMin + this->xMax)/2.0;
    this->pollinationPose.y = this->y + yPoseOffsetInit;
    if(yPoseOffsetInit > 0)
    {
        this->pollinationPose.theta = -PI/2.0;
        this->yNormalDir = 1.0;
    }
    else
    {
        this->pollinationPose.theta = PI/2.0;
        this->yNormalDir = -1.0;
    }
}

bool PlantRowMap::RowBlock::isInside(float xTest, float yTest, float robotY)
{
    if(fabs(yTest - this->y) < 0.001 && (robotY - this->y)*yNormalDir > 0.0)
    {
        if(xTest >= this->xMin && xTest <= this->xMax)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

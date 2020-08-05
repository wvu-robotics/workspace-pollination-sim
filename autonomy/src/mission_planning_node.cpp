#include <autonomy/mission_planning.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv,"mission_planning_node");
    MissionPlanning missionPlanning;
    missionPlanning.run();
    return 0;
}

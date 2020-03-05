#include "graph_slam/odometry_manager.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "odometry_manager_node");
    ros::NodeHandle nh;
    OdometryManager om(nh);
    om.run();
    return 0;
}
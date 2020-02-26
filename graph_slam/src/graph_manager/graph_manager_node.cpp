#include "graph_slam/graph_manager.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "graph_manager_node");
    ros::NodeHandle nh;
    GraphManager gm(nh);
    gm.run();
    return 0;
}
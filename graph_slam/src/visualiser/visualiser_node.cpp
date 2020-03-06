#include "graph_slam/visualiser.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "visualiser_node");
    ros::NodeHandle nh;
    Visualiser vis(nh);
    vis.run();
    return 0;
}
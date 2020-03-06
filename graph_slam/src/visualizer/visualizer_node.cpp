#include "graph_slam/visualizer.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "visualizer_node");
    ros::NodeHandle nh;
    Visualizer v(nh);
    v.run();
    return 0;
}
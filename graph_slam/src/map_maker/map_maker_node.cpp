#include "graph_slam/map_maker.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "map_maker_node");
    ros::NodeHandle nh;
    MapMaker mm(nh);
    mm.run();
    return 0;
}
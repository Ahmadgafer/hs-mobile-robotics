#include "graph_slam/scan_matcher.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "scan_matcher_node");
    ros::NodeHandle nh;
    ScanMatcher sm(nh);
    sm.run();
    return 0;
}
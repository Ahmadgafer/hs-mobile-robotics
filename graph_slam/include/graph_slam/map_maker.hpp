#ifndef __HSU_MOBILE_ROBOTICS_HH_MAP_MAKER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_MAP_MAKER_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/conversion_util.hpp>
#include <graph_slam/Graph.h>




class MapMaker
{
public:
    MapMaker(const ros::NodeHandle &_nh) : nh(_nh)
    {
        sub_graph = nh.subscribe("graph", 5, &MapMaker::subscribeGraphCallBack, this);
        pub_map = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);

        meta.resolution = 0.025;
        meta.width = 400;
        meta.height = 400;
        meta.origin.orientation.w = 1.;
        meta.origin.position.x = -5.;
        meta.origin.position.y = -5.;
    }

    ~MapMaker()
    {
    }

    void run()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_graph;
    ros::Publisher pub_map;
    nav_msgs::MapMetaData meta;

    void subscribeGraphCallBack(graph_slam::Graph msg)
    {
        nav_msgs::OccupancyGrid grid = composeOccupancyGrid(msg.keyframes);
        pub_map.publish(grid);
    }

    nav_msgs::OccupancyGrid composeOccupancyGrid(std::vector<graph_slam::KeyFrame> kfs)
    {
        nav_msgs::OccupancyGrid grid;
        // grid.info = meta;
        // grid.header.stamp = ros::Time::now();
        // grid.header.frame_id = "map";
        // std::vector<signed char> dat(400*400, -1);

        // for (graph_slam::KeyFrame kf: kfs)
        // {
        //     sensor_msgs::LaserScan scan = kf.scan;

        // }

        // grid.data = dat;

        return grid;


    }
};

#endif

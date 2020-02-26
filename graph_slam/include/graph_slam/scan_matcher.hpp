#ifndef __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/AddNewFactor.h>
#include <graph_slam/AddNewKF.h>
#include <graph_slam/GetLastKF.h>
#include <graph_slam/GetNearestKF.h>

class ScanMatcher
{
public:
    ScanMatcher(const ros::NodeHandle &_nh) : nh(_nh)
    {
        cli_add_new_factor = nh.serviceClient<graph_slam::AddNewFactor>("add_new_factor");
        cli_add_new_kf = nh.serviceClient<graph_slam::AddNewKF>("add_new_kf");
        cli_get_last_kf = nh.serviceClient<graph_slam::GetLastKF>("get_last_kf");
        cli_get_nearest_kf = nh.serviceClient<graph_slam::GetNearestKF>("get_nearest_kf");

        sub_laser_scan = nh.subscribe("laser_scan", 5, &ScanMatcher::laserScanCallback, this);
    }

    ~ScanMatcher()
    {
    }

    void run()
    {
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber sub_laser_scan;

    ros::ServiceClient cli_add_new_factor;
    ros::ServiceClient cli_add_new_kf;
    ros::ServiceClient cli_get_last_kf;
    ros::ServiceClient cli_get_nearest_kf;

    bool addNewFactor(graph_slam::Factor factor)
    {
        graph_slam::AddNewFactor srv;
        srv.request.factor = factor;
        bool is_ok = cli_add_new_factor.call(srv);
        return is_ok;
    }

    bool addNewKF(graph_slam::KeyFrame new_kf)
    {
        graph_slam::AddNewKF srv;
        srv.request.new_kf = new_kf;
        bool is_ok = cli_add_new_kf.call(srv);
        return is_ok;
    }

    graph_slam::KeyFrame getLastKF()
    {
        graph_slam::GetLastKF srv;
        bool is_ok = cli_get_last_kf.call(srv);
        return srv.response.last_kf;
    }

    graph_slam::KeyFrame getNearestKF(graph_slam::KeyFrame kf)
    {
        graph_slam::GetNearestKF srv;
        srv.request.kf = kf;
        bool is_ok = cli_get_last_kf.call(srv);
        return srv.response.nearest_kf;
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {

    }

};

#endif

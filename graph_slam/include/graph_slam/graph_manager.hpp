#ifndef __HSU_MOBILE_ROBOTICS_HH_GRAPH_MANAGER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_GRAPH_MANAGER_HPP


#include <ros/ros.h>

#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/AddNewFactor.h>
#include <graph_slam/AddNewKF.h>
#include <graph_slam/GetLastKF.h>
#include <graph_slam/GetNearestKF.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

class GraphManager
{
public:
    GraphManager(const ros::NodeHandle &_nh) : nh(_nh)
    {
        srv_add_new_factor = nh.advertiseService("add_new_factor", &GraphManager::addNewFactorCallback, this);
        srv_add_new_kf = nh.advertiseService("add_new_kf", &GraphManager::addNewKFCallback, this);
        srv_get_last_kf = nh.advertiseService("get_last_kf", &GraphManager::getLastKFCallback, this);
        srv_get_nearest_kf = nh.advertiseService("get_nearest_kf", &GraphManager::getNearestKFCallback, this);
    }

    ~GraphManager()
    {
    }

    void run()
    {
    }

private:
    // For ROS
    ros::NodeHandle nh;

    ros::ServiceServer srv_add_new_factor;
    ros::ServiceServer srv_add_new_kf;
    ros::ServiceServer srv_get_last_kf;
    ros::ServiceServer srv_get_nearest_kf;

    // For GTSAM
    

    bool addNewFactorCallback(graph_slam::AddNewFactor::Request &req, graph_slam::AddNewFactor::Response &res)
    {
        return true;
    }

    bool addNewKFCallback(graph_slam::AddNewKF::Request &req, graph_slam::AddNewKF::Response &res)
    {
        return true;
    }
    
    bool getLastKFCallback(graph_slam::GetLastKF::Request &req, graph_slam::GetLastKF::Response &res)
    {
        return true;
    }

    bool getNearestKFCallback(graph_slam::GetNearestKF::Request &req, graph_slam::GetNearestKF::Response &res)
    {
        return true;
    }


};



#endif
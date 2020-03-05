#ifndef __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/AddNewFactor.h>
#include <graph_slam/AddNewOdometryFactor.h>
#include <graph_slam/conversion_util.hpp>
#include <graph_slam/visualizer.hpp>

using namespace std;

class OdometryManager
{
public:

    OdometryManager(const ros::NodeHandle &_nh) : nh(_nh), visualizer(_nh)
    {
        cli_add_new_factor = nh.serviceClient<graph_slam::AddNewFactor>("add_new_factor");
        srv_add_new_factor = nh.advertiseService("add_new_odometry_factor", &OdometryManager::addNewFactorCallback, this);

        sub_odom = nh.subscribe("/odom", 5, &OdometryManager::odomCallback, this);
        
        callback_count = 0;
        prev_tf.setIdentity();
        
        bool parameters_loaded = true;
        parameters_loaded &= nh.getParam("wheel_odometry/covariance/segma_x", segma_x);
        parameters_loaded &= nh.getParam("wheel_odometry/covariance/segma_y", segma_y);
        parameters_loaded &= nh.getParam("wheel_odometry/covariance/segma_theta", segma_theta);

        Q.setZero();
        Q(0,0) = segma_x*segma_x;
        Q(1,1) = segma_y*segma_y;
        Q(2,2) = segma_theta*segma_theta;

        if(!parameters_loaded){
            ROS_ERROR("Can't load parameters");
        }
    }

    ~OdometryManager()
    {
    }

    void run()
    {
        ros::spin();
    }

private:
    int callback_count ;
    ros::NodeHandle nh;
    
    Eigen::Matrix4f prev_tf; // previous transform
    Eigen::Matrix3f Q; // Covariance
    
    ros::Subscriber sub_odom;

    ros::ServiceClient cli_add_new_factor;
    ros::ServiceServer srv_add_new_factor;

    Visualizer visualizer;

    map<double, Eigen::Matrix4f> buffer;

    float segma_x,segma_y,segma_theta;
    double dt;
    
    bool addNewFactor(graph_slam::Factor& factor)
    {
        graph_slam::AddNewFactor srv;
        srv.request.factor = factor;
        bool is_ok = cli_add_new_factor.call(srv);
        return is_ok;
    }

    bool addNewFactorCallback(graph_slam::AddNewOdometryFactor::Request &req, graph_slam::AddNewOdometryFactor::Response &res){
        if (req.id1 == -1)
            return true;
        Eigen::Matrix4f tf1 = buffer.lower_bound(req.stamp1.toSec())->second;
        Eigen::Matrix4f tf2 = buffer.lower_bound(req.stamp2.toSec())->second;
        Eigen::Matrix4f d_tf = tf1.inverse() * tf2;
        graph_slam::Factor factor;
        factor.id1 = req.id1;
        factor.id2 = req.id2;
        factor.type = "odometry";
        for(int i = 0; i < 9; i++) 
            factor.measurement_covariance[i] = Q(i/3, i%3);
        factor.measurement = make_Delta(d_tf);
        addNewFactor(factor);
        return true;
    }
   
    void odomCallback(const nav_msgs::Odometry& msg)
    {
        callback_count ++;
        if(callback_count == 1){ // just for first time recieving callbacl
            buffer[msg.header.stamp.toSec()] = prev_tf;
            return;
        }
        dt = msg.header.stamp.toSec() - buffer.rbegin()->first;
        Eigen::Matrix4f d_tf; 
        d_tf.setIdentity();
        double d_theta = msg.twist.twist.angular.z * dt;
        double dx = msg.twist.twist.linear.x * dt;
        d_tf(0,0) = cos(d_theta);
        d_tf(0,1) = -sin(d_theta);
        d_tf(1,0) = sin(d_theta);
        d_tf(1,1) = cos(d_theta);
        d_tf(0,3) = dx;
        Eigen::Matrix4f new_tf = prev_tf * d_tf;
        buffer[msg.header.stamp.toSec()] = new_tf;
        prev_tf = new_tf;
        
        graph_slam::KeyFrame keyframe;
        keyframe.pose = make_Delta(new_tf);
        visualizer.updateKF(keyframe,"robot_odom","world");
    }

};

#endif

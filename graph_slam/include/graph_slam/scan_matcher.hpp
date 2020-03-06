#ifndef __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP

const int KF_NEW_LIMIT_TIME = 5;
const float KF_NEW_LIMIT_DIST = 0.5; 
const float KF_NEW_LIMIT_ASSIMILARITY = 0.005;
const float LOOP_CLOSURE_LIMIT_ASSIMILARITY = 3;
const float LOOP_CLOSURE_LIMIT_TIME = 1.0;
const float LOOP_CLOSURE_LIMIT_DIST = 0.0;
const int LOOP_CLOSURE_KEY_FRAME_COUNT = 6;
// const double LASER_ODOM_COVARIANCE[9] = { 0.001, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.1 };
const double LASER_ODOM_GENERAL_SIGMAS[3] = { 0.1, 0.1, 0.15 };
const double LASER_ODOM_LOOP_CLOSURE_SIGMAS[3] = { 0.15, 0.15, 0.2 };

#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <graph_slam/Factor.h>
#include <graph_slam/FactorVis.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/AddNewFactor.h>
#include <graph_slam/AddNewKF.h>
#include <graph_slam/GetLastKF.h>
#include <graph_slam/GetNearestKF.h>
#include <graph_slam/conversion_util.hpp>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

using namespace std;

class ScanMatcher
{
public:
    ScanMatcher(const ros::NodeHandle &_nh) : nh(_nh)
    {
        cli_add_new_factor = nh.serviceClient<graph_slam::AddNewFactor>("add_new_factor");
        cli_add_new_kf = nh.serviceClient<graph_slam::AddNewKF>("add_new_kf");
        cli_get_last_kf = nh.serviceClient<graph_slam::GetLastKF>("get_last_kf");
        cli_get_nearest_kf = nh.serviceClient<graph_slam::GetNearestKF>("get_nearest_kf");

        sub_laser_scan = nh.subscribe("scan", 5, &ScanMatcher::laserScanCallback, this);
        // sub_odom = nh.subscribe("odom", 5, &ScanMatcher::odomCallback, this);
        
        // laser_point_cloud_new_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_point_cloud_new", 1000, this);
        // laser_point_cloud_last_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_point_cloud_last", 1000, this);
        kf_pub = nh.advertise<graph_slam::KeyFrame>("kf", 1000, this);
        factor_kfs_pub = nh.advertise<graph_slam::FactorVis>("factor_kfs", 1000, this); 

        // kf_last_time = ros::Time::now();
        // loop_closure_last_time = ros::Time::now();    

        tf_align_prior.setIdentity();
        w_T_last.setIdentity();
        w_T_new.setIdentity();
        nearest_T_w.setIdentity();
        nearest_T_new.setIdentity();
    }

    ~ScanMatcher()
    {
    }

    void run()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh;
    ros::Time kf_last_time;
    ros::Time kf_current_time;
    ros::Time loop_closure_last_time;
    ros::Time loop_closure_current_time;

    ros::Subscriber sub_laser_scan;
    ros::Subscriber sub_odom;
    ros::Publisher laser_point_cloud_new_pub;
    ros::Publisher laser_point_cloud_last_pub;
    ros::Publisher kf_pub;
    ros::Publisher factor_kfs_pub;

    ros::ServiceClient cli_add_new_factor;
    ros::ServiceClient cli_add_new_kf;
    ros::ServiceClient cli_get_last_kf;
    ros::ServiceClient cli_get_nearest_kf;

    Eigen::Matrix4f tf_align_prior;
    Eigen::Matrix4f w_T_last;
    Eigen::Matrix4f w_T_new;
    Eigen::Matrix4f w_T_nearest;
    Eigen::Matrix4f nearest_T_w;
    Eigen::Matrix4f nearest_T_new;

    graph_slam::KeyFrame kf_last;
    graph_slam::KeyFrame kf_new;
    graph_slam::KeyFrame kf_nearest;
    graph_slam::Factor factor_new;
    graph_slam::FactorVis factor_kfs;

    bool kf_first = true;
    bool kf_new_srv;
    bool kf_new_fac_srv;

    int32_t kf_id_count = 0;
    int kf_count_loop_closure = 0;
    
    double dt;
    double dt_loop;
    float dist = 0.0;
    float dist_loop = 0.0;

    sensor_msgs::PointCloud2 point_cloud2;
    sensor_msgs::PointCloud2 point_cloud2_last;

    Aligned laser_odom;

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
        bool is_ok = cli_get_nearest_kf.call(srv);
        return srv.response.nearest_kf;
    }


    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {   
        kf_current_time = scan->header.stamp;
        loop_closure_current_time = scan->header.stamp;
        point_cloud2 = scan_to_pointcloud(*scan);
        if (kf_first) {
            kf_first = false;
            kf_new.id = kf_id_count;
            kf_new.stamp = scan->header.stamp;
            kf_new.scan = *scan;
            kf_new.point_cloud = point_cloud2;
            kf_new.point_cloud.header.frame_id = "odom";
            factor_new.id2 = kf_id_count;
            factor_new.type = "prior";
            // BE AWARE: Maybe first pose should not be 0, 0 ,0 ? could cause some type of failure
            kf_new_srv = addNewKF(kf_new);
            kf_new_fac_srv = addNewFactor(factor_new);
            kf_pub.publish(kf_new);
            ++kf_id_count;
            kf_last_time = kf_new.stamp;
            loop_closure_last_time = kf_new.stamp;
            w_T_last = pose_to_transform(kf_new.pose);
            return;  
        }

        kf_last = getLastKF();
        // laser_point_cloud_new_pub.publish(point_cloud2);
        // laser_point_cloud_last_pub.publish(kf_last.point_cloud);
        laser_odom = gicp_register(point_cloud2, kf_last.point_cloud, tf_align_prior);
        w_T_new = w_T_last*laser_odom.last_T_new;
        std::cout << "KF converged: " << laser_odom.converged << endl;
        std::cout << "KF fitness: " << laser_odom.fitness << endl;
        dt = (kf_current_time - kf_last_time).toSec();
        dist = sqrt(pow(laser_odom.delta.x, 2) + pow(laser_odom.delta.y, 2));
        std::cout << "KF dt: " << dt <<endl;
        std::cout << "KF dist: " << dist <<endl;
        if (laser_odom.converged && (dt > KF_NEW_LIMIT_TIME || laser_odom.fitness > KF_NEW_LIMIT_ASSIMILARITY || dist > KF_NEW_LIMIT_DIST)) {
            std::cout << "NEW KF: " << kf_id_count << endl;
            kf_new.id = kf_id_count;
            kf_new.stamp = scan->header.stamp;
            kf_new.scan = *scan;
            kf_new.pose = transfrom_to_pose(w_T_new);
            kf_new.point_cloud = point_cloud2;
            kf_new.point_cloud.header.frame_id = "kf_" + to_string(kf_new.id);

            factor_new.id1 = kf_last.id;
            factor_new.id2 = kf_id_count;
            factor_new.type = "between";
            factor_new.measurement = laser_odom.delta;
            for (int i = 0; i < 3; i++) {
                factor_new.measurement_sigmas[i] = LASER_ODOM_GENERAL_SIGMAS[i]; //do copy instead
            }
            kf_new_srv = addNewKF(kf_new);
            kf_new_fac_srv = addNewFactor(factor_new); 
            kf_pub.publish(kf_new);

            factor_kfs.id = kf_id_count;
            factor_kfs.stamp = scan->header.stamp;
            factor_kfs.pose_last = kf_last.pose;
            factor_kfs.pose_new = kf_new.pose;
            factor_kfs_pub.publish(factor_kfs);  
            
            dt_loop = (loop_closure_current_time - loop_closure_last_time).toSec();
            dist_loop += sqrt(pow(kf_new.pose.x - kf_last.pose.x, 2) + pow(kf_new.pose.y - kf_last.pose.y, 2));
            std::cout << "LOOP dt: " << dt_loop <<endl;
            std::cout << "LOOP dist: " << dist_loop <<endl;
            std::cout << "KF COUNT: " << kf_count_loop_closure <<endl;
            // if (dt_loop > LOOP_CLOSURE_LIMIT_TIME || dist_loop > LOOP_CLOSURE_LIMIT_DIST || kf_count_loop_closure > LOOP_CLOSURE_KEY_FRAME_COUNT) {
            if (kf_count_loop_closure > LOOP_CLOSURE_KEY_FRAME_COUNT) {
                std::cout << "Try Loop Closure" <<endl;
                kf_nearest = getNearestKF(kf_new);
                std::cout << "Nearest KF: " << kf_nearest.id <<endl;
                w_T_nearest = pose_to_transform(kf_nearest.pose);
                nearest_T_w = w_T_nearest.inverse() * w_T_new;
                laser_odom = gicp_register(kf_new.point_cloud, kf_nearest.point_cloud, nearest_T_w);
                std::cout << "LOOP converged: " << laser_odom.converged <<endl;
                std::cout << "LOOP fitness: " << laser_odom.fitness <<endl;
                if (laser_odom.converged && (laser_odom.fitness < LOOP_CLOSURE_LIMIT_ASSIMILARITY)) {
                    std::cout << "Perform Loop CLosure" <<endl;
                    factor_new.id1 = kf_nearest.id;
                    factor_new.id2 = kf_id_count;
                    factor_new.type = "loop_closure";
                    factor_new.measurement = laser_odom.delta;
                    for (int i = 0; i < 3; i++) {
                        factor_new.measurement_sigmas[i] = LASER_ODOM_LOOP_CLOSURE_SIGMAS[i]; //do copy instead
                    }
                    kf_new_fac_srv = addNewFactor(factor_new);
                    dist_loop = 0.0;
                    std::cout << "LOOP CLOSED" <<endl;
                    loop_closure_last_time = kf_new.stamp;
                    kf_count_loop_closure = 0;
                }                
            }
            ++kf_count_loop_closure;
            ++kf_id_count;
            dist = 0.0;
            tf_align_prior.setIdentity();
            kf_last_time = kf_new.stamp;
            w_T_last = pose_to_transform(kf_new.pose);           
        } else {
            tf_align_prior = laser_odom.last_T_new;
            }  
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& od)
    {   
        if ((ros::Time::now() - kf_last_time).toSec() > 3) {
            // cout << (ros::Time::now() - kf_last_time).toSec() << endl;
            // tf::StampedTransform transform;
            // listener.lookupTransform("/base_footprint", "/base_link", scan->header::stamp, transform);
            graph_slam::KeyFrame test_kf;
            test_kf.id = od->header.seq;
            geometry_msgs::Quaternion quat = od->pose.pose.orientation;
            tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            // std::cout << std::to_string(od->pose.pose.position.x);
            test_kf.stamp = od->header.stamp;
            test_kf.pose.x = od->pose.pose.position.x;
            test_kf.pose.y = od->pose.pose.position.y;
            test_kf.pose.theta = yaw;
            kf_pub.publish(test_kf); 
            kf_last_time = ros::Time::now();
        }
    }

};

#endif

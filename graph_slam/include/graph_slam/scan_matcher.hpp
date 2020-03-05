#ifndef __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Time.h>

#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/AddNewFactor.h>
#include <graph_slam/AddNewKF.h>
#include <graph_slam/GetLastKF.h>
#include <graph_slam/GetNearestKF.h>
#include <graph_slam/AddNewOdometryFactor.h>
#include <graph_slam/conversion_util.hpp>
#include <graph_slam/visualizer.hpp>

using namespace std;

class ScanMatcher
{
public:

    ScanMatcher(const ros::NodeHandle &_nh) : nh(_nh), visualizer(_nh)
    {
        cli_add_new_odometry_factor = nh.serviceClient<graph_slam::AddNewOdometryFactor>("add_new_odometry_factor");
        cli_add_new_factor = nh.serviceClient<graph_slam::AddNewFactor>("add_new_factor");
        cli_add_new_kf = nh.serviceClient<graph_slam::AddNewKF>("add_new_kf");
        cli_get_last_kf = nh.serviceClient<graph_slam::GetLastKF>("get_last_kf");
        cli_get_nearest_kf = nh.serviceClient<graph_slam::GetNearestKF>("get_nearest_kf");

        sub_laser_scan = nh.subscribe("/scan", 5, &ScanMatcher::laserScanCallback, this);
        pub_pointcloud1 = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_publisher_1", 1000);
        pub_pointcloud2 = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_publisher_2", 1000);
        
        callback_count = 0;
        transform_prior.setIdentity();

        bool parameters_loaded = true;
        
        parameters_loaded &= nh.getParam("laser_odometry/fitness_threshould", fitness_threshould);
        parameters_loaded &= nh.getParam("laser_odometry/durationToAddNewKF", durationToAddNewKF);
        parameters_loaded &= nh.getParam("laser_odometry/covariance/segma_x", segma_x);
        parameters_loaded &= nh.getParam("laser_odometry/covariance/segma_y", segma_y);
        parameters_loaded &= nh.getParam("laser_odometry/covariance/segma_theta", segma_theta);

        parameters_loaded &= nh.getParam("laser_loop_closure/covariance/segma_x", _segma_x);
        parameters_loaded &= nh.getParam("laser_loop_closure/covariance/segma_y", _segma_y);
        parameters_loaded &= nh.getParam("laser_loop_closure/covariance/segma_theta", _segma_theta);
        parameters_loaded &= nh.getParam("laser_loop_closure/loop_close_count", loop_close_count);
        parameters_loaded &= nh.getParam("laser_loop_closure/fitness_threshould_loopclosure", fitness_threshould_loopclosure);

        if(!parameters_loaded){
            ROS_ERROR("Can't load parameters");
        }
    }

    ~ScanMatcher()
    {
    }

    void run()
    {
        ros::spin();
    }

private:
    int callback_count ;
    ros::NodeHandle nh;
    Eigen::Matrix4f transform_prior;

    ros::Publisher pub_pointcloud1;
    ros::Publisher pub_pointcloud2;

    ros::Subscriber sub_laser_scan;
    ros::ServiceClient cli_add_new_odometry_factor;
    ros::ServiceClient cli_add_new_factor;
    ros::ServiceClient cli_add_new_kf;
    ros::ServiceClient cli_get_last_kf;
    ros::ServiceClient cli_get_nearest_kf;

    Visualizer visualizer;

    int loop_close_count;
    double durationToAddNewKF;
    float segma_x,segma_y,segma_theta;
    float _segma_x,_segma_y,_segma_theta;
    double fitness_threshould;
    double fitness_threshould_loopclosure;

    bool addNewOdometryFactor(graph_slam::Factor& factor, ros::Time stamp1, ros::Time stamp2){
        graph_slam::AddNewOdometryFactor srv;
        srv.request.id1 = factor.id1;
        srv.request.id2 = factor.id2;
        srv.request.stamp1 = stamp1;
        srv.request.stamp2 = stamp2;
        bool is_ok = cli_add_new_odometry_factor.call(srv);
        return is_ok;
    }
    bool addNewFactor(graph_slam::Factor factor)
    {
        graph_slam::AddNewFactor srv;
        srv.request.factor = factor;
        bool is_ok = cli_add_new_factor.call(srv);
        return is_ok;
    }

    bool addNewKF(graph_slam::KeyFrame &new_kf)
    {
        graph_slam::AddNewKF srv;
        srv.request.new_kf = new_kf;
        bool is_ok = cli_add_new_kf.call(srv);
        if(is_ok){
            new_kf = srv.response.kf;
        }
        return is_ok;
    }

    bool getLastKF(graph_slam::KeyFrame& kf)
    {
        graph_slam::GetLastKF srv;
        bool is_ok = cli_get_last_kf.call(srv);
        kf = srv.response.last_kf;
        return is_ok;
    }

    bool getNearestKF(graph_slam::KeyFrame kf, graph_slam::KeyFrame& nearest_kf)
    {
        graph_slam::GetNearestKF srv;
        srv.request.kf = kf;
        bool is_ok = cli_get_nearest_kf.call(srv);
        nearest_kf = srv.response.nearest_kf;
        return is_ok;
    }

    bool checkAlignement(Alignement alignment){
        cout << "alignment fitness is : " << alignment.fitness << endl;
        //cout << alignment.transform << endl;
        return alignment.converged && alignment.fitness > fitness_threshould;
    }
    bool checkAlignementLoopClosure(Alignement alignment){
        cout << "alignment fitness is : " << alignment.fitness << endl;
        //cout << alignment.transform << endl;
        return alignment.converged && alignment.fitness < fitness_threshould_loopclosure;
    }
    void laserScanCallback(const sensor_msgs::LaserScan& msg)
    {
        sensor_msgs::PointCloud2 pc2 = scan_to_pointcloud(msg);

        graph_slam::KeyFrame lastKF;
        this->getLastKF(lastKF);
        if (lastKF.id == -1) {
            lastKF.point_cloud = pc2;
        }

        Alignement alignment = gicp_register(pc2,lastKF.point_cloud,transform_prior);

        if(lastKF.id == -1 || checkAlignement(alignment)){

            graph_slam::KeyFrame newKF;
            newKF.stamp = msg.header.stamp;
            newKF.pose = move_pose(lastKF.pose, alignment.Delta);
            newKF.point_cloud = pc2;

            visualizer.updateKF(newKF);
            pub_pointcloud1.publish(transform_pointcloud(newKF.point_cloud, newKF.pose));
            pub_pointcloud2.publish(transform_pointcloud(lastKF.point_cloud, lastKF.pose));


            double duration = (newKF.stamp - lastKF.stamp).toSec();
            double distance = hypot(newKF.pose.x - lastKF.pose.x, newKF.pose.y - lastKF.pose.y);
            double similarity = 1.0;

            // if (duration > 1.0 || similarity < 1 || distance > 10 || lastKF.id == -1){
            if (duration >= durationToAddNewKF || lastKF.id == -1){
                this->addNewKF(newKF);
                graph_slam::Factor factor;
                if(lastKF.id == -1)
                    factor.type = "prior";
                else
                    factor.type = "between";
                factor.id1 = lastKF.id;
                factor.id2 = newKF.id;
                factor.measurement = alignment.Delta;
                float covariance[9] = {0.0}; covariance[0]=segma_x*segma_x; covariance[4]=segma_y*segma_y; covariance[8]=segma_theta*segma_theta;
                for(int i = 0; i < 9; i++) factor.measurement_covariance[i] = covariance[i];
                this->addNewFactor(factor);
                // this->addNewOdometryFactor(factor,lastKF.stamp, newKF.stamp);
                transform_prior.setIdentity();

                callback_count ++;
                if (callback_count % loop_close_count == 0){
                    graph_slam::KeyFrame nearestKF;
                    if(this->getNearestKF(newKF,nearestKF)){
                        cout << "Loop closure accepted" << endl;
                        geometry_msgs::Pose2D new_delta;

                        new_delta = get_prior(nearestKF.pose, newKF.pose);
                        Eigen::Matrix4f new_tf_prior = make_tf(new_delta);
                        Alignement alignment2 = gicp_register(newKF.point_cloud,nearestKF.point_cloud,new_tf_prior);

                        if(checkAlignementLoopClosure(alignment2)){
                            graph_slam::Factor new_factor;
                            new_factor.type = "between";
                            new_factor.id1 = nearestKF.id;
                            new_factor.id2 = newKF.id;
                            new_factor.measurement = alignment2.Delta;
                            float covariance_new[9] = {0.0}; covariance_new[0]=_segma_x * _segma_x; covariance_new[4]=_segma_y * _segma_y; covariance_new[8]=_segma_theta * _segma_theta;
                            for(int i = 0;i < 9; i++) new_factor.measurement_covariance[i] = covariance_new[i];
                            this->addNewFactor(new_factor);
                        }
                        else {
                            callback_count--;
                        }
                    }
                }
            }
            else {
                transform_prior = alignment.transform;
            }
        }

    }

};

#endif

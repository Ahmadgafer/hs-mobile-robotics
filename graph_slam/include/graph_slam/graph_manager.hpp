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
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <graph_slam/visualizer.hpp>

using namespace std;

class GraphManager
{
public:
    GraphManager(const ros::NodeHandle &_nh) : nh(_nh), visualizer(_nh)
    {
        srv_add_new_factor = nh.advertiseService("add_new_factor", &GraphManager::addNewFactorCallback, this);
        srv_add_new_kf = nh.advertiseService("add_new_kf", &GraphManager::addNewKFCallback, this);
        srv_get_last_kf = nh.advertiseService("get_last_kf", &GraphManager::getLastKFCallback, this);
        srv_get_nearest_kf = nh.advertiseService("get_nearest_kf", &GraphManager::getNearestKFCallback, this);

        bool parameters_loaded = true;
        parameters_loaded &= nh.getParam("laser_loop_closure/skipLastKF", skipLastKF);

        if(!parameters_loaded){
            ROS_ERROR("Can't load parameters");
        }

        visualizer.updatePointclouds(keyframes);

    }

    ~GraphManager()
    {
    }

    void run()
    {
        ros::spin();
    }

private:
    // For ROS

    
    ros::NodeHandle nh;
    vector<graph_slam::KeyFrame> keyframes;
    vector<graph_slam::Factor> factors;
    gtsam::NonlinearFactorGraph graph;

    ros::ServiceServer srv_add_new_factor;
    ros::ServiceServer srv_add_new_kf;
    ros::ServiceServer srv_get_last_kf;
    ros::ServiceServer srv_get_nearest_kf;
    
    Visualizer visualizer;
    
    int skipLastKF;

    bool addNewFactorCallback(graph_slam::AddNewFactor::Request &req, graph_slam::AddNewFactor::Response &res)
    {
        ROS_INFO_STREAM("Add new Factor request: " << req);
        if (req.factor.type == "prior"){
            gtsam::Pose2 priorMean(0.0,0.0,0.0);
            gtsam::Matrix33 covariance;
            for(int i = 0; i < 9; i++) {
                covariance(i/3, i%3) = req.factor.measurement_covariance[i];
            }
            gtsam::noiseModel::Gaussian::shared_ptr priorNoise = 
                gtsam::noiseModel::Gaussian::Covariance(covariance);
            // gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
            //     gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1,0.1,0.1));
            graph.add(gtsam::PriorFactor<gtsam::Pose2>(req.factor.id2, priorMean, priorNoise));
            factors.push_back(req.factor);
        }
        else if (req.factor.type == "between" || req.factor.type == "odometry"){
            gtsam::Pose2 odometry(req.factor.measurement.x,req.factor.measurement.y,req.factor.measurement.theta );
            gtsam::Matrix33 covariance;
            for(int i = 0; i < 9; i++) {
                covariance(i/3, i%3) = req.factor.measurement_covariance[i];
            }
            // gtsam::print(gtsam::Matrix(covariance));
            gtsam::noiseModel::Gaussian::shared_ptr noise = 
                gtsam::noiseModel::Gaussian::Covariance(covariance);
            graph.add(gtsam::BetweenFactor<gtsam::Pose2>(req.factor.id1, req.factor.id2, odometry, noise));
            factors.push_back(req.factor);
            if(req.factor.id1 + 1 != req.factor.id2){
                loop_closure();
            }
        }
        else {
            ROS_ERROR("wrong req.factor.type");
        }
        factors.push_back(req.factor);
        
        visualizer.updateGraph(keyframes,factors);
        return true;
    }

    bool addNewKFCallback(graph_slam::AddNewKF::Request &req, graph_slam::AddNewKF::Response &res)
    {
        //ROS_INFO_STREAM("Add new KF request: " << req);
        req.new_kf.id = int(keyframes.size());
        keyframes.push_back(req.new_kf);
        res.kf = keyframes.back();
        ROS_INFO_STREAM("Add new KF: " << res.kf.id << " " << res.kf.pose);
        visualizer.updateGraph(keyframes,factors);
        return true;
    }
    
    bool getLastKFCallback(graph_slam::GetLastKF::Request &req, graph_slam::GetLastKF::Response &res)
    {
        if(keyframes.size() == 0){
            res.last_kf = graph_slam::KeyFrame();
            res.last_kf.id = -1;
        }
        else
            res.last_kf = keyframes.back(); 
            
        return true;
    }

    bool getNearestKFCallback(graph_slam::GetNearestKF::Request &req, graph_slam::GetNearestKF::Response &res)
    {   
        double min_distance = 100000.0;
        if (skipLastKF >= keyframes.size())
            return false;
        for (int i = 0;i < (int)keyframes.size() - skipLastKF; i++){
            double distance = hypot(req.kf.pose.x - keyframes[i].pose.x, req.kf.pose.y - keyframes[i].pose.y);
            if(distance < min_distance){
                min_distance = distance;
                res.nearest_kf = keyframes[i];
            }
        }
        // ROS_INFO_STREAM("Get nearest KF response: " << res.nearest_kf.id << " " << res.nearest_kf.pose); 
        return true;
    }

    void loop_closure(){
        gtsam::Values initial;
        for(int i = 0;i < (int)keyframes.size(); i++){
            initial.insert(keyframes[i].id, gtsam::Pose2(keyframes[i].pose.x, keyframes[i].pose.y, keyframes[i].pose.theta));
        }
        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
        for ( const gtsam::Values::ConstKeyValuePair& key_value: result )
        {
            gtsam::Pose2 pose = key_value.value.cast<gtsam::Pose2>();
            int i = key_value.key;
            keyframes[i].pose.x = pose.x();
            keyframes[i].pose.y = pose.y();
            keyframes[i].pose.theta = pose.theta();
        }
        visualizer.updatePointclouds(keyframes);
    }

};



#endif
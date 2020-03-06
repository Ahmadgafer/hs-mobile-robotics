#ifndef __HSU_MOBILE_ROBOTICS_HH_GRAPH_MANAGER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_GRAPH_MANAGER_HPP

const int KF_NEAREST_BUFFER = 3;

#include <ros/ros.h>

#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/Graph.h>
#include <graph_slam/AddNewFactor.h>
#include <graph_slam/AddNewKF.h>
#include <graph_slam/GetLastKF.h>
#include <graph_slam/GetNearestKF.h>
#include <graph_slam/conversion_util.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam;

class GraphManager
{
public:
    GraphManager(const ros::NodeHandle &_nh) : nh(_nh)
    {
        srv_add_new_factor = nh.advertiseService("add_new_factor", &GraphManager::addNewFactorCallback, this);
        srv_add_new_kf = nh.advertiseService("add_new_kf", &GraphManager::addNewKFCallback, this);
        srv_get_last_kf = nh.advertiseService("get_last_kf", &GraphManager::getLastKFCallback, this);
        srv_get_nearest_kf = nh.advertiseService("get_nearest_kf", &GraphManager::getNearestKFCallback, this);

        graph_pub = nh.advertise<graph_slam::Graph>("graph", 1000, this);

        count_factor_id = 0;
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

    ros::ServiceServer srv_add_new_factor;
    ros::ServiceServer srv_add_new_kf;
    ros::ServiceServer srv_get_last_kf;
    ros::ServiceServer srv_get_nearest_kf;

    ros::Publisher graph_pub;

    graph_slam::Factor factor;
    graph_slam::Graph graph_msg;

    vector<graph_slam::KeyFrame> kf_list;
    vector<graph_slam::Factor> factor_list;

    // For GTSAM
    NonlinearFactorGraph graph;
    Values values;
    Values result;
    // noiseModel::Diagonal::shared_ptr noise_model_prior;
    noiseModel::Diagonal::shared_ptr noise_model_laser_odom;

    int32_t count_factor_id;
    int32_t count_kf;


    void loopClosure(){
        values.print("\nOldValues:\n");
        result = LevenbergMarquardtOptimizer(graph, values).optimize();        
        values = result;
        result.print("\nREsults:\n");
        values.print("\nNew Values:\n");
        graph.print("\nFactor Graph:\n");

        for ( const gtsam::Values::ConstKeyValuePair& key_value: values )
        {
            gtsam::Pose2 pose = key_value.value.cast<gtsam::Pose2>();
            int i = key_value.key;
            kf_list[i].pose.x = pose.x();
            kf_list[i].pose.y = pose.y();
            kf_list[i].pose.theta = pose.theta();
            // cout << " frame " << i << " " << pose.x() << " " << pose.y() << " " << pose.theta() << endl;
        }
    }

    bool addNewFactorCallback(graph_slam::AddNewFactor::Request &req, graph_slam::AddNewFactor::Response &res)
    {
        noise_model_laser_odom = noiseModel::Diagonal::Sigmas(Vector3(req.factor.measurement_sigmas[0], req.factor.measurement_sigmas[1], req.factor.measurement_sigmas[2]));
        Pose2 laser_odom(req.factor.measurement.x, req.factor.measurement.y, req.factor.measurement.theta);
        if (req.factor.type == "prior") {
                graph.add(PriorFactor<Pose2>(count_factor_id, laser_odom, noise_model_laser_odom));
                return true;
        } 
        else if (req.factor.type == "between") {
            factor_list.push_back(req.factor);
            graph.add(BetweenFactor<Pose2>(req.factor.id1, req.factor.id2, laser_odom, noise_model_laser_odom));
        }
        else if (req.factor.type == "loop_closure" && !values.empty()) {
            factor_list.push_back(req.factor);
            graph.add(BetweenFactor<Pose2>(req.factor.id1, req.factor.id2, laser_odom, noise_model_laser_odom));
            ROS_INFO_STREAM("Loop closure factor" << req.factor);
            loopClosure();
        }
        ++count_factor_id;
        graph_msg.keyframes = kf_list;
        graph_msg.factors = factor_list;
        graph_pub.publish(graph_msg);
        return true;
    }

    bool addNewKFCallback(graph_slam::AddNewKF::Request &req, graph_slam::AddNewKF::Response &res)
    {
        Pose2 kf(req.new_kf.pose.x, req.new_kf.pose.y, req.new_kf.pose.theta);
        kf_list.push_back(req.new_kf);
        values.insert(req.new_kf.id, kf);
        ++count_kf;
        return true;
    }

    bool getLastKFCallback(graph_slam::GetLastKF::Request &req, graph_slam::GetLastKF::Response &res)
    {
        res.last_kf = kf_list.back();
        return true;
    }

    bool getNearestKFCallback(graph_slam::GetNearestKF::Request &req, graph_slam::GetNearestKF::Response &res)
    {
        int kf_size_buffered = kf_list.size();
        double min_dist = INFINITY;
        if (kf_size_buffered > KF_NEAREST_BUFFER) {
            kf_size_buffered = kf_size_buffered - KF_NEAREST_BUFFER;
        }
        for (int i = 0; i < kf_size_buffered - 1; i++) {
            float dist = sqrt(pow(kf_list[i].pose.x - req.kf.pose.x, 2) + pow(kf_list[i].pose.y - req.kf.pose.y, 2));
            std::cout << "i = " << i << std::endl; 
            if (dist < min_dist) {
                min_dist = dist;
                res.nearest_kf = kf_list[i];
            }
        }
        std::cout << "kf_nearest_id = " << res.nearest_kf.id << std::endl;
        return true;
    }
};

#endif
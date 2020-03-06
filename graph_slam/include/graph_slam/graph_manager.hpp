#ifndef __HSU_MOBILE_ROBOTICS_HH_GRAPH_MANAGER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_GRAPH_MANAGER_HPP


#include <unordered_map>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/Graph.h>
#include <graph_slam/AddNewFactor.h>
#include <graph_slam/AddNewKF.h>
#include <graph_slam/GetLastKF.h>
#include <graph_slam/GetNearestKF.h>

#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>



class GraphManager
{
public:
    GraphManager(const ros::NodeHandle &_nh) : nh(_nh)
    {

        pub_graph = nh.advertise<graph_slam::Graph>("graph", 10);
        loadParams();
        loadServices();
        initGraph();
        kf_counter = 0;
        
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
    ros::Publisher pub_graph;

    // For GTSAM
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    int kf_counter;
    std::vector<graph_slam::KeyFrame> kf_list;
    std::vector<graph_slam::Factor> factor_list;

    // Paramter
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise;
    gtsam::noiseModel::Diagonal::shared_ptr wheel_odometry_noise;
    gtsam::noiseModel::Diagonal::shared_ptr loop_closure_noise;
    std::vector<double> odom_noise_params;
    std::vector<double> wheel_odom_noise_params;
    std::vector<double> loop_closure_noise_params;
    int n_ignore_recent_kf;

    void initGraph()
    {
    }

    void loadServices()
    {
        srv_add_new_factor = nh.advertiseService("add_new_factor", &GraphManager::addNewFactorCallback, this);
        srv_add_new_kf = nh.advertiseService("add_new_kf", &GraphManager::addNewKFCallback, this);
        srv_get_last_kf = nh.advertiseService("get_last_kf", &GraphManager::getLastKFCallback, this);
        srv_get_nearest_kf = nh.advertiseService("get_nearest_kf", &GraphManager::getNearestKFCallback, this);
    }

    void loadParams()
    {
        std::vector<double> def_odom_noise_params = {0.3, 0.3, 0.5};
        std::vector<double> def_wheel_odom_noise_params = {0.1, 0.1, 0.1};
        std::vector<double> def_loop_closure_noise_params = {0.3, 0.3, 1.0};
        nh.param<std::vector<double>>("odometry_noise", odom_noise_params, def_odom_noise_params);
        nh.param<std::vector<double>>("wheel_odometry_noise", wheel_odom_noise_params, def_wheel_odom_noise_params);
        nh.param<std::vector<double>>("loop_closure_noise", loop_closure_noise_params, def_loop_closure_noise_params); 
        odometry_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(odom_noise_params.at(0), odom_noise_params.at(1), odom_noise_params.at(2)));
        wheel_odometry_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(wheel_odom_noise_params.at(0), wheel_odom_noise_params.at(1), wheel_odom_noise_params.at(2)));
        loop_closure_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(loop_closure_noise_params.at(0), loop_closure_noise_params.at(1), loop_closure_noise_params.at(2)));

        nh.param<int>("number_of_keyframes_to_ignore_for_finding_nearest", n_ignore_recent_kf, 5);
    }

    bool addNewFactorCallback(graph_slam::AddNewFactor::Request &req, graph_slam::AddNewFactor::Response &res)
    {
        factor_list.push_back(req.factor);
        gtsam::Pose2 transform(req.factor.measurement.x, req.factor.measurement.y, req.factor.measurement.theta);

        if (req.factor.type == "odometry")
        {
            gtsam::BetweenFactor<gtsam::Pose2> factor(req.factor.id1, req.factor.id2, transform, odometry_noise);
            graph.add(factor);

        }
        else if (req.factor.type == "wheel_odometry")
        {
            gtsam::BetweenFactor<gtsam::Pose2> factor(req.factor.id1, req.factor.id2, transform, wheel_odometry_noise);
            graph.add(factor);
            optimize();

        }
        else if (req.factor.type == "loop_closure")
        {
            gtsam::BetweenFactor<gtsam::Pose2> factor(req.factor.id1, req.factor.id2, transform, loop_closure_noise);
            graph.add(factor);
            optimize();
        }
        publishCurrentGraph();
        return true;
    }

    bool addNewKFCallback(graph_slam::AddNewKF::Request &req, graph_slam::AddNewKF::Response &res)
    {
        graph_slam::KeyFrame new_kf = req.new_kf;
        new_kf.id = kf_counter;
        gtsam::Pose2 pose(new_kf.pose.x, new_kf.pose.y, new_kf.pose.theta);
        values.insert(kf_counter, pose);
        res.kf = new_kf;
        kf_list.push_back(new_kf);
        if (kf_counter == 0)
        {
            graph.add(gtsam::PriorFactor<gtsam::Pose2>(kf_counter, pose, gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1e-10, 1e-10, 1e-10))));
        }
        kf_counter += 1;

        return true;
    }
    
    bool getLastKFCallback(graph_slam::GetLastKF::Request &req, graph_slam::GetLastKF::Response &res)
    {
        res.last_kf = kf_list.at(kf_counter-1);
        return true;
    }

    bool getNearestKFCallback(graph_slam::GetNearestKF::Request &req, graph_slam::GetNearestKF::Response &res)
    {
        graph_slam::KeyFrame nearest_kf = findNearestKF(req.kf);
        res.nearest_kf = nearest_kf;
        return true;
    }

    double distBetweenKFs(graph_slam::KeyFrame kf1, graph_slam::KeyFrame kf2)
    {
        double x1 = kf1.pose.x, y1 = kf1.pose.y;
        double x2 = kf2.pose.x, y2 = kf2.pose.y;
        double dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
        return dist;
    }

    graph_slam::KeyFrame findNearestKF(graph_slam::KeyFrame source)
    {
        double nearest_dist = 1e10;
        double nearest_id = 0;
        for (int i = 0; i < kf_counter - n_ignore_recent_kf - 1; i++)
        {
            double dist = distBetweenKFs(source, kf_list[i]);
            if (dist < nearest_dist)
            {
                nearest_dist = dist;
                nearest_id = i;
            }
        }
        return kf_list[nearest_id];
    }

    geometry_msgs::Pose2D gtsam2ROSPose2(gtsam::Pose2 pose)
    {
        geometry_msgs::Pose2D pose_ros;
        pose_ros.x = pose.x();
        pose_ros.y = pose.y();
        pose_ros.theta = pose.theta();
        return pose_ros;
    }

    void optimize()
    {
        gtsam::Values optimized = gtsam::LevenbergMarquardtOptimizer(graph, values).optimize();
        values = optimized;
        for (const gtsam::Values::ConstFiltered<gtsam::Pose2>::KeyValuePair& kv: values.filter<gtsam::Pose2>())
        {
            kf_list[kv.key].pose = gtsam2ROSPose2(kv.value);
        }
    }

    void publishCurrentGraph()
    {
        graph_slam::Graph graph;
        graph.factors = factor_list;
        graph.keyframes = kf_list;
        pub_graph.publish(graph);
    }

};



#endif
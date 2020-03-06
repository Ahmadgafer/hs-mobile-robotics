#ifndef __HSU_MOBILE_ROBOTICS_HH_VISUALIZER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_VISUALIZER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/Graph.h>
#include <pcl_ros/transforms.h>

#include <graph_slam/conversion_util.hpp>


class Visualizer
{
public:
    Visualizer(const ros::NodeHandle &_nh) : nh(_nh)
    {
        pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("smoothed_pointcloud", 10);
        pub_markers = nh.advertise<visualization_msgs::MarkerArray>("graph_visualization", 10);
        sub_graph = nh.subscribe("graph", 5, &Visualizer::visualizeCallBack, this);
    }

    ~Visualizer()
    {
    }

    void run()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_pointcloud;
    ros::Publisher pub_markers;
    ros::Subscriber sub_graph;
    int counter = 0;
    int counter2 = 0;
    const int odometry_factors_id = 12934;
    const int loop_closure_factors_id = 832749;
    const int key_frames_id = 329487;
    const int connect_kf_id = 29384;

    void visualizeCallBack(graph_slam::Graph msg)
    {
        auto kfs = msg.keyframes;
        auto fs = msg.factors;
        visualization_msgs::MarkerArray arr;
        // arr.markers.push_back(odometryFactorLines(kfs, fs));
        arr.markers.push_back(loopClosureFactorLines(kfs, fs));
        arr.markers.push_back(pointsOfKFs(kfs));    
        arr.markers.push_back(connectKFs(kfs, fs));
        pub_markers.publish(arr);
        publishPointCloudFromGraph(msg);
    }

    sensor_msgs::PointCloud2 getTransformedPointCloud(graph_slam::KeyFrame &kf)
    {
        sensor_msgs::PointCloud2 pc_orig = kf.point_cloud;
        Eigen::Matrix4f transform = conversion_util::makeTransform(kf.pose);
        sensor_msgs::PointCloud2 pc_transformed = kf.point_cloud;
        pcl_ros::transformPointCloud(transform, pc_orig, pc_transformed);
        pc_transformed.header.frame_id = "map";
        return pc_transformed;

    }

    void publishPointCloudFromGraph(graph_slam::Graph &msg) 
    {
        sensor_msgs::PointCloud2 pc;
        pc.header.frame_id = "map";
        for (graph_slam::KeyFrame kf: msg.keyframes)
        {
            sensor_msgs::PointCloud2 pc_ = getTransformedPointCloud(kf);
            pcl::concatenatePointCloud(pc, pc_, pc);
        }
        pub_pointcloud.publish(pc);
    }

    visualization_msgs::Marker pointsOfKFs(std::vector<graph_slam::KeyFrame> &kfs)
    {
        visualization_msgs::Marker m;
        m.header.seq = counter;
        m.header.frame_id = "map";
        m.id = key_frames_id;
        m.type = visualization_msgs::Marker::POINTS;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1;
        m.color.a = 1;
        m.color.b = 1;
        m.scale.x = 3e-2;
        m.scale.y = 3e-2;
        for (graph_slam::KeyFrame kf : kfs)
        {
            geometry_msgs::Point p;
            p.x = kf.pose.x;
            p.y = kf.pose.y;
            m.points.push_back(p);
        }
        return m;
    }

    geometry_msgs::Point getFactorEnd(graph_slam::KeyFrame source, graph_slam::Factor factor)
    {
        geometry_msgs::Point p;
        p.x = source.pose.x;
        p.y = source.pose.y;
        double theta = source.pose.theta;
        double dx = factor.measurement.x;
        double dy = factor.measurement.y;
        p.x += dx * cos(theta) - dy * sin(theta);
        p.y += dx * sin(theta) + dy * cos(theta);
        return p;
    }

    visualization_msgs::Marker connectKFs(std::vector<graph_slam::KeyFrame> &kfs, std::vector<graph_slam::Factor> &fs)
    {
        visualization_msgs::Marker m;
        m.header.seq = counter;
        m.header.frame_id = "map";
        m.id = connect_kf_id;
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1;
        m.color.a = 1.0;
        m.color.r = 1.0;
        m.scale.x = 1e-2;
        for (graph_slam::Factor f : fs)
        {
            if (f.type != "odometry")
            {
                continue;
            }
            graph_slam::KeyFrame kf1 = kfs[f.id1];
            graph_slam::KeyFrame kf2 = kfs[f.id2];
            geometry_msgs::Point p1;
            p1.x = kf1.pose.x;
            p1.y = kf1.pose.y;
            geometry_msgs::Point p2;
            p2.x = kf2.pose.x;
            p2.y = kf2.pose.y;
            m.points.push_back(p1);
            m.points.push_back(p2);
        }

        return m;
    }

    visualization_msgs::Marker odometryFactorLines(std::vector<graph_slam::KeyFrame> &kfs, std::vector<graph_slam::Factor> &fs)
    {
        visualization_msgs::Marker m;
        m.header.seq = counter;
        m.header.frame_id = "map";
        m.id = odometry_factors_id;
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1;
        m.color.a = 0.8;
        m.color.r = 0.5;
        m.color.g = 0.5;
        m.scale.x = 1e-2;
        for (graph_slam::Factor f : fs)
        {
            if (f.type != "odometry")
            {
                continue;
            }
            graph_slam::KeyFrame kf1 = kfs[f.id1];
            graph_slam::KeyFrame kf2 = kfs[f.id2];
            geometry_msgs::Point p1;
            p1.x = kf1.pose.x;
            p1.y = kf1.pose.y;
            geometry_msgs::Point p2 = getFactorEnd(kf1, f);
            m.points.push_back(p1);
            m.points.push_back(p2);
        }

        return m;
    }

    visualization_msgs::Marker loopClosureFactorLines(std::vector<graph_slam::KeyFrame> &kfs, std::vector<graph_slam::Factor> &fs)
    {
        visualization_msgs::Marker m;
        m.header.seq = counter;
        m.header.frame_id = "map";
        m.id = loop_closure_factors_id;
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1;
        m.color.a = 1;
        m.color.g = 1;
        m.scale.x = 1e-2;
        for (graph_slam::Factor f : fs)
        {
            if (f.type != "loop_closure")
            {
                continue;
            }
            graph_slam::KeyFrame kf1 = kfs[f.id1];
            graph_slam::KeyFrame kf2 = kfs[f.id2];
            geometry_msgs::Point p1;
            p1.x = kf1.pose.x;
            p1.y = kf1.pose.y;
            geometry_msgs::Point p2 = getFactorEnd(kf1, f);
            // p2.x = kf2.pose.x;
            // p2.y = kf2.pose.y;
            m.points.push_back(p1);
            m.points.push_back(p2);
        }

        return m;
    }
};

#endif

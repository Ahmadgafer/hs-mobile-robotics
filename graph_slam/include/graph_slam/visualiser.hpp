#ifndef __HSU_MOBILE_ROBOTICS_HH_VISUALISER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_VISUALISER_HPP

#include <cstring>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_broadcaster.h>

#include <graph_slam/Factor.h>
#include <graph_slam/FactorVis.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/Graph.h>
#include <graph_slam/AddNewFactor.h>
#include <graph_slam/AddNewKF.h>
#include <graph_slam/GetLastKF.h>
#include <graph_slam/GetNearestKF.h>
#include <graph_slam/conversion_util.hpp>

using namespace std;

class Visualiser
{
public:
    Visualiser(const ros::NodeHandle &_nh) : nh(_nh)
    {
        // sub_kf = nh.subscribe("kf", 5, &Visualiser::kfVisCallback, this);
        // sub_factor = nh.subscribe("factor_kfs", 5, &Visualiser::factorVisCallback, this);
        sub_graph = nh.subscribe("graph", 5, &Visualiser::graphVisCallback, this);      
        kf_vis_pub = nh.advertise<visualization_msgs::Marker>("kf_vis", 1000, this);
        factor_vis_pub = nh.advertise<visualization_msgs::Marker>("factor_vis", 1000, this); 
        laser_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_point_cloud", 1000, this);
        point_cloud_map_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_map", 1000, this);
    }

    ~Visualiser()
    {
    }

    void run()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber sub_kf;
    ros::Subscriber sub_factor;
    ros::Subscriber sub_graph;
    ros::Publisher kf_vis_pub;
    ros::Publisher factor_vis_pub;
    ros::Publisher laser_point_cloud_pub;
    ros::Publisher point_cloud_map_pub;

    tf::TransformBroadcaster tf_br;
    tf::Transform kf_transform;

    vector<graph_slam::KeyFrame> key_frames;
    vector<graph_slam::Factor> factors;

    void graphVisCallback(const graph_slam::Graph& graph)
    { 
        sensor_msgs::PointCloud2 point_cloud_map;
        key_frames = graph.keyframes;
        factors = graph.factors;  
        for (int i = 0; i < key_frames.size(); i++) {
            kfVis(key_frames[i]);
            point_cloud_map = concatPointCloud(point_cloud_map, key_frames[i]);
            point_cloud_map_pub.publish(point_cloud_map);
            // kf_transform.setOrigin(tf::Vector3(key_frames[i].pose.x, key_frames[i].pose.y, 0.0));
            // tf::Quaternion q;
            // q.setRPY(0, 0, key_frames[i].pose.theta);
            // kf_transform.setRotation(q);
            // tf_br.sendTransform(tf::StampedTransform(kf_transform, key_frames[i].stamp, "odom", "kf_" + to_string(key_frames[i].id)));
            // laser_point_cloud_pub.publish(key_frames[i].point_cloud);
        }
        for (int j = 0; j < factors.size(); j++) {
            factorVis(factors[j], j);
        }
    }

    void kfVis(graph_slam::KeyFrame& kf)
    {   
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = kf.stamp;
        marker.ns = "graph_slam_kfs";
        marker.id = kf.id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        tf::Quaternion tf_q;
        tf_q.setRPY(0.0, 0.0,  kf.pose.theta);
        geometry_msgs::Quaternion q;
        tf::quaternionTFToMsg(tf_q, q);
        marker.pose.position.x = kf.pose.x;
        marker.pose.position.y = kf.pose.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = q.z;
        marker.pose.orientation.w = q.w;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 0 / 255.;
        marker.color.g =  204 / 255.;
        marker.color.b = 204 / 255.;        
        marker.lifetime = ros::Duration();
        kf_vis_pub.publish(marker);
    }

    void factorVis(graph_slam::Factor& factor, int ind)
    {   
        visualization_msgs::Marker marker;
        geometry_msgs::Point point_last;
        geometry_msgs::Point point_new;
        point_last.x = key_frames[factor.id1].pose.x;
        point_last.y = key_frames[factor.id1].pose.y;
        point_new.x = key_frames[factor.id2].pose.x;
        point_new.y = key_frames[factor.id2].pose.y;
        marker.header.frame_id = "/odom";
        marker.ns = "graph_slam_factors";
        marker.id = ind;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.points.push_back(point_last);
        marker.points.push_back(point_new);
        marker.scale.x = 0.005;
        marker.scale.y = 0.01;
        marker.scale.z = 0;
        marker.color.a = 1.0;
        if (factor.type == "between" ) {
            marker.color.r = 0 / 255.;
            marker.color.g = 0 / 255.;
            marker.color.b = 0 / 255.;
        } else {
            marker.color.r = 255 / 255.;
            marker.color.g = 255 / 255.;
            marker.color.b = 0 / 255.;
        }
                
        marker.lifetime = ros::Duration();
        kf_vis_pub.publish(marker);
    }    

    void kfVisCallback(const graph_slam::KeyFrame::ConstPtr& kf_data)
    {   
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/odom";
        marker.header.stamp = kf_data->stamp;
        marker.ns = "graph_slam_kfs";
        marker.id = kf_data->id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        tf::Quaternion tf_q;
        tf_q.setRPY(0.0, 0.0,  kf_data->pose.theta);
        geometry_msgs::Quaternion q;
        tf::quaternionTFToMsg(tf_q, q);
        marker.pose.position.x = kf_data->pose.x;
        marker.pose.position.y = kf_data->pose.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = q.z;
        marker.pose.orientation.w = q.w;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0 / 255;
        marker.color.g =  0 / 255;
        marker.color.b = 255 / 255;        
        marker.lifetime = ros::Duration();
        kf_vis_pub.publish(marker);
    }

    void factorVisCallback(const graph_slam::FactorVis::ConstPtr& factor_data)
    {   
        visualization_msgs::Marker marker;
        geometry_msgs::Point point_last;
        geometry_msgs::Point point_new;
        point_last.x = factor_data->pose_last.x;
        point_last.y = factor_data->pose_last.y;
        point_new.x = factor_data->pose_new.x;
        point_new.y = factor_data->pose_new.y;
        marker.header.frame_id = "/odom";
        marker.header.stamp = factor_data->stamp;
        marker.ns = "graph_slam_factors";
        marker.id = factor_data->id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.points.push_back(point_last);
        marker.points.push_back(point_new);
        marker.scale.x = 0.005;
        marker.scale.y = 0.01;
        marker.scale.z = 0;
        marker.color.a = 1.0;
        marker.color.r = 172 / 255;
        marker.color.g = 73 / 255;
        marker.color.b = 245 / 255;        
        marker.lifetime = ros::Duration();
        kf_vis_pub.publish(marker);
    }
};

#endif
#ifndef __HSU_MOBILE_ROBOTICS_HH_VISUALIZER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_VISUALIZER_HPP

#include <ros/ros.h>

#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <eigen3/Eigen/Core>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <graph_slam/conversion_util.hpp>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

class Visualizer
{
public:
    Visualizer(const ros::NodeHandle &_nh): nh(_nh)
    {
        graph_visualizer = nh.advertise<visualization_msgs::MarkerArray>( "/graph_visualizer_pub", 100 );
        pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_publisher", 1000);
        pointcloud_publisher.publish(sensor_msgs::PointCloud2());
    }

    ~Visualizer()
    {
    }

    void updateKF(const graph_slam::KeyFrame& kf, string name="robot", string parent = "world"){
        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = parent;
        static_transformStamped.child_frame_id = name;
        static_transformStamped.transform.translation.x = kf.pose.x;
        static_transformStamped.transform.translation.y = kf.pose.y;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, kf.pose.theta);
        static_transformStamped.transform.rotation.x = quat.x();
        static_transformStamped.transform.rotation.y = quat.y();
        static_transformStamped.transform.rotation.z = quat.z();
        static_transformStamped.transform.rotation.w = quat.w();
        static_broadcaster.sendTransform(static_transformStamped);
        
    }

    void updateGraph(const vector<graph_slam::KeyFrame>& keyframes, const vector<graph_slam::Factor>& factors){
        int id = 0;
        visualization_msgs::MarkerArray markerArray;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.ns = "";
        marker.lifetime = ros::Duration(0);
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.z = 0;


        marker.action = visualization_msgs::Marker::DELETEALL;
        markerArray.markers.push_back(marker);
        graph_visualizer.publish(markerArray);      
        marker.action = visualization_msgs::Marker::ADD;
        markerArray.markers.pop_back();

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.type = visualization_msgs::Marker::SPHERE;
        
        for(int i = 0; i < (int)keyframes.size(); i++){
            marker.id = id++;
            marker.text = "graph_node_" + to_string(i);
            marker.pose.position.x = keyframes[i].pose.x;
            marker.pose.position.y = keyframes[i].pose.y;
            tf2::Quaternion quat;
            quat.setRPY(0, 0, keyframes[i].pose.theta);
            marker.pose.orientation.x = quat.getX();
            marker.pose.orientation.y = quat.getY();
            marker.pose.orientation.z = quat.getZ();
            marker.pose.orientation.w = quat.getW();
            markerArray.markers.push_back(marker);
        }

        marker.scale.x = 1;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.type = visualization_msgs::Marker::ARROW;
        for(int i = 0 ; i < (int)factors.size(); i++){
            if (factors[i].id1 == -1)
                continue;
            marker.id = id++;
            marker.text = "factor_" + to_string(i);
            geometry_msgs::Pose2D measurement = factors[i].measurement;
            marker.scale.x = sqrt(measurement.x*measurement.x + measurement.y*measurement.y);
            marker.pose.position.x = keyframes[factors[i].id1].pose.x;
            marker.pose.position.y = keyframes[factors[i].id1].pose.y;
            tf2::Quaternion quat;
            quat.setRPY(0, 0, atan2(keyframes[factors[i].id2].pose.y - keyframes[factors[i].id1].pose.y,
            keyframes[factors[i].id2].pose.x - keyframes[factors[i].id1].pose.x));

            marker.pose.orientation.x = quat.getX();
            marker.pose.orientation.y = quat.getY();
            marker.pose.orientation.z = quat.getZ();
            marker.pose.orientation.w = quat.getW();

            if (factors[i].id1 + 1 == factors[i].id2){
                if (factors[i].type == "between"){
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }
                else {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                }
            }
            else{
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            
            markerArray.markers.push_back(marker);
        }
        graph_visualizer.publish(markerArray);        
    }

    void updatePointclouds(const vector<graph_slam::KeyFrame> &keyframes){
        pointcloud_publisher.publish(merge_pointcloud(keyframes));
    }
private:
    ros::NodeHandle nh;
    ros::Publisher graph_visualizer;
    ros::Publisher pointcloud_publisher;
    
};

#endif

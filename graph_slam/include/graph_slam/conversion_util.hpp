#ifndef __HSU_MOBILE_ROBOTICS_HH_CONVERSION_UTIL_HPP
#define __HSU_MOBILE_ROBOTICS_HH_CONVERSION_UTIL_HPP

#include <cmath>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Pose2D.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>


static sensor_msgs::PointCloud2 scan_to_pointcloud(sensor_msgs::LaserScan input) {

    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 output;
    projector.projectLaser(input, output);
    return output;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr format_pointcloud(sensor_msgs::PointCloud2 input) {
    pcl::PCLPointCloud2 pcl2_pointcloud;
    pcl_conversions::toPCL(input, pcl2_pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl2_pointcloud, *output);
    return output;
}

static geometry_msgs::Pose2D transfrom_to_pose(Eigen::Matrix4f t_matrix){
    geometry_msgs::Pose2D output;
    // t_matrix().transposeInPlace();
    output.x = t_matrix(0, 3);
    output.y = t_matrix(1, 3);
    output.theta = atan2( t_matrix(1, 0) , t_matrix(0, 0) );
    return output;
}

static Eigen::Matrix4f pose_to_transform(const geometry_msgs::Pose2D& pose_2d) {
    Eigen::Matrix4f transform;
    transform.setIdentity();
    float cos_th = cos(pose_2d.theta);
    float sin_th = sin(pose_2d.theta);
    transform(0,0) = cos_th;
    transform(0,1) = -sin_th;
    transform(1,0) = sin_th;
    transform(1,1) = cos_th;
    transform(0,3) = pose_2d.x;
    transform(1,3) = pose_2d.y;
    return transform;
}

static sensor_msgs::PointCloud2 transformPC(const graph_slam::KeyFrame kf) {
    sensor_msgs::PointCloud2 tf_point_cloud;
    Eigen::Matrix4f tf_mat = pose_to_transform(kf.pose);
    pcl_ros::transformPointCloud(tf_mat, kf.point_cloud, tf_point_cloud);
    return tf_point_cloud;
}

static sensor_msgs::PointCloud2 concatPointCloud(sensor_msgs::PointCloud2& c_pc, const graph_slam::KeyFrame kf) {
    sensor_msgs::PointCloud2 c_pc_ = c_pc;
    sensor_msgs::PointCloud2 pc = transformPC(kf);
    pcl::concatenatePointCloud(c_pc_, pc, c_pc);
    return c_pc;
}

struct Aligned
{
    bool converged = false;
    float fitness;
    Eigen::Matrix4f last_T_new;
    geometry_msgs::Pose2D delta;
    double covariance[9] = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.1 };
};

static Aligned gicp_register(const sensor_msgs::PointCloud2 input_1, const sensor_msgs::PointCloud2 input_2, Eigen::Matrix4f& transform){
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setUseReciprocalCorrespondences(true);
    icp.setMaximumIterations(100); // ICP example 50
    icp.setMaxCorrespondenceDistance(0.25); // ICP example 0.
    icp.setTransformationEpsilon(1e-8); // ICP example 1e-8
    icp.setEuclideanFitnessEpsilon(0.1); // ICP example 1
    icp.getConvergeCriteria()->setMaximumIterationsSimilarTransforms(10);

    // assign inputs
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = format_pointcloud(input_1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = format_pointcloud(input_2);
    icp.setInputSource(pointcloud_1);
    icp.setInputTarget(pointcloud_2);

    // std::cout << "PCL size " << pointcloud_1->size() << " " << pointcloud_2->size() << std::endl;

    // align
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*pointcloud_transform, transform);

    Aligned output;
    output.converged = icp.hasConverged();
    output.fitness = icp.getFitnessScore();

    if (icp.hasConverged())
    {
        Eigen::Matrix4f final_transform = icp.getFinalTransformation();
        // Get transformation Delta and compute its covariance
        output.last_T_new = final_transform;
        // std::cout << "last_T_new " << final_transform << std::endl;
        output.delta = transfrom_to_pose(final_transform);
    }
    return output;
}

#endif
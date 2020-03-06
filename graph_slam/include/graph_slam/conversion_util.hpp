#ifndef __HSU_MOBILE_ROBOTICS_HH_CONVERSION_UTIL_HPP
#define __HSU_MOBILE_ROBOTICS_HH_CONVERSION_UTIL_HPP

#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/point_cloud.h>

namespace conversion_util
{
static sensor_msgs::PointCloud2 scanToROSPointcloud(sensor_msgs::LaserScan input)
{

    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 output;
    projector.projectLaser(input, output);

    return output;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr rosPointcloudToPCL(sensor_msgs::PointCloud2 input)
{

    pcl::PCLPointCloud2 pcl2_pointcloud;
    pcl_conversions::toPCL(input, pcl2_pointcloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl2_pointcloud, *output);

    return output;
}
static geometry_msgs::Pose2D makeDelta(const Eigen::Matrix4f &T)
{
    geometry_msgs::Pose2D Delta;
    Delta.x = T(0, 3);
    Delta.y = T(1, 3);
    Delta.theta = atan2(T(1, 0), T(0, 0));
    return Delta;
}

static Eigen::Matrix4f makeTransform(const geometry_msgs::Pose2D &Delta)
{
    Eigen::Matrix4f transform;
    transform.setIdentity();
    float cos_th = cos(Delta.theta);
    float sin_th = sin(Delta.theta);
    transform(0, 0) = cos_th;
    transform(0, 1) = -sin_th;
    transform(1, 0) = sin_th;
    transform(1, 1) = cos_th;
    transform(0, 3) = Delta.x;
    transform(1, 3) = Delta.y;
    return transform;
}

static Eigen::Matrix4f invertTransform(Eigen::Matrix4f mat_)
{
    std::cout << mat_ << std::endl;
    Eigen::Matrix4f mat(mat_);
    std::swap(mat(0, 1), mat(1, 0));
    float x = mat(0, 3);
    float y = mat(1, 3);
    mat(0, 3) = -(mat(0, 0) * x + mat(0, 1) * y);
    mat(1, 3) = -(mat(1, 0) * x + mat(1, 1) * y);

    std::cout << mat << std::endl;
    return mat;

}

} // namespace conversion_util

#endif
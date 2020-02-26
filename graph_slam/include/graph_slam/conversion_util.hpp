#ifndef __HSU_MOBILE_ROBOTICS_HH_CONVERSION_UTIL_HPP
#define __HSU_MOBILE_ROBOTICS_HH_CONVERSION_UTIL_HPP

#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/point_cloud.h>


sensor_msgs::PointCloud2 scan_to_pointcloud(sensor_msgs::LaserScan input) {

    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 output;
    projector.projectLaser(input, output);

    return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr format_pointcloud(sensor_msgs::PointCloud2 input) {

    pcl::PCLPointCloud2 pcl2_pointcloud;
    pcl_conversions::toPCL(input, pcl2_pointcloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl2_pointcloud, *output);

    return output;
}

#endif
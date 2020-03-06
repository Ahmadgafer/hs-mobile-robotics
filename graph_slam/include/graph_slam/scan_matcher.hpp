#ifndef __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP
#define __HSU_MOBILE_ROBOTICS_HH_SCAN_MATCHER_HPP

#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Time.h>

#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_eigen/tf2_eigen.h>


#include <graph_slam/Factor.h>
#include <graph_slam/KeyFrame.h>
#include <graph_slam/AddNewFactor.h>
#include <graph_slam/AddNewKF.h>
#include <graph_slam/GetLastKF.h>
#include <graph_slam/GetNearestKF.h>

#include <graph_slam/conversion_util.hpp>

void debug_convergence_state(pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState state)
{
    if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_NOT_CONVERGED)
    {
        std::cout << "CONVERGENCE_CRITERIA_NOT_CONVERGED" << std::endl;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_ITERATIONS)
    {
        std::cout << "CONVERGENCE_CRITERIA_ITERATIONS" << std::endl;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_TRANSFORM)
    {
        std::cout << "CONVERGENCE_CRITERIA_TRANSFORM" << std::endl;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_ABS_MSE)
    {
        std::cout << "CONVERGENCE_CRITERIA_ABS_MSE" << std::endl;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_REL_MSE)
    {
        std::cout << "CONVERGENCE_CRITERIA_REL_MSE" << std::endl;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES)
    {
        std::cout << "CONVERGENCE_CRITERIA_NO_CORRESPONDENCES" << std::endl;        
    }
}

bool check_convergence(pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState state)
{
    if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_NOT_CONVERGED)
    {
        return false;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_ITERATIONS)
    {
        return false;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_TRANSFORM)
    {
        return true;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_ABS_MSE)
    {
        return true;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_REL_MSE)
    {
        return true;
    }
    else if (state == pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES)
    {
        return false;
    }

}

struct Alignment
{
    pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState convergence_state;
    bool converged;
    double fitness;
    Eigen::Matrix4f transform;
    geometry_msgs::Pose2D delta;
};

class ScanMatcher
{
public:
    ScanMatcher(const ros::NodeHandle &_nh) : nh(_nh)
    {
        cli_add_new_factor = nh.serviceClient<graph_slam::AddNewFactor>("add_new_factor");
        cli_add_new_kf = nh.serviceClient<graph_slam::AddNewKF>("add_new_kf");
        cli_get_last_kf = nh.serviceClient<graph_slam::GetLastKF>("get_last_kf");
        cli_get_nearest_kf = nh.serviceClient<graph_slam::GetNearestKF>("get_nearest_kf");

        sub_laser_scan = nh.subscribe("scan", 10, &ScanMatcher::laserScanCallback, this);
        counter = 0;
        trf_prior.setIdentity();
        nh.param<double>("new_kf_time", new_kf_time, 1.);
        nh.param<double>("kf_len_threshold", kf_len_threshold, 0.25);
        nh.param<double>("kf_angle_threshold", kf_angle_threshold, 0.785);
        nh.param<double>("similarity_threshold_new_kf", similarity_threshold_new_kf, 0.03);
        nh.param<double>("similarity_threshold_loop_closure", similarity_threshold_loop_closure, 0.01);
        nh.param<int>("number_of_keyframes_to_ignore_for_finding_nearest", to_start_finding_loop_closure, 5);
        tf_listener.reset(new tf2_ros::TransformListener(buffer));
    }

    ~ScanMatcher()
    {
    }

    void run()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber sub_laser_scan;

    ros::ServiceClient cli_add_new_factor;
    ros::ServiceClient cli_add_new_kf;
    ros::ServiceClient cli_get_last_kf;
    ros::ServiceClient cli_get_nearest_kf;
    tf2_ros::Buffer buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    Eigen::Matrix4f trf_prior;
    int counter;
    int to_start_finding_loop_closure;
    sensor_msgs::LaserScan latest;
    double similarity_threshold_new_kf;
    double similarity_threshold_loop_closure;
    double new_kf_time;
    double kf_len_threshold;
    double kf_angle_threshold;

    bool addNewFactor(graph_slam::Factor factor)
    {
        graph_slam::AddNewFactor srv;
        srv.request.factor = factor;
        bool is_ok = cli_add_new_factor.call(srv);
        return is_ok;
    }

    graph_slam::KeyFrame addNewKF(graph_slam::KeyFrame new_kf)
    {
        graph_slam::AddNewKF srv;
        srv.request.new_kf = new_kf;
        bool is_ok = cli_add_new_kf.call(srv);
        return srv.response.kf;
    }

    graph_slam::KeyFrame getLastKF()
    {
        graph_slam::GetLastKF srv;
        bool is_ok = cli_get_last_kf.call(srv);
        return srv.response.last_kf;
    }

    graph_slam::KeyFrame getNearestKF(graph_slam::KeyFrame kf)
    {
        graph_slam::GetNearestKF srv;
        srv.request.kf = kf;
        bool is_ok = cli_get_nearest_kf.call(srv);
        return srv.response.nearest_kf;
    }

    double rostimeToDouble(ros::Time time)
    {
        return double(time.sec) + 1e-9 * (time.nsec);
    }

    bool voteForKF(Alignment align, graph_slam::KeyFrame last_kf, sensor_msgs::LaserScan msg)
    {
        return (align.fitness > similarity_threshold_new_kf) || (align.delta.x *align.delta.x + align.delta.y * align.delta.y >= kf_len_threshold * kf_len_threshold || abs(align.delta.theta) > kf_angle_threshold);
    }

    void laserScanStoreCallback(const sensor_msgs::LaserScan &msg)
    {
        latest = msg;
    }

    void laserScanCallback(const sensor_msgs::LaserScan msg)
    {
        if (counter < 3)
        {
            counter++;
            return;
        }
        if (counter == 3)
        {
            addNewKF(composeInitialKeyFrame(msg));
            counter++;
            return;
        }

        sensor_msgs::PointCloud2 point_cloud = conversion_util::scanToROSPointcloud(msg);
        graph_slam::KeyFrame last_kf = getLastKF();
        Alignment align = icpRegister(point_cloud, last_kf.point_cloud, trf_prior);
        // std::cout << counter << " " << align.fitness << std::endl;
        // std::cout << (rostimeToDouble(msg.header.stamp) - rostimeToDouble(last_kf.stamp)) << std::endl;

        if (!align.converged || !check_convergence(align.convergence_state))
        // if (!align.converged)
        {
            ROS_INFO("gicp didn't converge, fitness: %lf", align.fitness);
            return;
        }

        if (voteForKF(align, last_kf, msg))
        {
            std::cout << counter << " " << align.fitness << std::endl;
            debug_convergence_state(align.convergence_state);
            // Make new keyframe, factor, try to find loop closure
            graph_slam::KeyFrame new_kf = composeKeyFrame(msg, point_cloud, align, last_kf);
            graph_slam::KeyFrame new_kf_with_id = addNewKF(new_kf);
            graph_slam::Factor new_factor = composeFactor(last_kf, new_kf_with_id, align, "odometry");
            addNewFactor(new_factor);
            // addNewFactor(composeTFFactor(last_kf, new_kf_with_id));
            if (counter > to_start_finding_loop_closure + 1)
            {
                graph_slam::KeyFrame nearest_kf = getNearestKF(new_kf_with_id);
                // std::cout << nearest_kf << std::endl;
                Eigen::Matrix4f trf_estimated = estimateTrf(nearest_kf, new_kf);
                // Eigen::Matrix4f trf_estimated = estimateTrf(new_kf, nearest_kf);
                std::cout << trf_estimated << std::endl;
                Alignment align = icpRegister(new_kf_with_id.point_cloud, nearest_kf.point_cloud, trf_estimated);
                std::cout << "loop closure\n";
                std::cout << counter << " " << align.fitness << std::endl;
                debug_convergence_state(align.convergence_state);
                if (align.fitness < similarity_threshold_loop_closure && check_convergence(align.convergence_state))
                {
                    graph_slam::Factor new_lc_factor = composeFactor(nearest_kf, new_kf_with_id, align, "loop_closure");
                    addNewFactor(new_lc_factor);
                }
            }

            counter++;
            trf_prior.setIdentity();
        }
        else
        {
            trf_prior = align.transform;
        }
    }

    geometry_msgs::Pose2D getPos(ros::Time time)
    {
        geometry_msgs::TransformStamped transform = buffer.lookupTransform("odom", "base_scan", time, ros::Duration(0.1));
        geometry_msgs::Pose2D pose;
        pose.x = transform.transform.translation.x;
        pose.y = transform.transform.translation.y;
        pose.theta = atan2(transform.transform.rotation.z, transform.transform.rotation.w) * 2;
        return pose;
    }

    geometry_msgs::Pose2D getDelta(ros::Time last, ros::Time now)
    {
        Eigen::Affine3d t_to_l, t_to_n;
        geometry_msgs::TransformStamped transform_to_last = buffer.lookupTransform("odom", "base_scan", last);
        geometry_msgs::TransformStamped transform_to_now = buffer.lookupTransform("odom", "base_scan", now);
        t_to_l = tf2::transformToEigen(transform_to_last);
        t_to_n = tf2::transformToEigen(transform_to_now);
        Eigen::Affine3d l_to_n = t_to_l.inverse() * t_to_n;
        geometry_msgs::Pose pose3d = tf2::toMsg(l_to_n);

        
        geometry_msgs::Pose2D pose;
        pose.x = pose3d.position.x;
        pose.y = pose3d.position.y;
        pose.theta = atan2(pose3d.orientation.z, pose3d.orientation.w) * 2;
        return pose;

    }

    Eigen::Matrix4f estimateTrf(graph_slam::KeyFrame &nearest_kf, graph_slam::KeyFrame &new_kf)
    {
        std::cout << nearest_kf.pose << std::endl;
        std::cout << new_kf.pose << std::endl;
        return conversion_util::invertTransform(conversion_util::makeTransform(nearest_kf.pose)) * conversion_util::makeTransform(new_kf.pose);
    }

    graph_slam::KeyFrame composeInitialKeyFrame(const sensor_msgs::LaserScan &scan)
    {
        graph_slam::KeyFrame kf;
        kf.scan = scan;
        kf.point_cloud = conversion_util::scanToROSPointcloud(scan);
        kf.stamp = scan.header.stamp;
        kf.pose = getPos(scan.header.stamp);
        return kf;
    }

    graph_slam::KeyFrame composeKeyFrame(const sensor_msgs::LaserScan &scan, const sensor_msgs::PointCloud2 &point_cloud, Alignment &align, graph_slam::KeyFrame &last_kf)
    {
        graph_slam::KeyFrame kf;
        kf.stamp = scan.header.stamp;
        kf.scan = scan;
        kf.point_cloud = point_cloud;
        Eigen::Matrix4f to_last_kf = conversion_util::makeTransform(last_kf.pose);
        Eigen::Matrix4f to_new_kf = to_last_kf * align.transform;
        kf.pose = conversion_util::makeDelta(to_new_kf);
        return kf;
    }

    graph_slam::Factor composeFactor(graph_slam::KeyFrame last_kf, graph_slam::KeyFrame new_kf, Alignment align, std::string type)
    {
        graph_slam::Factor factor;
        factor.id1 = last_kf.id;
        factor.id2 = new_kf.id;
        factor.type = type;
        factor.measurement = align.delta;
        // std::cout << "scan" << std::endl;
        // std::cout << factor.measurement << std::endl;
        return factor;
    }

    graph_slam::Factor composeTFFactor(graph_slam::KeyFrame last_kf, graph_slam::KeyFrame new_kf)
    {
        graph_slam::Factor factor;
        factor.id1 = last_kf.id;
        factor.id2 = new_kf.id;
        factor.type = "wheel_odometry";
        factor.measurement = getDelta(last_kf.stamp, new_kf.stamp);
        // std::cout << "wheel" << std::endl;
        // std::cout << factor.measurement << std::endl;

        return factor;
    }

    Alignment gicpRegister(const sensor_msgs::PointCloud2 input_1, const sensor_msgs::PointCloud2 input_2, Eigen::Matrix4f &transform)
    {
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = conversion_util::rosPointcloudToPCL(input_1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = conversion_util::rosPointcloudToPCL(input_2);
        gicp.setInputSource(pointcloud_1);
        gicp.setInputTarget(pointcloud_2);

        // align
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
        gicp.align(*pointcloud_transform, transform);

        Alignment output;
        output.convergence_state = gicp.getConvergeCriteria()->getConvergenceState();
        output.converged = gicp.hasConverged();
        output.fitness = gicp.getFitnessScore();

        if (gicp.hasConverged())
        {
            transform = gicp.getFinalTransformation();

            output.transform = transform;
            geometry_msgs::Pose2D transform_delta = conversion_util::makeDelta(transform);
            output.delta = transform_delta;
        }
        return output;
    }


    Alignment icpRegister(const sensor_msgs::PointCloud2 input_1, const sensor_msgs::PointCloud2 input_2, Eigen::Matrix4f &transform)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

        // gicp.setUseReciprocalCorrespondences(true);
        // gicp.setMaximumIterations(50); // ICP example 50
        // gicp.setMaxCorrespondenceDistance(0.05); // ICP example 0.05
        // gicp.setTransformationEpsilon(1e-8); // ICP example 1e-8
        // gicp.setEuclideanFitnessEpsilon(0.1); // ICP example 1
        // gicp.getConvergeCriteria()->setMaximumIterationsSimilarTransforms(10);
        gicp.setUseReciprocalCorrespondences(true);
        gicp.setMaximumIterations(200); // ICP example 50
        gicp.setMaxCorrespondenceDistance(0.1); // ICP 
        gicp.setEuclideanFitnessEpsilon(0.003); // ICP example 1
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = conversion_util::rosPointcloudToPCL(input_1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = conversion_util::rosPointcloudToPCL(input_2);
        gicp.setInputSource(pointcloud_1);
        gicp.setInputTarget(pointcloud_2);

        // align
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
        gicp.align(*pointcloud_transform, transform);

        Alignment output;
        output.convergence_state = gicp.getConvergeCriteria()->getConvergenceState();
        output.converged = gicp.hasConverged();
        output.fitness = gicp.getFitnessScore();

        if (gicp.hasConverged())
        {
            transform = gicp.getFinalTransformation();

            output.transform = transform;
            geometry_msgs::Pose2D transform_delta = conversion_util::makeDelta(transform);
            output.delta = transform_delta;
        }
        return output;
    }
};

#endif

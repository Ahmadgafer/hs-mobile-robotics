#ifndef __HSU_MOBILE_ROBOTICS_HH_CONVERSION_UTIL_HPP
#define __HSU_MOBILE_ROBOTICS_HH_CONVERSION_UTIL_HPP

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
#include <laser_geometry/laser_geometry.h>


#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_ros/point_cloud.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <graph_slam/KeyFrame.h>

using namespace std;

static sensor_msgs::PointCloud2 scan_to_pointcloud(sensor_msgs::LaserScan input) {
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 output;
    projector.projectLaser(input, output);
    output.header.frame_id = "world";
    return output;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr format_pointcloud(sensor_msgs::PointCloud2 input) {

    pcl::PCLPointCloud2 pcl2_pointcloud;
    pcl_conversions::toPCL(input, pcl2_pointcloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl2_pointcloud, *output);

    return output;
}

static sensor_msgs::PointCloud2 transform_pointcloud(sensor_msgs::PointCloud2 pc, geometry_msgs::Pose2D tf){
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta = tf.theta;
    transform (0,0) = cos(theta);
    transform (0,1) = -sin(theta);
    transform (1,0) = sin(theta);
    transform (1,1) = cos(theta);
    transform (0,3) = tf.x;
    transform (1,3) = tf.y;

    pcl::PointCloud<pcl::PointXYZ>::Ptr src = format_pointcloud(pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*src, *transformed_cloud, transform);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_cloud.get(),output);
    output.header.frame_id = "world";
    return output;
}


static sensor_msgs::PointCloud2 merge_pointcloud(const vector<graph_slam::KeyFrame>& pc) {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr p;
    sensor_msgs::PointCloud2 output;
    for(int i = 0; i < (int)pc.size(); i++){
        sensor_msgs::PointCloud2 temp;
        pcl::concatenatePointCloud(output,transform_pointcloud(pc[i].point_cloud, pc[i].pose),temp);
        output = temp;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr s = format_pointcloud(pc[i].point_cloud);
        // if(s->size() > 0)
        //     *p += *s;   
    }
    // if(p->size() > 0)
    //     pcl::toROSMsg(*p.get(),output);
    output.header.frame_id = "world";
    return output;
}
static geometry_msgs::Pose2D move_pose(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D t){
    tf::Transform t1,t2;
    t1.setOrigin(tf::Vector3(pose.x,pose.y,0));
    tf::Quaternion quat;
    quat.setRPY(0, 0, pose.theta);
    t1.setRotation(quat);

    t2.setOrigin(tf::Vector3(t.x,t.y,0));
    quat.setRPY(0, 0, t.theta);
    tf::Matrix3x3 R1(quat);
    
    t2.setRotation(quat);

    t1 = t1*t2;
    
    pose.x = t1.getOrigin().getX();
    pose.y = t1.getOrigin().getY();
    double yaw, pitch, roll;
    tf::Matrix3x3 mat(t1.getRotation());
    mat.getRPY(roll, pitch, yaw);

    pose.theta = double(yaw);

    return pose;
}


static geometry_msgs::Pose2D get_prior(geometry_msgs::Pose2D pose1, geometry_msgs::Pose2D pose2){
    tf::Transform t1,t2;
    t1.setOrigin(tf::Vector3(pose1.x,pose1.y,0));
    tf::Quaternion quat;
    quat.setRPY(0, 0, pose1.theta);
    t1.setRotation(quat);

    t2.setOrigin(tf::Vector3(pose2.x,pose2.y,0));
    quat.setRPY(0, 0, pose2.theta);
    t2.setRotation(quat);
    
    t1 = t1.inverse()*t2;
    
    pose1.x = t1.getOrigin().getX();
    pose1.y = t1.getOrigin().getY();
    double yaw, pitch, roll;
    tf::Matrix3x3 mat(t1.getRotation());
    mat.getRPY(roll, pitch, yaw);
    pose1.theta = double(yaw);

    return pose1;
}


struct Alignement {
    int converged;
    double fitness;
    Eigen::Matrix4f transform;
    geometry_msgs::Pose2D Delta;
};

static geometry_msgs::Pose2D make_Delta(Eigen::Matrix4f& transform){
    geometry_msgs::Pose2D pose;
    pose.x = transform(0,3);
    pose.y = transform(1,3);
    pose.theta = atan2(transform(1,0),transform(0,0));
    return pose;
}

static Eigen::Matrix4f make_tf(geometry_msgs::Pose2D pose){
    Eigen::Matrix4f transform;
    transform.setIdentity();
    transform(0,0) = cos(pose.theta);
    transform(1,1) = cos(pose.theta);
    transform(0,1) = -sin(pose.theta);
    transform(1,0) = sin(pose.theta);
    transform(0,3) = pose.x;
    transform(1,3) = pose.y;
    return transform;
}


static void debug_convergence_state(pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState state)
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

static bool check_convergence(pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState state)
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

static Alignement gicp_register(const sensor_msgs::PointCloud2 input_1, const sensor_msgs::PointCloud2 input_2, Eigen::Matrix4f& transform){
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    // pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    

    gicp.setUseReciprocalCorrespondences(true);
    gicp.setMaximumIterations(100); // ICP example 50
    gicp.setMaxCorrespondenceDistance(0.25); // ICP example 0.05
    gicp.setTransformationEpsilon(1e-8); // ICP example 1e-8
    gicp.setEuclideanFitnessEpsilon(0.1); // ICP example 1
    gicp.getConvergeCriteria()->setMaximumIterationsSimilarTransforms(10);
    
    

    
    //  gicp.setRotationEpsilon();
    //  gicp.setCorrespondenceRandomness();
    //  gicp.setMaximumOptimizerIterations(50);
    
    // assign inputs
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = format_pointcloud(input_1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = format_pointcloud(input_2);
    gicp.setInputSource(pointcloud_1);
    gicp.setInputTarget(pointcloud_2);
    // align
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
    gicp.align(*pointcloud_transform, transform);
    debug_convergence_state(gicp.getConvergeCriteria()->getConvergenceState());
    Alignement output;
    // output.convergence_state = gicp.getConvergeCriteria()->getConvergenceState();
    output.converged = gicp.hasConverged();
    output.fitness = gicp.getFitnessScore();

    // ROS_INFO("Alignement converged: (%d) with fitness: %f", output.converged, output.fitness);

    if (gicp.hasConverged())
    {
        transform = gicp.getFinalTransformation();
        // Get transformation Delta and compute its covariance
        output.transform = transform;
        output.Delta = make_Delta(transform);
        // Eigen::Matrix3d covariance_Delta = compute_covariance(sigma_xy, sigma_th);
        // output.Delta = create_Pose2DWithCovariance_msg(transform_Delta, covariance_Delta);
    }
    return output;
}

#endif

// #include "graph_slam/scan_matcher.hpp"

// ScanMatcher::ScanMatcher(const ros::NodeHandle &_nh) : nh(_nh)
// {
//     cli_add_new_factor = nh.serviceClient<graph_slam::AddNewFactor>("add_new_factor");
//     cli_add_new_kf = nh.serviceClient<graph_slam::AddNewKF>("add_new_kf");
//     cli_get_last_kf = nh.serviceClient<graph_slam::GetLastKF>("get_last_kf");
//     cli_get_nearest_kf = nh.serviceClient<graph_slam::GetNearestKF>("get_nearest_kf");

//     sub_laser_scan = nh.subscribe("scan", 5, &ScanMatcher::laserScanCallback, this);
//     counter = 0;
//     trf_prior.setIdentity();
//     nh.param<double>("new_kf_time", new_kf_time, 1.);
//     nh.param<double>("similarity_threshold_new_kf", similarity_threshold_new_kf, 0.03);
//     nh.param<double>("similarity_threshold_loop_closure", similarity_threshold_loop_closure, 0.01);
//     nh.param<int>("number_of_keyframes_to_ignore_for_finding_nearest", to_start_finding_loop_closure, 5);
//     tf_listener.reset(new tf2_ros::TransformListener(buffer));
// }

// ScanMatcher::~ScanMatcher()
// {
// }

// void ScanMatcher::run()
// {
//     ros::spin();
// }

// bool ScanMatcher::addNewFactor(graph_slam::Factor factor)
// {
//     graph_slam::AddNewFactor srv;
//     srv.request.factor = factor;
//     bool is_ok = cli_add_new_factor.call(srv);
//     return is_ok;
// }

// graph_slam::KeyFrame ScanMatcher::addNewKF(graph_slam::KeyFrame new_kf)
// {
//     graph_slam::AddNewKF srv;
//     srv.request.new_kf = new_kf;
//     bool is_ok = cli_add_new_kf.call(srv);
//     return srv.response.kf;
// }

// graph_slam::KeyFrame ScanMatcher::getLastKF()
// {
//     graph_slam::GetLastKF srv;
//     bool is_ok = cli_get_last_kf.call(srv);
//     return srv.response.last_kf;
// }

// graph_slam::KeyFrame ScanMatcher::getNearestKF(graph_slam::KeyFrame kf)
// {
//     graph_slam::GetNearestKF srv;
//     srv.request.kf = kf;
//     bool is_ok = cli_get_nearest_kf.call(srv);
//     return srv.response.nearest_kf;
// }

// double ScanMatcher::rostimeToDouble(ros::Time time)
// {
//     return double(time.sec) + 1e-9 * (time.nsec);
// }

// bool ScanMatcher::voteForKF(Alignment align, graph_slam::KeyFrame last_kf, sensor_msgs::LaserScan msg)
// {
//     return (align.fitness < similarity_threshold_new_kf) && ((rostimeToDouble(msg.header.stamp) - rostimeToDouble(last_kf.stamp)) > new_kf_time);
// }

// void ScanMatcher::laserScanStoreCallback(const sensor_msgs::LaserScan &msg)
// {
//     latest = msg;
// }

// void ScanMatcher::laserScanCallback(const sensor_msgs::LaserScan msg)
// {
//     if (counter < 1)
//     {
//         counter++;
//         return;
//     }
//     if (counter == 1)
//     {
//         addNewKF(composeInitialKeyFrame(msg));
//         counter++;
//         return;
//     }

//     sensor_msgs::PointCloud2 point_cloud = conversion_util::scanToROSPointcloud(msg);
//     graph_slam::KeyFrame last_kf = getLastKF();
//     Alignment align = gicpRegister(point_cloud, last_kf.point_cloud, trf_prior);
//     // std::cout << counter << " " << align.fitness << std::endl;
//     // std::cout << (rostimeToDouble(msg.header.stamp) - rostimeToDouble(last_kf.stamp)) << std::endl;

//     if (!align.converged)
//     {
//         return;
//     }

//     if (voteForKF(align, last_kf, msg))
//     {
//         // Make new keyframe, factor, try to find loop closure
//         graph_slam::KeyFrame new_kf = composeKeyFrame(msg, point_cloud, align, last_kf);
//         graph_slam::KeyFrame new_kf_with_id = addNewKF(new_kf);
//         graph_slam::Factor new_factor = composeFactor(last_kf, new_kf_with_id, align, "odometry");
//         addNewFactor(new_factor);
//         if (counter > to_start_finding_loop_closure + 1)
//         {
//             graph_slam::KeyFrame nearest_kf = getNearestKF(new_kf_with_id);
//             // std::cout << nearest_kf << std::endl;
//             Eigen::Matrix4f trf_estimated = estimateTrf(nearest_kf, new_kf);
//             Alignment align = gicpRegister(new_kf_with_id.point_cloud, nearest_kf.point_cloud, trf_estimated);
//             if (align.converged && align.fitness < similarity_threshold_loop_closure)
//             {
//                 graph_slam::Factor new_lc_factor = composeFactor(nearest_kf, new_kf_with_id, align, "loop_closure");
//                 addNewFactor(new_lc_factor);
//             }
//         }
//         counter++;
//         trf_prior.setIdentity();
//     }
//     else
//     {
//         trf_prior = align.transform;
//     }
// }

// geometry_msgs::Pose2D ScanMatcher::getPos(ros::Time time)
// {
//     geometry_msgs::TransformStamped transform = buffer.lookupTransform("odom", "base_link", time, ros::Duration(0.1));
//     geometry_msgs::Pose2D pose;
//     pose.x = transform.transform.translation.x;
//     pose.y = transform.transform.translation.y;
//     pose.theta = -acos(transform.transform.rotation.w) * 2;
//     return pose;
// }

// graph_slam::Factor ScanMatcher::composeTFFactor(graph_slam::KeyFrame last_kf, graph_slam::KeyFrame new_kf)
// {
//     graph_slam::Factor factor;
//     return factor;
// }

// Eigen::Matrix4f ScanMatcher::estimateTrf(graph_slam::KeyFrame &nearest_kf, graph_slam::KeyFrame &new_kf)
// {
//     return conversion_util::invertTransform(conversion_util::makeTransform(nearest_kf.pose)) * conversion_util::makeTransform(new_kf.pose);
// }

// graph_slam::KeyFrame ScanMatcher::composeInitialKeyFrame(const sensor_msgs::LaserScan &scan)
// {
//     graph_slam::KeyFrame kf;
//     kf.scan = scan;
//     kf.point_cloud = conversion_util::scanToROSPointcloud(scan);
//     kf.stamp = scan.header.stamp;
//     kf.pose = getPos(scan.header.stamp);
//     return kf;
// }

// graph_slam::KeyFrame ScanMatcher::composeKeyFrame(const sensor_msgs::LaserScan &scan, const sensor_msgs::PointCloud2 &point_cloud, Alignment &align, graph_slam::KeyFrame &last_kf)
// {
//     graph_slam::KeyFrame kf;
//     kf.stamp = scan.header.stamp;
//     kf.scan = scan;
//     kf.point_cloud = point_cloud;
//     Eigen::Matrix4f to_last_kf = conversion_util::makeTransform(last_kf.pose);
//     Eigen::Matrix4f to_new_kf = to_last_kf * align.transform;
//     kf.pose = conversion_util::makeDelta(to_new_kf);
//     return kf;
// }

// graph_slam::Factor ScanMatcher::composeFactor(graph_slam::KeyFrame last_kf, graph_slam::KeyFrame new_kf, Alignment align, std::string type)
// {
//     graph_slam::Factor factor;
//     factor.id1 = last_kf.id;
//     factor.id2 = new_kf.id;
//     factor.type = type;
//     factor.measurement = align.delta;
//     return factor;
// }

// Alignment ScanMatcher::gicpRegister(const sensor_msgs::PointCloud2 input_1, const sensor_msgs::PointCloud2 input_2, Eigen::Matrix4f &transform)
// {
//     // assign inputs
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_1 = conversion_util::rosPointcloudToPCL(input_1);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2 = conversion_util::rosPointcloudToPCL(input_2);
//     gicp.setInputSource(pointcloud_1);
//     gicp.setInputTarget(pointcloud_2);

//     // align
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
//     gicp.align(*pointcloud_transform, transform);

//     Alignment output;
//     output.convergence_state = gicp.getConvergeCriteria()->getConvergenceState();
//     output.converged = gicp.hasConverged();
//     output.fitness = gicp.getFitnessScore();

//     if (gicp.hasConverged())
//     {
//         transform = gicp.getFinalTransformation();

//         output.transform = transform;
//         geometry_msgs::Pose2D transform_delta = conversion_util::makeDelta(transform);
//         output.delta = transform_delta;
//     }
//     return output;
// }

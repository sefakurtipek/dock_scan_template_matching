#include "LaserScanProcessor.h"
#include "functions.h"

LaserScanProcessor::LaserScanProcessor() :  nh_(), listener_(){ 
    nh_.getParam("initialGuess_yaw_degrees", initialGuess_yaw_degrees);
    nh_.getParam("modelSampleNum", modelSampleNum);
    nh_.getParam("max_distance_between_corresponding_points", max_distance_between_corresponding_points);
    nh_.getParam("scoreThreshold", scoreThreshold);
    nh_.getParam("vShapeModelName", vShapeModelName);
    nh_.getParam("laserFrameName", laserFrameName);
    nh_.getParam("laserScanName", laserScanName);
    nh_.getParam("mapFrameName", mapFrameName);
    nh_.getParam("absolute_path", absolute_path);

    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(laserScanName, 1000, &LaserScanProcessor::scanCallback, this);
    preprocessed_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/preprocessed_points", 1);
    vShapedModel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/vShapedModelCloud", 1);
    finalTransformedModel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/finalTransformedModel", 1);
    smallStepsTransformedModel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/smallStepsTransformedModel", 1);
   
}

Eigen::Matrix4f LaserScanProcessor::getTransform(const std::string& target_frame, const std::string& source_frame) {
    tf::StampedTransform transform;
    try {
        listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0));
        listener_.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        throw;
    }
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transform, eigen_transform);
    return eigen_transform;
}

void LaserScanProcessor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanCloud(new pcl::PointCloud<pcl::PointXYZ>());
    if(!convertScanToPointCloud(laserScan, laserScanCloud, projector_)) { ROS_ERROR("Failed to convert laser scan to point cloud");return;}
    laserScanCloud = preprocessPointCloud(laserScanCloud);

    sensor_msgs::PointCloud2 preprocessed_output;
    pcl::toROSMsg(*laserScanCloud, preprocessed_output);
    preprocessed_output.header.frame_id = laserFrameName;
    preprocessed_output.header.stamp = ros::Time::now();
    preprocessed_points_pub_.publish(preprocessed_output);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::Ptr vShapeModel(new pcl::PointCloud<pcl::PointXYZ>);
    std::string fullPath_vShapedModel = absolute_path + vShapeModelName;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fullPath_vShapedModel, *vShapeModel) == -1 )
        {PCL_ERROR ("Couldn't read file \n");return;}

    float initial_guess_yaw_radian = (M_PI/180) * initialGuess_yaw_degrees;
    Eigen::Matrix4f transformMatrixTF = LaserScanProcessor::getTransform(laserFrameName, mapFrameName);
    Eigen::Matrix4f rotationYawRadianZ = Eigen::Matrix4f::Identity();
    rotationYawRadianZ.block<3,3>(0,0) = Eigen::AngleAxisf(initial_guess_yaw_radian, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    Eigen::Matrix3f rotationFromTF = transformMatrixTF.block<3,3>(0,0);
    Eigen::Matrix4f rotationTF_4x4 = Eigen::Matrix4f::Identity();
    rotationTF_4x4.block<3,3>(0,0) = rotationFromTF;

    Eigen::Matrix4f combinedMatrix = rotationTF_4x4 * rotationYawRadianZ;

    pcl::transformPointCloud(*vShapeModel, *vShapeModel, combinedMatrix); // initial pose is given

    int modelSampleNum = 10;
    const float max_distance_between_corresponding_points = 0.1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledVShapeModel = downsampleVShapeModel(vShapeModel, modelSampleNum); // TO DO : TUNE NUMBER
    
    sensor_msgs::PointCloud2 vShapeModel_pc2;
    pcl::toROSMsg(*downsampledVShapeModel, vShapeModel_pc2);
    vShapeModel_pc2.header.frame_id = laserFrameName;
    vShapeModel_pc2.header.stamp = ros::Time::now();
    vShapedModel_pub_.publish(vShapeModel_pc2);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(laserScanCloud);

    Eigen::Matrix4f bestTransformation;
    float bestScore = std::numeric_limits<float>::infinity(); // Initialize with worst score

    // Iterate through points in laserScanCloud to find a match
    int startPointsIdx = 0;
    for (int i = startPointsIdx; i< laserScanCloud->points.size();i++) {
        Eigen::Matrix4f transformation = calculateTransformationMatrix(laserScanCloud->points[i], downsampledVShapeModel);

        // Transform the downsampled model with the current hypothesis
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedModel(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*downsampledVShapeModel, *transformedModel, transformation);

        sensor_msgs::PointCloud2 finalTransformedModelMsg2;
        pcl::toROSMsg(*transformedModel, finalTransformedModelMsg2);
        finalTransformedModelMsg2.header.frame_id = laserFrameName;
        finalTransformedModelMsg2.header.stamp = ros::Time::now();;
        smallStepsTransformedModel_pub_.publish(finalTransformedModelMsg2);      

        // Check for correspondences in the laserScanCloud
        if (allPointsHaveCorrespondence(transformedModel, laserScanCloud, max_distance_between_corresponding_points, kdtree)) {
            // Calculate score for this transformation
            float score = calculateScore(transformedModel, laserScanCloud);

            // Update the best transformation if the current score is lower (better)
            if (score < bestScore) {
                bestScore = score;
                bestTransformation = transformation;
            }
        }
    }
    ROS_INFO("bestScore: %lf", bestScore );

    if(bestScore > scoreThreshold)
    {
        std::cout << "BAD SCORE"<< std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr finalTransformedModel(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*downsampledVShapeModel, *finalTransformedModel, bestTransformation);
    sensor_msgs::PointCloud2 finalTransformedModel_pc2;
    pcl::toROSMsg(*finalTransformedModel, finalTransformedModel_pc2);
    finalTransformedModel_pc2.header.frame_id = laserFrameName;
    finalTransformedModel_pc2.header.stamp = ros::Time::now();
    finalTransformedModel_pub_.publish(finalTransformedModel_pc2);
}
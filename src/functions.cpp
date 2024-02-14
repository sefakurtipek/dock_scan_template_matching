#include "functions.h"
#include "LaserScanProcessor.h"

bool convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             laser_geometry::LaserProjection& projector) {
    // First, make sure the cloud is empty
    cloud->clear();
    // Create a PointCloud2 pointer to store the converted data
    sensor_msgs::PointCloud2 cloud2;  
    // Use the passed-in projector instead of projector_
    projector.projectLaser(*scan, cloud2);
    // Convert to PCL data type
    pcl::fromROSMsg(cloud2, *cloud);
    // Check if the conversion was successful
    if (cloud->points.empty()) {
        return false;
    } else {
        return true;
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // PassThrough filter for cropping the point cloud
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.0, 2.0);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.0, 2.0);
    pass.filter(*cloud_filtered);
    return cloud_filtered;
}

Eigen::Matrix4f getInitialTransformationMatrix(tf::Vector3 translation) {
    float Yaw_degrees = translation.z();
    translation.setZ(0.0);
    // Convert Yaw from degrees to radians
    float Yaw_radians = Yaw_degrees * M_PI / 180.0f;
    // Create a rotation matrix (assuming Z-axis rotation)
    Eigen::Matrix3f rotationMatrix;
    rotationMatrix = Eigen::AngleAxisf(Yaw_radians, Eigen::Vector3f::UnitZ());
    // Convert tf::Vector3 to Eigen::Vector3f for translation
    Eigen::Vector3f translationEigen(translation.x(), translation.y(), translation.z());
   // Combine rotation and translation into an affine transformation matrix
    Eigen::Affine3f initialTransformation = Eigen::Translation3f(translationEigen) * rotationMatrix;
    // Convert to 4x4 matrix
    Eigen::Matrix4f initialTransformationMatrix = initialTransformation.matrix();
    return initialTransformationMatrix;
}

void findNearestNeighbors(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& modelSet,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& dataSet,
    pcl::PointCloud<pcl::PointXYZ>& correspondenceCloud) {

    std::vector<pcl::PointXYZ> nearestNeighbors;

    correspondenceCloud.clear();
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(dataSet); // I am searching for scan dataset to find best nearest two points from model set

    for (const auto& point : *modelSet) {
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);

        if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            nearestNeighbors.push_back(
                dataSet->points[pointIdxNKNSearch[0]]
            );
        }
    }

    for (const auto& nn : nearestNeighbors) {
        correspondenceCloud.points.push_back(nn);
    }
    correspondenceCloud.header = dataSet->header;
    correspondenceCloud.width = correspondenceCloud.points.size();
    correspondenceCloud.height = 1;
    correspondenceCloud.is_dense = true;
    
/*     ROS_INFO("modelSet size: %d", modelSet->size());
    ROS_INFO("dataSet size: %d", dataSet->size());
    ROS_INFO("correspondenceCloud size: %d", correspondenceCloud.size()); */
}

void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::Vector3& translation, double theta) {
    double theta_radians = theta * M_PI / 180.0;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // Set the rotation part of the transformation around the Z axis
    transform.rotate(Eigen::AngleAxisf(theta_radians, Eigen::Vector3f::UnitZ()));
    // Set the translation part of the transformation
    transform.translation() << translation.x(), translation.y(), 0.0f;
    // Transform the point cloud using the transformation matrix
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    cloud = transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleVShapeModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& vShapeModel, int num_points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    float step = static_cast<float>(vShapeModel->size() - 1) / (num_points - 1);
    for (int i = 0; i < num_points; ++i) {
        downsampled->push_back(vShapeModel->points[static_cast<int>(i * step)]);
    }
    return downsampled;
}

bool allPointsHaveCorrespondence(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformedModel,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud,
                                 const float maxDistance,
                                 pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree) {
    for (const auto& point : transformedModel->points) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree.radiusSearch(point, maxDistance, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0) {
            return false;
        }
    }
    return true;
}

// Function to calculate the score based on the sum of squared distances from
// each point in the transformedModel to its nearest neighbor in the laserScanCloud
float calculateScore(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformedModel,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(laserScanCloud);

    float score = 0.0f;
    for (const auto& point : transformedModel->points) {
        std::vector<int> nearestNeighborIndex(1);
        std::vector<float> nearestNeighborSquaredDistance(1);

        // If a nearest neighbor is found in the laserScanCloud
        if (kdtree.nearestKSearch(point, 1, nearestNeighborIndex, nearestNeighborSquaredDistance) > 0) {
            // Add the squared distance to the score
            score += nearestNeighborSquaredDistance[0];
        } else {
            // If no nearest neighbor is found, add a large penalty to the score
            // This depends on the scale of your point cloud
            score += std::numeric_limits<float>::max();
        }
    }

    return score;
}

Eigen::Matrix4f calculateTransformationMatrix(const pcl::PointXYZ& targetPoint, 
                                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledModel) {
    Eigen::Vector3f translation(targetPoint.x - downsampledModel->points[0].x, targetPoint.y - downsampledModel->points[0].y, 0.0f);
    Eigen::Matrix4f transformationMatrix = Eigen::Matrix4f::Identity();
    transformationMatrix(0,3) = translation.x();
    transformationMatrix(1,3) = translation.y();
    return transformationMatrix;
}

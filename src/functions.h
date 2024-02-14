#include "LaserScanProcessor.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>


#include <Eigen/Geometry>
#include <Eigen/Core>

#include <vector>
#include <queue>
#include <numeric>
#include <cmath>
#include <unordered_set>
#include <set>

bool convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             laser_geometry::LaserProjection& projector);
pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
Eigen::Matrix4f getInitialTransformationMatrix(tf::Vector3 translation);
void findNearestNeighbors(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& modelSet,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& dataSet,
    pcl::PointCloud<pcl::PointXYZ>& correspondenceCloud);
void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const tf::Vector3& translation, double theta);
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleVShapeModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr& vShapeModel, int num_points);
bool allPointsHaveCorrespondence(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformedModel,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud,
                                 const float maxDistance,
                                 pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree);
float calculateScore(const pcl::PointCloud<pcl::PointXYZ>::Ptr& transformedModel,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& laserScanCloud);
Eigen::Matrix4f calculateTransformationMatrix(const pcl::PointXYZ& targetPoint, 
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampledModel);
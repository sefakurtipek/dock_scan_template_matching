#ifndef LASER_SCAN_PROCESSOR_H
#define LASER_SCAN_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include "functions.h"

class LaserScanProcessor {
public:
    LaserScanProcessor();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScan);
    Eigen::Matrix4f getTransform(const std::string& target_frame, const std::string& source_frame);
 
private:
    float initialGuess_yaw_degrees; 
    int modelSampleNum;
    float max_distance_between_corresponding_points;
    float scoreThreshold;
    std::string vShapeModelName;
    std::string laserFrameName;
    std::string laserScanName;
    std::string mapFrameName;
    std::string absolute_path;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    ros::Publisher preprocessed_points_pub_;
    ros::Publisher vShapedModel_pub_;
    ros::Publisher finalTransformedModel_pub_;
    ros::Publisher smallStepsTransformedModel_pub_;

};

#endif // LASER_SCAN_PROCESSOR_H
# Robot Charge Station Detection using Template Matching

### Overview
The `dock_scan_template_matching` package is a specialized ROS package. It implements algorithm for robust robot charge station detection for accurate and precise docking purpose using 2D laser scan data. Firstly downsamples model point cloud to decrease computational complexity. After that, sampled model point cloud is transformed to every point from laser scan data on 2D. After transformation of model point cloud, score is calculated according to corresponding points between model and laser scan data by comparing their distance. Bestscore gives best transformation for model point cloud. It means that best tranformation gives accurate and precise location of robot charge station for docking purpose.
I am sharing my result on record of rviz visualizer;
https://youtu.be/Tps68zlMCuk

## Dependencies
- ROS (tested on Melodic)
- PCL (tested on 1.2)
- Eigen3
- C++11 or later

## Installation
```bash
git clone https://github.com/sefakurtipek/dock_scan_template_matching.git
catkin_make // to make build
```
## Usage

```bash
rosbag play scanMatchBag1.bag

```
The user needs to write the following commands on linux terminal to take parameters and run ROS package
```bash
    cd dock_scan_template_matching/launch
    roslaunch dock_scan_template_matching params.launch
```
In order to visualize I used rviz. I need to remap tf and tf_static according to name of the robot. For example, I took "/r300311695" name from 'rostopic list' command

```bash
rosrun rviz rviz /tf:=/r300311695/tf /tf_static=/r300311695/tf_static
```

## Parameters you need to know
The user may need to tune given parameters located in `params.launch` file.
`initialGuess_yaw_degrees`: initial angle guess on radian
`modelSampleNum`: Number of points taken from model point cloud. Higher number of model samples may increase computational complexity of the algorithm
`max_distance_between_corresponding_points`: Maximum distance threshold to match between model samples and laser scan data points on 2D
`scoreThreshold`: If bestScore is higher than this scoreThreshold, program prints `BAD SCORE`.
`vShapeModelName`: robot charge station model point cloud file. Type of file is `.pcd`
`laserFrameName`: Frame name of laser scan data
`mapFrameName`: Map frame name. It is used to get frame transformation.
`laserScanName`: Laser scan data topic name. It is used to subscribe lase I             
`absolute_path`: Absolute path of project files. It is used to give model point cloud file location.

```bash
    <param name="initialGuess_yaw_degrees" type="double" value="265.364" />
    <param name="modelSampleNum" type="int" value="10" />
    <param name="max_distance_between_corresponding_points" type="double" value=" 0.1" />
    <param name="scoreThreshold" type="double" value="0.01" />
    <param name="vShapeModelName" type="string" value="RMRS004.pcd" />
    <param name="laserFrameName" type="string" value="/r300311695/base_front_laser_link" /> <!-- Set robot laser frame according to your robot -->
    <param name="mapFrameName" type="string" value="v/mjdtwfb31e2e/map" />
    <param name="laserScanName" type="string" value="/r300311695/scan" />
    <param name="absolute_path" type="string" value="/home/sefa/catkin_ws/src/dock_scan_template_matching/src/" />
```

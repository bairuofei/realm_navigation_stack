#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string name_space = "robot_1";


double cameraOffsetZ = 0;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
double vehicleHeight = 0.75;
double terrainVoxelSize = 0.05;
double groundHeightThre = 0.1;
bool adjustZ = false;
double terrainRadiusZ = 0.5;
int minTerrainPointNumZ = 10;
double smoothRateZ = 0.2;
bool adjustIncl = false;
double terrainRadiusIncl = 1.5;
int minTerrainPointNumIncl = 500;
double smoothRateIncl = 0.2;
double InclFittingThre = 0.2;
double maxIncl = 30.0;

const int systemDelay = 5;
int systemInitCount = 0;
bool systemInited = false;

const int stackNum = 400;
int odomSendIDPointer = -1;

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());

std::vector<int> scanInd;

ros::Time odomTime;  // used to record the last odom time, synchronize with laserpoints

float vehicleX = 0;  // 全局变量，用来保存机器人当前的位置
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleRecX = 0;  // 机器人在发布上一个laser_points时的位姿， rec代表record
float vehicleRecY = 0;
float vehicleRecZ = 0;
float vehicleRecRoll = 0;
float vehicleRecPitch = 0;
float vehicleRecYaw = 0;

float terrainRecRoll = 0;
float terrainRecPitch = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

float terrainZ = 0;      // Current terrain information, updated in callback function
float terrainRoll = 0;
float terrainPitch = 0;


ros::Publisher *pubScanPointer = NULL;

nav_msgs::Odometry odomData; // publish to /state_estimation
ros::Publisher pubVehicleOdom;



// FIXED: 原代码假设雷达不旋转，只是随着机器人移动，因此这里的代码需要把pointCloud根据机器人的yaw角进行旋转
void scanHandler(const sensor_msgs::PointCloud2::ConstPtr &scanIn)
{
    if (!systemInited)
    {
        systemInitCount++;
        if (systemInitCount > systemDelay)
        {
            systemInited = true;
        }
        return;
    }

    if (odomSendIDPointer < 0)
    {
        return;
    }


    double odomRecTime = odomTime.toSec();  // read current odom's corresponding time
    vehicleRecX = vehicleX;  // Rec代表record
    vehicleRecY = vehicleY;
    vehicleRecZ = vehicleZ;
    vehicleRecRoll = vehicleRoll;
    vehicleRecPitch = vehiclePitch;
    vehicleRecYaw = vehicleYaw;

    terrainRecRoll = terrainRoll;
    terrainRecPitch = terrainPitch;


    float sinTerrainRecRoll = sin(terrainRecRoll);
    float cosTerrainRecRoll = cos(terrainRecRoll);
    float sinTerrainRecPitch = sin(terrainRecPitch);
    float cosTerrainRecPitch = cos(terrainRecPitch);
    float sinVehicleYaw = sin(vehicleRecYaw);
    float cosVehicleYaw = cos(vehicleRecYaw);

    scanData->clear();
    pcl::fromROSMsg(*scanIn, *scanData);
    pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

    int scanDataSize = scanData->points.size();
    for (int i = 0; i < scanDataSize; i++)
    {   
        float new_x = cosVehicleYaw * scanData->points[i].x - sinVehicleYaw * scanData->points[i].y;
        float new_y = sinVehicleYaw * scanData->points[i].x + cosVehicleYaw * scanData->points[i].y;
        float new_z = scanData->points[i].z;

        float pointX1 = new_x;
        float pointY1 = new_y * cosTerrainRecRoll - new_z * sinTerrainRecRoll;
        float pointZ1 = new_y * sinTerrainRecRoll + new_z * cosTerrainRecRoll;

        float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
        float pointY2 = pointY1;
        float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

        float pointX3 = pointX2 + vehicleRecX;
        float pointY3 = pointY2 + vehicleRecY;
        float pointZ3 = pointZ2 + vehicleRecZ;

        scanData->points[i].x = pointX3;
        scanData->points[i].y = pointY3;
        scanData->points[i].z = pointZ3;
    }

    // publish 5Hz registered scan messages
    sensor_msgs::PointCloud2 scanData2;
    pcl::toROSMsg(*scanData, scanData2);
    scanData2.header.stamp = ros::Time().fromSec(odomRecTime);
    scanData2.header.frame_id = "map";
    // printf("\n %s \n", scanIn->header.frame_id.c_str());
    pubScanPointer->publish(scanData2);
}


void speedHandler(const geometry_msgs::TwistStamped::ConstPtr &speedIn)
{
    vehicleSpeed = speedIn->twist.linear.x;
    vehicleYawRate = speedIn->twist.angular.z;
}


void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // 找到 robot_1 在 model_states 中的位置
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == name_space)  // find robot_x model state
        {   
            odomTime = ros::Time::now();  // Global variable
            geometry_msgs::Pose robot_pose = msg->pose[i];
            vehicleX = robot_pose.position.x;
            vehicleY = robot_pose.position.y;
            vehicleZ = robot_pose.position.z;

            // 从四元数提取roll, pitch, yaw
            tf::Quaternion quat(robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w);
            tf::Matrix3x3 m(quat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            vehicleRoll = roll;
            vehiclePitch = pitch;
            vehicleYaw = yaw;
            break;
        }
    }

    // publish /state_estimation topic
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);
    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 5 * (vehicleRoll - vehicleRecRoll);  // 由于vehicleRecRoll的设置频率为5，因此这里乘5，表示一秒的移动速度
    odomData.twist.twist.angular.y = 5 * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = 5 * (vehicleYaw - vehicleRecYaw);
    odomData.twist.twist.linear.x = 5 * (vehicleX - vehicleRecX);
    odomData.twist.twist.linear.z = 5 * (vehicleZ - vehicleRecZ);
    pubVehicleOdom.publish(odomData);    // /state_estimation topic

    if (odomSendIDPointer < 0) {
        vehicleRecX = vehicleX;  // 初始化vehileRecX的值
        vehicleRecY = vehicleY;
        vehicleRecZ = vehicleZ;
        vehicleRecRoll = vehicleRoll;
        vehicleRecPitch = vehiclePitch;
        vehicleRecYaw = vehicleYaw;
    }
    odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pioneerSimulator");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("name_space", name_space);
    nhPrivate.getParam("cameraOffsetZ", cameraOffsetZ);
    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
    nhPrivate.getParam("vehicleHeight", vehicleHeight);
    nhPrivate.getParam("vehicleX", vehicleX);
    nhPrivate.getParam("vehicleY", vehicleY);
    nhPrivate.getParam("vehicleZ", vehicleZ);
    nhPrivate.getParam("terrainZ", terrainZ);
    nhPrivate.getParam("vehicleYaw", vehicleYaw);
    nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
    nhPrivate.getParam("groundHeightThre", groundHeightThre);
    nhPrivate.getParam("adjustZ", adjustZ);
    nhPrivate.getParam("terrainRadiusZ", terrainRadiusZ);
    nhPrivate.getParam("minTerrainPointNumZ", minTerrainPointNumZ);
    nhPrivate.getParam("adjustIncl", adjustIncl);
    nhPrivate.getParam("terrainRadiusIncl", terrainRadiusIncl);
    nhPrivate.getParam("minTerrainPointNumIncl", minTerrainPointNumIncl);
    nhPrivate.getParam("InclFittingThre", InclFittingThre);
    nhPrivate.getParam("maxIncl", maxIncl);



    ros::Subscriber subSpeed = nh.subscribe<geometry_msgs::TwistStamped>("cmd_vel_sim", 5, speedHandler);
    ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud2>("filtered_velodyne_points", 2, scanHandler);
    ros::Subscriber subModelState = nh.subscribe("/gazebo/model_states", 10, modelStatesCallback);


    // Publish /state_estimation
    odomData.header.frame_id = "map";
    odomData.child_frame_id = name_space+"/base_link";
    pubVehicleOdom = nh.advertise<nav_msgs::Odometry>("state_estimation", 5);  // TODO: better way is to use pointer

    // Publish registered_scan
    ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2>("registered_scan", 2);
    pubScanPointer = &pubScan;

    printf("\nSimulation started.\n\n");
    ros::spin();

    return 0;
}

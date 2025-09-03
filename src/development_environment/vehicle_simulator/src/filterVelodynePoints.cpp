#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// 声明发布器
ros::Publisher pubFilterScan;

tf::TransformListener* listener;

std::string name_space;
std::vector<std::string> robot_names;
int robot_number;

// 回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // 将 PointCloud2 转换为 PCL 点云
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*input, cloud);

    // Get the robot's own transform
    std::vector<tf::StampedTransform> transforms;
    std::vector<std::pair<float, float>> otherPositions;
    tf::Vector3 relativeTranslation;
    for (auto& otherName: robot_names) {
        if (otherName.substr(0, 7) == name_space) {
            continue;
        }
        tf::StampedTransform transform;
        try {
            listener->lookupTransform(name_space+"/velodyne", otherName + "velodyne", ros::Time(0), transform);
            transforms.push_back(transform);
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            continue;
        }
        relativeTranslation = transform.getOrigin();
        otherPositions.push_back({relativeTranslation.getX(), relativeTranslation.getY()});
    }

    // TODO: Remove points on other robots

    // 过滤掉靠近任何机器人的点
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered;
    double distance_threshold = 0.7; // 定义阈值距离

    for (const auto& point : cloud.points)
    {
        bool keep_point = true;
        for (const auto& transform : transforms)
        {
            double dx = point.x - transform.getOrigin().x();
            double dy = point.y - transform.getOrigin().y();
            double dz = point.z - transform.getOrigin().z();
            double distance = sqrt(dx * dx + dy * dy + dz * dz);

            if (distance < distance_threshold)
            {
                keep_point = false;
                break;
            }
        }
        if (keep_point)
        {
            cloud_filtered.push_back(point);
        }
    }

    // 将过滤后的 PCL 点云转换回 PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_filtered, output);
    output.header = input->header;

    // 发布过滤后的点云
    pubFilterScan.publish(output);
}

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "filter_pointcloud2");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Create a transform listener
    listener = new tf::TransformListener;

    name_space = nh_private.param("name_space", std::string("robot_1"));

    robot_number = nh.param("/robot_number", 2);  // Get global parameter
    printf("\033[91m Get robot_number is %d\033[0m", robot_number);
    for (int i = 0; i < robot_number; i++) {
        robot_names.push_back("robot_" + std::to_string(i+1) + "/");
    }


    // 订阅 velodyne_points 主题
    ros::Subscriber subRawScan = nh.subscribe("velodyne_points", 1, pointCloudCallback);

    // 发布 filtered_velodyne_points 主题
    pubFilterScan = nh.advertise<sensor_msgs::PointCloud2>("filtered_velodyne_points", 1);

    // 循环等待回调
    ros::spin();

    return 0;
}

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

using namespace std;

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
        
        string laserscan_topic;
        string pointcloud_topic;
        string frame_id;

};

My_Filter::My_Filter(){
    
    ros::NodeHandle nh("~");

    nh.param<std::string>("laserscan_topic", laserscan_topic, "/scan");
    nh.param<std::string>("pointcloud_topic", pointcloud_topic, "/cloud");
    nh.param<std::string>("frame_id", frame_id, "/frame");

        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> (laserscan_topic.c_str(), 100, &My_Filter::scanCallback, this);//设置输入雷达的laserscan topic名
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (pointcloud_topic.c_str(), 100, false);//设置输出点云topic名
        tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}
void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud(frame_id.c_str(), *scan, cloud, tfListener_);//设置输出点云的坐标系
    point_cloud_publisher_.publish(cloud);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");

    My_Filter filter;

    ros::spin();

    return 0;
}

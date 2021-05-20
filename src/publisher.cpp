#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/point_cloud.h"
#include "std_msgs/msg/string.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

#define POINTS_PATH(camera_name)                                                                                       \
	(static_cast<std::ostringstream&&>(std::ostringstream() << "/" << camera_name << "/depth/color/points")).str()

class MultiCloudPublisher : public rclcpp::Node
{
public:
	MultiCloudPublisher() : Node("multi_cloud")
	{
		_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("multi_cloud", 1);
		this->declare_parameter<std::vector<std::string>>("cameras", {});
		this->get_parameter("cameras", _camera_names);

		_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		_tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer, true);

		for (auto camera_name : this->_camera_names)
		{
			RCLCPP_INFO(this->get_logger(), "Adding subscription for camera at path: %s", POINTS_PATH(camera_name).c_str());
			_camera_subscriptions.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
					POINTS_PATH(camera_name), rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
					std::bind(&MultiCloudPublisher::point_cloud_callback, this, std::placeholders::_1)));
		}
	}

private:
	void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
		// TODO: Don't hard-code the source frame
		geometry_msgs::msg::TransformStamped transform_stamped_msg;
		std::string frame_id = msg->header.frame_id;
		try
		{
			transform_stamped_msg = _tf_buffer->lookupTransform("world", frame_id, rclcpp::Time(0));
		}
		catch (tf2::TransformException& ex)
		{
			RCLCPP_WARN(this->get_logger(), "%s", ex.what());
			return;
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::moveFromROSMsg(*msg, *pcl_cloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::transformPointCloud(*pcl_cloud, *pcl_transformed_cloud, tf2::transformToEigen(transform_stamped_msg).matrix());
		_point_clouds[frame_id] = pcl_transformed_cloud;

		publish_points();
	}
	void publish_points()
	{
		if (_point_clouds.empty())
		{
			return;
		}

		pcl::PointCloud<pcl::PointXYZRGB> combined_pcl_cloud;
		for (const auto& points_pair : _point_clouds)
		{
			combined_pcl_cloud += *points_pair.second;
		}

		sensor_msgs::msg::PointCloud2 combined_cloud;
		pcl::toROSMsg(combined_pcl_cloud, combined_cloud);

		combined_cloud.header.frame_id = "world";
		combined_cloud.header.stamp = now();

		_publisher->publish(combined_cloud);
	}
	std::vector<std::string> _camera_names;
	std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> _camera_subscriptions;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;
	std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
	std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _point_clouds;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MultiCloudPublisher>());
	rclcpp::shutdown();
	return 0;
}
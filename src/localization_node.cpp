#include <blaze/Math.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <CRSLibtmp/mutexed_latest.hpp>
#include <CRSLibtmp/LightweightPointCloud/LShapeFit.hpp>

namespace localization_node
{
	using CRSLib::LightweightPointCloud::LShapeFit::calc_l_shape;
	using CRSLib::LightweightPointCloud::LShapeFit::Pose2D;
	using CRSLib::LightweightPointCloud::LShapeFit::Vec2D;
	using LatestPose2D = CRSLib::MutexedLatest<Pose2D, rclcpp::Time>;

	class LocalizationNode final : public rclcpp::Node
	{
		rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub;
		rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr odom_sub;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

		LatestPose2D urg_pose{std::forward_as_tuple(), std::forward_as_tuple(rclcpp::Time{0, 0})};
		LatestPose2D urg_velocity{std::forward_as_tuple(Pose2D::zero()), std::forward_as_tuple(rclcpp::Time{0, 0})};

	public:
		LocalizationNode(const rclcpp::NodeOptions& options):
			rclcpp::Node("l_shape_fit_localization", options)
		{
			{
				double x = 0, y = 0, theta = 0;
				if
				(
					this->get_parameter<double>("initial_global_pose.x", x) &&
					this->get_parameter<double>("initial_global_pose.y", y) &&
					this->get_parameter<double>("initial_global_pose.theta", theta)
				)
				{
					urg_pose.update(LatestPose2D::Stamped{Pose2D{Vec2D{x, y}, theta}, rclcpp::Time{0, 0}});
				}
				else
				{
					RCLCPP_ERROR(this->get_logger(), "Cannot read initial_global_pose.");
				}
			}

			pose_pub = this->create_publisher<geometry_msgs::msg::Pose2D>("pose", 1);
			odom_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("odom", 1, std::bind(&LocalizationNode::odom_callback, this, std::placeholders::_1));
			scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, std::bind(&LocalizationNode::scan_callback, this, std::placeholders::_1));
		}

	private:
		void odom_callback(const geometry_msgs::msg::TwistStamped::SharedPtr from_odom_urg_velocity)
		{
			urg_velocity.update(LatestPose2D::Stamped{Pose2D{{from_odom_urg_velocity->twist.linear.x, from_odom_urg_velocity->twist.linear.y}, from_odom_urg_velocity->twist.angular.z}, from_odom_urg_velocity->header.stamp});
		}

		// 同時に同じ関数を呼ばせてはならない。他の関数とは並列実行可能。
		void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_data)
		{
			const Pose2D urg_velocity = this->urg_velocity.get().value;
			const Pose2D urg_pose = this->urg_pose.get().value;

			std::vector<Vec2D> data_points{};
			data_points.reserve(scan_data->ranges.size());

			{
				double angle = scan_data->angle_min;
				double time = 0;
				for(const auto& range : scan_data->ranges)
				{
					if(scan_data->range_min <= range && range <= scan_data->range_max)
					{
						const auto urg_at_this_time = urg_pose.point + urg_velocity.point * time;
						const auto from_urg = range * Vec2D{blaze::cos(angle), blaze::sin(angle)};
						data_points.emplace_back(urg_at_this_time + from_urg);
					}

					angle += scan_data->angle_increment;
					time += scan_data->time_increment;
				}
			}

			if(const auto new_urg_pose = calc_l_shape(data_points, urg_pose, 0.1); new_urg_pose)
			{
				this->urg_pose.update(LatestPose2D::Stamped{*new_urg_pose, scan_data->header.stamp});

				geometry_msgs::msg::Pose2D message{};
				message.x = new_urg_pose->point[0];
				message.y = new_urg_pose->point[1];
				message.theta = new_urg_pose->theta;
				
				pose_pub->publish(message);
			}
			else
			{
				this->urg_pose.update(LatestPose2D::Stamped{urg_pose + urg_velocity * scan_data->scan_time, scan_data->header.stamp});
			}
		}
	};
}

RCLCPP_COMPONENTS_REGISTER_NODE(localization_node::LocalizationNode)
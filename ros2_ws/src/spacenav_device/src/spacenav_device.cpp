// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "spacenav_device.hpp"

#include <cmath> // std::isfinite

#include <chrono>
#include <functional> //std::bind
#include <memory>
#include <iostream>
#include <stdexcept>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/exceptions.hpp>

constexpr auto DEFAULT_NODE_NAME = "/cartesian_control_server_ros2";
constexpr auto DEFAULT_AXIS_SCALE = 0.3;
constexpr auto DEFAULT_STREAMING_MSG = "twist";

using namespace std::placeholders;

	SpacenavSubscriber::SpacenavSubscriber() : Node("spacenav_device")
	{
		last_update_time_ = this->now(); // Initialize last update time

		// Declare node Parameters
		rcl_interfaces::msg::ParameterDescriptor descriptor_msg;
		descriptor_msg.name = "streaming_msg";
		descriptor_msg.description = "Streaming command msg type to be used by the device.";
		descriptor_msg.read_only = false;
		descriptor_msg.additional_constraints = "Only 'twist' or 'pose' are allowed.";
		descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
		
		this->declare_parameter<std::string>("streaming_msg", DEFAULT_STREAMING_MSG, descriptor_msg); 
		this->get_parameter("streaming_msg", streaming_msg_);

		rcl_interfaces::msg::ParameterDescriptor descriptor_scale;
		descriptor_scale.name = "scale";
		descriptor_scale.description = "Scale factor to be applied to the spacenav device.";
		descriptor_scale.read_only = false;
		descriptor_scale.additional_constraints = "Only positive double values are allowed.";
		descriptor_scale.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE);

		this->declare_parameter<double>("scale", DEFAULT_AXIS_SCALE, descriptor_scale); 
		this->get_parameter("scale", scale_);

		// Callback for parameter changes
		callback_handle_ = this->add_on_set_parameters_callback(std::bind(&SpacenavSubscriber::parameter_callback, this, _1));
		
		// Set parameters for external node
		client_param_ = this->create_client<SetParameters>(DEFAULT_NODE_NAME + std::string("/set_parameters"));

		while (!client_param_->wait_for_service(std::chrono::seconds(1))) 
		{
			if (!rclcpp::ok()) 
			{
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				throw std::runtime_error("Interrupted while waiting for the service. Exiting."); 
				return;
			}

			RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
		}

		
		// Subscribers

		subscription_spnav_ = this->create_subscription<sensor_msgs::msg::Joy>("/spacenav/joy", 10, 
											std::bind(&SpacenavSubscriber::spnav_callback, 
											this, _1));
																					
		subscription_state_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(DEFAULT_NODE_NAME + std::string("/state/pose"), 10,
													std::bind(&SpacenavSubscriber::state_callback,
													this, _1));
		
		
		// Timer
		timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&SpacenavSubscriber::timer_callback, this));
		
		// Publisher
		publisher_spnav_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(DEFAULT_NODE_NAME + std::string("/command/twist"), 10);
		publisher_spnav_pose_ = this->create_publisher<geometry_msgs::msg::Pose>(DEFAULT_NODE_NAME + std::string("/command/pose"), 10);
		publisher_spnav_wrench_ = this->create_publisher<geometry_msgs::msg::Wrench>(DEFAULT_NODE_NAME + std::string("/command/wrench"), 10);
		publisher_spnav_gripper_ = this->create_publisher<std_msgs::msg::Int32>(DEFAULT_NODE_NAME + std::string("/command/gripper"), 10);


		// Parameters validation with exceptions to avoid runtime errors
		if (streaming_msg_ == "twist" && !set_preset_streaming_cmd("twist"))
		{
			throw rclcpp::exceptions::InvalidEventError::runtime_error("Failed to set preset streaming command.");
		}

		else if (streaming_msg_ == "pose" && !set_preset_streaming_cmd("pose"))
		{
			throw rclcpp::exceptions::InvalidEventError::runtime_error("Failed to set preset streaming command.");
		}
		
		else if (streaming_msg_ == "wrench" && !set_preset_streaming_cmd("wrench"))
		{
			throw rclcpp::exceptions::InvalidEventError::runtime_error("Failed to set preset streaming command.");
		}

		else if (streaming_msg_ != "twist" && streaming_msg_ != "pose" && streaming_msg_ != "wrench")
		{
			throw rclcpp::exceptions::InvalidParameterValueException("Invalid parameter type for streaming_msg. Only 'twist', 'pose' or 'wrench' are allowed.");			
		}

		if (scale_ <= 0)
		{
			throw rclcpp::exceptions::InvalidParameterValueException("Invalid parameter for scale. Only positive values are allowed.");
		}
}

// ------------------------------------------------------------------------------------

SpacenavSubscriber::~SpacenavSubscriber() 
{}

// -----------------------Callback from SpaceNavigator----------------------------------

	void SpacenavSubscriber::spnav_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		std::vector<float> a = msg->axes;
		std::vector<int> b = msg->buttons;

		if(b[0] == 1)
		{
			if(gripper_state_ != GRIPPER_CLOSE)
			{
				auto msg_gripper = std::make_shared<std_msgs::msg::Int32>();
				msg_gripper->data = GRIPPER_CLOSE;
				publisher_spnav_gripper_->publish(*msg_gripper);
			}

			gripper_state_ = GRIPPER_CLOSE;
		}
		else if (b[1] == 1)
		{
			if(gripper_state_ != GRIPPER_OPEN)
			{
				auto msg_gripper = std::make_shared<std_msgs::msg::Int32>();
				msg_gripper->data = GRIPPER_OPEN;
				publisher_spnav_gripper_->publish(*msg_gripper);
			}

			gripper_state_ = GRIPPER_OPEN;
		}
		else if(gripper_state_ != GRIPPER_NONE)
		{
			if(gripper_state_ != GRIPPER_STOP)
			{
				gripper_state_ = GRIPPER_STOP;
				auto msg_gripper = std::make_shared<std_msgs::msg::Int32>();
				msg_gripper->data = GRIPPER_STOP;
				publisher_spnav_gripper_->publish(*msg_gripper);
			}
			else
			{
				gripper_state_ = GRIPPER_NONE;
			}
		}
		else
		{
			gripper_state_ = GRIPPER_NONE;
		}

		for(int i = 0; i < 6; i++)
		{
			a[i] *= scale_; // Improve sensibility 
		}
		
		if (streaming_msg_ == "twist")
		{
			auto msg_twist = std::make_shared<geometry_msgs::msg::Twist>(); 
			if(msg_twist )
			{
				msg_twist->linear.x = a[0];
				msg_twist->linear.y = a[1];
				msg_twist->linear.z = a[2];
				msg_twist->angular.y = a[4];
				msg_twist->angular.z = a[5];

				// Publish message through timer_callback
				std::lock_guard<std::mutex> lock(msg_mutex_);
				last_msg_ = msg_twist;
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Failed to create Twist message");  
    		}
		} 

		else if (streaming_msg_ == "pose") 
		{
			auto current_time = now();
			auto dt = (current_time - last_update_time_).seconds(); // Get elapsed time since last update from sensor input
			last_update_time_ = current_time;

			if(!initial_pose_set_)
			{
				RCLCPP_WARN(this->get_logger(), "Initial pose not set. Cannot publish Pose message yet.");
				return;
			}

			// Set initial position as a virtual point to avoid PID compensation
			else if(initial_pose_set_ && !virtual_pose_set)
			{
				current_orientation_ = initial_orientation_;
				current_position_ = initial_position_;

				virtual_pose_set = true;
			}
			
			// Transform linear and angular velocities into space translations (de = v*dt)			
			std::vector<double> msg_traslation;

			for (int j=0; j<6; j++)
			{
				msg_traslation.push_back(a[j] * dt * scale_); 
			}

			tf2::Vector3 linear_translation(msg_traslation[0], msg_traslation[1], msg_traslation[2]);
			tf2::Vector3 angular_rotation(msg_traslation[3], msg_traslation[4], msg_traslation[5]);

			// New position applying linear translation
			tf2::Vector3 new_position = current_position_ + linear_translation;

			// New orientation applying axis-angle rotation
			tf2::Quaternion q;
			tf2::Vector3 axis = angular_rotation.normalized();
			double angle = angular_rotation.length(); 
			q.setRotation(axis, angle);
			
			tf2::Quaternion new_orientation = current_orientation_ * q;
			new_orientation.normalize();
			
			// Avoid NaN values
			if (std::isfinite(new_position.x()) && std::isfinite(new_position.y()) && std::isfinite(new_position.z()) &&
				std::isfinite(new_orientation.x()) && std::isfinite(new_orientation.y()) && std::isfinite(new_orientation.z()) && std::isfinite(new_orientation.w()))
			{	
				// Only publish if position has changed
				if(new_position!=current_position_ || new_orientation!=current_orientation_) 
				{
					current_position_ = new_position;
					current_orientation_ = new_orientation;

					auto msg_pose = std::make_shared<geometry_msgs::msg::Pose>();
					msg_pose->position.x = new_position.x();
					msg_pose->position.y = new_position.y();
					msg_pose->position.z = new_position.z();
					msg_pose->orientation = tf2::toMsg(new_orientation);;

					std::lock_guard<std::mutex> lock(msg_mutex_); // Lock mutex so it is not accessed by timer_callback 
					last_msg_pose_ = msg_pose;
					position_changed_ = true;
				}	
			}
		}

		else if(streaming_msg_ == "wrench")
		{
			auto msg_wrench = std::make_shared<geometry_msgs::msg::Wrench>();

			msg_wrench->force.x = a[0];
			msg_wrench->force.y = a[1];
			msg_wrench->force.z = a[2];
			msg_wrench->torque.x = a[3];
			msg_wrench->torque.y = a[4];
			msg_wrench->torque.z = a[5];	

			// Publish message
			std::lock_guard<std::mutex> lock(msg_mutex_);
			last_msg_wrench_ = msg_wrench;
		}
	}

// ---------------------- Get initial position and orientation from robot -----------------------

	void SpacenavSubscriber::state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
	{
		if(!initial_pose_set_ && streaming_msg_ == "pose")
		{
			tf2::fromMsg(msg->pose.position, initial_position_);
			tf2::fromMsg(msg->pose.orientation, initial_orientation_);
			if((initial_pose_set_ = true))
			{
				RCLCPP_INFO(this->get_logger(), "Initial pose correctly set.");
			}
		}	
	}	

// ------------------------- Set external node parameter -----------------------------

	bool SpacenavSubscriber::set_preset_streaming_cmd(const std::string &value)
	{
		// Creating request
		auto request = std::make_shared<SetParameters::Request>();

		// Creating parameter
		Parameter param;
		param.name = "preset_streaming_cmd";
		param.value.type = rclcpp::ParameterType::PARAMETER_STRING; 
		param.value.string_value = value;
		request->parameters.push_back(param);

		// Calling service
		using ServiceResponseFuture = rclcpp::Client<SetParameters>::SharedFuture;
		
		auto response_received_callback = [this](ServiceResponseFuture future) 
		{
			if (future.get()->results[0].successful)
			{
				RCLCPP_INFO(this->get_logger(), "Preset streaming command correctly stablished in external node.");
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Failed to set preset streaming command.");
			}
		};
	
		auto result = client_param_->async_send_request(request, response_received_callback); 

		return result.valid();
	}

// ----------------------Callback for parameter changes------------------------------

	rcl_interfaces::msg::SetParametersResult SpacenavSubscriber::parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
	{
		rcl_interfaces::msg::SetParametersResult result;
		result.successful = true;
		for (const auto &param: parameters)
		{
			if(param.get_name() == "streaming_msg")  
			{
				if(streaming_msg_ == param.value_to_string())
				{
					RCLCPP_WARN(this->get_logger(), "Param for streaming_msg is already set to %s!", streaming_msg_.c_str());
					continue;
				}
				else
				{
					if(param.value_to_string() == "twist")
					{
						streaming_msg_ = "twist";	
						if(!set_preset_streaming_cmd("twist"))
						{
							result.successful = false;
						}
						else
						{
							RCLCPP_INFO(this->get_logger(), "Param for streaming_msg correctly stablished: %s", streaming_msg_.c_str());
						}

					}
					else if (param.value_to_string() == "pose")
					{
						streaming_msg_ = "pose";
						if(!set_preset_streaming_cmd("pose"))
						{
							result.successful = false;
						}
						else
						{
							RCLCPP_INFO(this->get_logger(),"Param for streaming_msg correctly stablished: %s", streaming_msg_.c_str());
						
							// Reset initial pose
							initial_pose_set_ = false;
							virtual_pose_set = false;
						}
					}
					else if(param.value_to_string() == "wrench")
					{
						streaming_msg_ = "wrench";
						if(!set_preset_streaming_cmd("wrench"))
						{
							result.successful = false;
						}
						else
						{
							RCLCPP_INFO(this->get_logger(),"Param for streaming_msg correctly stablished: %s", streaming_msg_.c_str());
						}
					}
					else
					{
						result.successful = false;
						result.reason = "Invalid parameter type for streaming_msg. Only 'twist', 'pose' or 'wrench' are allowed.";
						RCLCPP_ERROR(this->get_logger(), result.reason.c_str());
					}
				}	
			}

			else if(param.get_name() == "scale")
			{
				if (param.as_double() > 0)
				{
					scale_ = param.as_double();
					RCLCPP_INFO(this->get_logger(), "Param for scale correctly stablished: %f", scale_);
				}
				else
				{
					result.successful = false;
					result.reason = "Invalid parameter for scale. Only positive values are allowed.";
					RCLCPP_ERROR(this->get_logger(), result.reason.c_str());
				}
			}
		}
		return result;
	}	

// ----------------------Timer callback for publishing msgs at 50 Hz------------------------------

	void SpacenavSubscriber::timer_callback()
	{

		if (streaming_msg_ == "twist")  
		{
			geometry_msgs::msg::Twist::SharedPtr msg_to_publish;
			{
				std::lock_guard<std::mutex> lock(msg_mutex_);
				if (last_msg_!=nullptr) // Check if there is a message to publish to avoid publishing null messages
				{
					msg_to_publish = last_msg_;
				}
				else
				{
					return;
				}	
			}
			
			bool zero_msg = (msg_to_publish->linear.x == 0.0 && msg_to_publish->linear.y == 0.0 && msg_to_publish->linear.z == 0.0 && 
							msg_to_publish->angular.x == 0.0 && msg_to_publish->angular.y == 0.0 && msg_to_publish->angular.z == 0.0);
								
			if (msg_to_publish) 
			{									
				publisher_spnav_twist_->publish(*msg_to_publish);

				if(!zero_msg)
				{
					RCLCPP_INFO(this->get_logger(), "Spnav Twist: [%f %f %f] [%f %f %f]", msg_to_publish->linear.x, msg_to_publish->linear.y, msg_to_publish->linear.z, 
													msg_to_publish->angular.x, msg_to_publish->angular.y, msg_to_publish->angular.z);
				}
			}
		}
		else if(streaming_msg_ == "pose")
		{
			geometry_msgs::msg::Pose::SharedPtr msg_to_publish;
			{
				std::lock_guard<std::mutex> lock(msg_mutex_);
				if(last_msg_pose_!=nullptr)
				{
					msg_to_publish = last_msg_pose_;
				}
				else
				{
					return;
				}	
			}
			if (msg_to_publish && position_changed_)
			{
                RCLCPP_INFO(this->get_logger(), "Spnav Pose: [%f %f %f] [%f %f %f %f]", msg_to_publish->position.x, msg_to_publish->position.y, msg_to_publish->position.z,
                            					 msg_to_publish->orientation.x, msg_to_publish->orientation.y, msg_to_publish->orientation.z, msg_to_publish->orientation.w);

                publisher_spnav_pose_->publish(*msg_to_publish);
				position_changed_ = false;	
			}
		}
		else if(streaming_msg_ == "wrench")
		{
			geometry_msgs::msg::Wrench::SharedPtr msg_to_publish;
			{
				std::lock_guard<std::mutex> lock(msg_mutex_);
				if(last_msg_wrench_!=nullptr)
				{
					msg_to_publish = last_msg_wrench_;
				}
				else
				{
					return;
				}	
			}
			if (msg_to_publish)
			{
				publisher_spnav_wrench_->publish(*msg_to_publish);
				RCLCPP_INFO(this->get_logger(), "Spnav Wrench: [%f %f %f] [%f %f %f]", msg_to_publish->force.x, msg_to_publish->force.y, msg_to_publish->force.z,
													msg_to_publish->torque.x, msg_to_publish->torque.y, msg_to_publish->torque.z);
			}
		}
	}	


// -----------------------------------------------------------------------------

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	try
	{
		rclcpp::spin(std::make_shared<SpacenavSubscriber>());
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "Something went wrong. Exiting." << std::endl;
	}
	
	rclcpp::spin(std::make_shared<SpacenavSubscriber>());
	rclcpp::shutdown();
	return 0;
}

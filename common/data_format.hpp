#ifndef DATA_HH
#define DATA_HH

#include <iostream>
#include <chrono>
#include <array>
#include <memory>
#include <boost/optional.hpp>

#include <opencv2/core/mat.hpp>
#undef Success // For 'Success' conflict
#include <eigen3/Eigen/Dense>
#include <GL/gl.h>
#include <GLFW/glfw3.h>
//#undef Complex // For 'Complex' conflict
#include "phonebook.hpp"
#include "switchboard.hpp"

// Tell gldemo and timewarp_gl to use two texture handle for left and right eye
#define USE_ALT_EYE_FORMAT
#define NANO_SEC 1000000000.0

namespace ILLIXR {

	typedef std::chrono::time_point<std::chrono::system_clock> time_type;
	typedef unsigned long long ullong;

	// Data type that combines the IMU and camera data at a certain timestamp.
	// If there is only IMU data for a certain timestamp, img0 and img1 will be null
	// time is the current UNIX time where dataset_time is the time read from the csv
	struct imu_cam_type : public switchboard::event {
		time_type time;
		Eigen::Vector3f angular_v;
		Eigen::Vector3f linear_a;
		std::unique_ptr<const cv::Mat> img0;
		std::unique_ptr<const cv::Mat> img1;
		ullong dataset_time;
		imu_cam_type() { }
		imu_cam_type(time_type time_, Eigen::Vector3f angular_v_, Eigen::Vector3f linear_a_, std::unique_ptr<const cv::Mat>&& img0_, std::unique_ptr<const cv::Mat>&& img1_, ullong dataset_time_)
			: time{time_}
			, angular_v{angular_v_}
			, linear_a{linear_a_}
			, img0{std::move(img0_)}
			, img1{std::move(img1_)}
			, dataset_time{dataset_time_}
		{ }
		imu_cam_type& operator=(const imu_cam_type& other) {
			time = other.time;
			angular_v = other.angular_v;
			img0.reset(new cv::Mat{*other.img0});
			img1.reset(new cv::Mat{*other.img1});
			dataset_time = other.dataset_time;
			return *this;
		}
	};

	struct pose_type : public switchboard::event {
		time_type time; 
		Eigen::Vector3f position;
		Eigen::Quaternionf orientation;
		pose_type(time_type time_, Eigen::Vector3f position_, Eigen::Quaternionf orientation_)
			: time{time_}
			, position{position_}
			, orientation{orientation_}
		{ }
		pose_type() { }
	};

	typedef struct {
		int pixel[1];
	} camera_frame;

	class global_config : public phonebook::service {
	public:
		global_config(GLFWwindow* _glfw_context) : glfw_context(_glfw_context) { }
		GLFWwindow* glfw_context;
	};

	// Single-texture format; arrayed by left/right eye
	// Single-texture format; arrayed by left/right eye
	struct rendered_frame : public switchboard::event {
		GLuint texture_handle;
		pose_type render_pose; // The pose used when rendering this frame.
		rendered_frame() {}
		rendered_frame(GLuint texture_handle_, pose_type render_pose_)
			: texture_handle{texture_handle_}
			, render_pose{render_pose_}
		{ }
	};

	// Using arrays as a swapchain
	// Array of left eyes, array of right eyes
	// This more closely matches the format used by Monado
	struct rendered_frame_alt : public switchboard::event {
		std::array<GLuint, 2> texture_handles; // Does not change between swaps in swapchain
		std::array<GLuint, 2> swap_indices; // Which element of the swapchain
		pose_type render_pose; // The pose used when rendering this frame.
		rendered_frame_alt() {}
		rendered_frame_alt(std::array<GLuint, 2> texture_handles_, std::array<GLuint, 2> swap_indices_, pose_type render_pose_)
			: texture_handles{texture_handles_}
			, swap_indices(swap_indices_)
			, render_pose{render_pose_}
		{ }
	};

	struct hologram_input : public switchboard::event {
		int seq;
		hologram_input() { }
		hologram_input(int seq_) : seq{seq_} { }
	};

	typedef struct {
		int dummy;
	} hologram_output;

	/* I use "accel" instead of "3-vector" as a datatype, because
	this checks that you meant to use an acceleration in a certain
	place. */
	struct accel { };

	// High-level HMD specification, timewarp plugin
	// may/will calculate additional HMD info based on these specifications
	struct hmd_physical_info {
		float   ipd;
		int		displayPixelsWide;
		int		displayPixelsHigh;
		float	chromaticAberration[4];
		float	K[11];
		int		visiblePixelsWide;
		int		visiblePixelsHigh;
		float	visibleMetersWide;
		float	visibleMetersHigh;
		float	lensSeparationInMeters;
		float	metersPerTanAngleAtCenter;
	};
}

#endif

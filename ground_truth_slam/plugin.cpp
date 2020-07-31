#include <chrono>
#include <iomanip>
#include <thread>
#include <functional>
#include "common/plugin.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "data_loading.hpp"

using namespace ILLIXR;

class ground_truth_slam : public plugin {
public:
	ground_truth_slam(std::string name_, phonebook* pb_)
		: plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_true_pose{sb->get_writer<pose_type>("true_pose")}
		, _m_sensor_data{load_data(std::string{std::getenv("ILLIXR_DATA")})}
	{ }

	virtual void start() override {
		sb->schedule<imu_cam_type>(get_name(), "imu_cam", std::bind(&ground_truth_slam::feed_ground_truth, this, std::placeholders::_1));
	}

	void feed_ground_truth(ptr<const imu_cam_type> datum) {
		// std::cerr << "ground_truth_slam::feed_ground_truth " << datum.get() << std::endl;
		ullong rounded_time = floor(datum->dataset_time / 10000);
		_m_sensor_data_it = _m_sensor_data.find(rounded_time);

		if (_m_sensor_data_it == _m_sensor_data.end()) {
			std::cout << "True pose not found at timestamp: " << rounded_time << std::endl;
			return;
		}

		pose_type* true_pose = new (_m_true_pose.allocate()) pose_type {datum->time, _m_sensor_data_it->second.position, _m_sensor_data_it->second.orientation};
		// std::cout << "The pose was found at " << true_pose->position[0] << ", " << true_pose->position[1] << ", " << true_pose->position[2] << std::endl; 

		_m_true_pose.put(true_pose);
	}

private:
	const std::shared_ptr<switchboard> sb;
	switchboard::writer<pose_type> _m_true_pose;
	const std::map<ullong, sensor_types> _m_sensor_data;
	std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
};

PLUGIN_MAIN(ground_truth_slam);

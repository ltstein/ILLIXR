#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "data_loading.hpp"
#include "common/data_format.hpp"
#include "common/threadloop.hpp"

using namespace ILLIXR;

const std::string data_path = "data1/";

class offline_imu_cam : public threadloop {
public:
	// Public constructor, Spindle passes the phonebook to this
	// constructor. In turn, the constructor fills in the private
	// references to the switchboard plugs, so the plugin can read
	// the data whenever it needs to.
	offline_imu_cam(phonebook* pb) // constructor
		: _m_sensor_data{load_data(data_path)} //ignore
		, _m_sb{pb->lookup_impl<switchboard>()} // in phonebook, find switchboard and assign to private reference _m_sb
		, _m_imu_cam{_m_sb->publish<imu_cam_type>("imu_cam")}
		, _m_sensor_data_it{_m_sensor_data.cbegin()} //ignore
	{
		dataset_first_time = _m_sensor_data_it->first; //ignore
		real_first_time = std::chrono::system_clock::now(); //ignore
	}

protected:
	// a continous loop like Update() in Unity
	virtual void _p_one_iteration() override { 
		if (_m_sensor_data_it != _m_sensor_data.end()) {

			//ignore these three lines
			ullong dataset_now = _m_sensor_data_it->first;
			reliable_sleep(std::chrono::nanoseconds{dataset_now - dataset_first_time} + real_first_time);
			time_type ts = real_first_time + std::chrono::nanoseconds{dataset_now - dataset_first_time};

			const sensor_types& sensor_datum = _m_sensor_data_it->second;
			if (sensor_datum.imu0) {
				_m_imu_cam->put(new imu_cam_type{
					ts,
					(sensor_datum.imu0.value().angular_v).cast<float>(),
					(sensor_datum.imu0.value().linear_a).cast<float>(),
					sensor_datum.cam0
						? std::make_optional<cv::Mat*>(sensor_datum.cam0.value().load().release())
						: std::nullopt,
					sensor_datum.cam1
						? std::make_optional<cv::Mat*>(sensor_datum.cam1.value().load().release())
						: std::nullopt,
					dataset_now,
				});
			}

			// last_time = dataset_now;
			++_m_sensor_data_it;
		}
	}

private:
	// For switchboard
	switchboard * const _m_sb; // reference to switchboard
	std::unique_ptr<writer<imu_cam_type>> _m_imu_cam; // pointer to the imu_cam

	// the camera and imu data
	const std::map<ullong, sensor_types> _m_sensor_data;
	std::map<ullong, sensor_types>::const_iterator _m_sensor_data_it;
	
	// Timestamps 
	ullong dataset_first_time; // the starting first time of the dataset
	time_type real_first_time; // the starting first time of the operating system
};

PLUGIN_MAIN(offline_imu_cam)

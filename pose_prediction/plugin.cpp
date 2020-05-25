#include "common/phonebook.hpp"
#include "common/pose_prediction.hpp"
#include "common/data_format.hpp"

using namespace ILLIXR;

class pose_prediction_impl : public pose_prediction {
public:
    pose_prediction_impl(phonebook* pb_)
    	: pb{pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, _m_pose{sb->get_reader<pose_type>("slow_pose")}
        , _m_true_pose{sb->get_reader<pose_type>("true_pose")}
    { }

	virtual void start() override {
		pb->register_impl<pose_prediction>(this);
	}

    // In the future this service will be pose predict which will predict a pose some t in the future
    virtual pose_type get_fast_pose() override {
        ptr<const pose_type> latest_pose = _m_pose.get_latest_ro();
        return correct_pose(latest_pose);
    }

    virtual pose_type get_fast_true_pose() override {
        ptr<const pose_type> true_pose = _m_true_pose.get_latest_ro();
        if (true_pose == nullptr) {
            return pose_type{};
        }
        return correct_pose(true_pose);
    }

private:
    phonebook* const pb;
    switchboard* const sb;
    switchboard::reader<pose_type> _m_pose;
    switchboard::reader<pose_type> _m_true_pose;
    
    pose_type correct_pose(ptr<const pose_type> pose) {
        pose_type swapped_pose {*pose};

        // This uses the OpenVINS standard output coordinate system.
        // This is a mapping between the OV coordinate system and the OpenGL system.
        swapped_pose.position.x() = -pose->position.y();
        swapped_pose.position.y() = pose->position.z();
        swapped_pose.position.z() = -pose->position.x();

		
        // There is a slight issue with the orientations: basically,
        // the output orientation acts as though the "top of the head" is the
        // forward direction, and the "eye direction" is the up direction.
        // Can be offset with an initial "calibration quaternion."
        swapped_pose.orientation.w() = pose->orientation.w();
        swapped_pose.orientation.x() = -pose->orientation.y();
        swapped_pose.orientation.y() = pose->orientation.z();
        swapped_pose.orientation.z() = -pose->orientation.x();

        return swapped_pose;
    }
};

PLUGIN_MAIN(pose_prediction_impl);

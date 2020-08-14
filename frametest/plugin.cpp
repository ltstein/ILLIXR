#include <chrono>
#include "common/phonebook.hpp"
#include "common/plugin.hpp"
#include "common/threadloop.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"

using namespace ILLIXR;

// Inherit from `plugin` if you don't need the threadloop
class frametest : public threadloop {
public:
    frametest(std::string name_, phonebook* pb_) :
        threadloop{name_, pb_},
        // find the switchboard in phonebook
        sb{pb->lookup_impl<switchboard>()}
    {
        #ifdef USE_ALT_EYE_FORMAT
        sb->schedule<rendered_frame_alt>(get_name(), "tw_frame", [&](const rendered_frame_alt *event3) {
            /*
            This is a [lambda expression][1]
            [1]: https://en.cppreference.com/w/cpp/language/lambda
            */
            std::cout << "Got a new event on tw_frame: " << event3 << std::endl;
        });
        #else
        sb->schedule<rendered_frame>(get_name(), "tw_frame", [&](const rendered_frame *event3) {
            /*
            This is a [lambda expression][1]
            [1]: https://en.cppreference.com/w/cpp/language/lambda
            */
            std::cout << "Got a new event on tw_frame: " << event3 << std::endl;
        });
        #endif
    }

    void _p_one_iteration() override {
        std::cout << "Running" << std::endl;
        auto target = std::chrono::high_resolution_clock::now() +  std::chrono::milliseconds{10};
        reliable_sleep(target);
    }

private:
    const std::shared_ptr<switchboard> sb;
};

// This line makes the plugin importable by Spindle
PLUGIN_MAIN(frametest);
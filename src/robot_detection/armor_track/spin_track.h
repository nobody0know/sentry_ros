#include "armor_detection.h"

namespace robot_detection {

    class SpinTracker
    {
    public:
        Armor last_armor;                       //本次装甲板
        bool is_initialized;                    //是否完成初始化
        chrono_time last_timestamp;             //本次装甲板时间戳

        explicit SpinTracker(const Armor& src, chrono_time src_timestamp);
        bool update_tracker(const Armor& new_armor, chrono_time timestamp);
    };

}
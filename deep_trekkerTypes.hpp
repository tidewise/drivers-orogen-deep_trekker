#ifndef deep_trekker_TYPES_HPP
#define deep_trekker_TYPES_HPP

#include "base/Time.hpp"

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace deep_trekker {
    struct GetRequestsIntervals {
        base::Time revolution_pose_z_attitude;
        base::Time powered_reel;
        base::Time camera_head;
    };
}

#endif


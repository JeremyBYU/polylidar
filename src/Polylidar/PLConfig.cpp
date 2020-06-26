#include "Polylidar/Utility.hpp"
#include "PLConfig.hpp"

namespace Polylidar {

std::string GetPolylidarVersion() { return std::string("Polylidar ") + POLYLIDAR_VERSION; }

bool RobustPredicatesActivated() { return PL_USE_ROBUST_PREDICATES; }

} // namespace Polylidar
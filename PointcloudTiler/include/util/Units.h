#pragma once

#include <boost/units/quantity.hpp>
#include <boost/units/systems/information.hpp>

namespace unit {
using byte = boost::units::quantity<boost::units::information::hu::byte::info>;
}

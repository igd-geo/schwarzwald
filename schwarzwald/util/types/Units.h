#pragma once

#include <boost/units/quantity.hpp>
#include <boost/units/systems/information.hpp>
#include <string>

namespace unit {
using byte = boost::units::quantity<boost::units::information::hu::byte::info>;

/**
 * Format a quantity into a human-readable string using metric prefixes (kilo,
 * mega etc.). Some examples:
 *
 *   format_with_metric_prefix(1234)       -> 1.234k
 *   format_with_metric_prefix(34'500'000) -> 3.45M
 *   format_with_metric_prefix(1010, 1)    -> 1k (precision of one decimal
 * place)
 */
std::string format_with_metric_prefix(double quantity, uint32_t precision = 3);
/**
 * Format a quantity into a human-readable string using binary prefixes (kibi,
 * mebi etc.). Some examples:
 *
 *   format_with_binary_prefix(1024)       -> 1Ki
 *   format_with_binary_prefix(3'600'000)  -> 3.433Mi
 *   format_with_binary_prefix(1048, 1)    -> 1Ki(precision of one decimal
 * place)
 */
std::string format_with_binary_prefix(double quantity, uint32_t precision = 3);

} // namespace unit

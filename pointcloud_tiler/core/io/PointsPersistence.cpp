#include "io/PointsPersistence.h"

#include <boost/format.hpp>

PointAttributes
supported_output_attributes_for_format(OutputFormat format)
{
  switch (format) {
    case OutputFormat::BIN:
      return BinaryPersistence::supported_output_attributes();
    case OutputFormat::CZM_3DTILES:
      return Cesium3DTilesPersistence::supported_output_attributes();
    case OutputFormat::ENTWINE_LAS:
    case OutputFormat::ENTWINE_LAZ:
    case OutputFormat::LAS:
    case OutputFormat::LAZ:
      return LASPersistence::supported_output_attributes();
    default:
      throw std::runtime_error{
        (boost::format("Invalid OutputFormat: %1%") % static_cast<int>(format)).str()
      };
  }
}
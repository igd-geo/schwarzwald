#include "io/PointsPersistence.h"

#include <boost/format.hpp>

PointsPersistence
make_persistence(OutputFormat format,
                 const fs::path& output_directory,
                 const PointAttributes& input_attributes,
                 const PointAttributes& output_attributes,
                 RGBMapping rgb_mapping,
                 float spacing,
                 const AABB& bounds)
{
  switch (format) {
    case OutputFormat::BIN:
      return PointsPersistence{ BinaryPersistence{
        output_directory, input_attributes, output_attributes, Compressed::No } };
    case OutputFormat::BINZ:
      return PointsPersistence{ BinaryPersistence{
        output_directory, input_attributes, output_attributes, Compressed::Yes } };
    case OutputFormat::CZM_3DTILES:
      return PointsPersistence{ Cesium3DTilesPersistence{ output_directory,
                                                          input_attributes,
                                                          output_attributes,
                                                          rgb_mapping,
                                                          spacing,
                                                          bounds.getCenter() } };
    case OutputFormat::LAS:
      return PointsPersistence{ LASPersistence{
        output_directory, input_attributes, output_attributes } };
    case OutputFormat::LAZ:
      return PointsPersistence{ LASPersistence{
        output_directory, input_attributes, output_attributes, Compressed::Yes } };
    case OutputFormat::ENTWINE_LAS:
      return PointsPersistence{ EntwinePersistence{
        output_directory.string(), input_attributes, output_attributes, EntwineFormat::LAS } };
    case OutputFormat::ENTWINE_LAZ:
      return PointsPersistence{ EntwinePersistence{
        output_directory.string(), input_attributes, output_attributes, EntwineFormat::LAZ } };
    default:
      throw std::invalid_argument{ "Unrecognized output format!" };
  }
}

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

#include "io/LASFile.h"
#include "types/Units.h"
#include "util/stuff.h"

#include <boost/program_options.hpp>
#include <chrono>
#include <limits>
#include <sstream>
#include <taskflow/taskflow.hpp>
#include <unordered_map>

namespace fs = std::experimental::filesystem;
namespace bpo = boost::program_options;

enum class TestType
{
  ReadSequential,
  ReadParallel,
  WriteSequential,
  WriteParallel
};

struct Args
{
  fs::path source_directory;
  fs::path output_directory;
  TestType test_type;
};

static Args
parse_args(int argc, char** argv)
{
  std::string source_directory;
  std::string output_directory;
  std::string test_type_string;

  bpo::positional_options_description positional_options;
  positional_options.add("test-type", 1);

  static const std::unordered_map<std::string, TestType> TestTypes = {
    { "read-sequential", TestType::ReadSequential },
    { "read-parallel", TestType::ReadParallel },
    { "write-sequential", TestType::WriteSequential },
    { "write-parallel", TestType::WriteParallel }
  };

  const auto test_type_help_string = [&]() {
    std::stringstream ss;
    ss << "One of ";
    std::for_each(std::begin(TestTypes), std::end(TestTypes), [&ss](const auto& kv) {
      ss << "\"" << kv.first << "\" ";
    });
    return ss.str();
  }();

  bpo::options_description options("Options");
  options.add_options()("help,h", "Produce help message")(
    "test-type",
    bpo::value<std::string>(&test_type_string)->required(),
    test_type_help_string.c_str())("source,i",
                                   bpo::value<std::string>(&source_directory)->required(),
                                   "Source directory containing one or more LAS/LAZ files")(
    "output,o",
    bpo::value<std::string>(&output_directory)->required(),
    "Output directory for write tests");

  bpo::variables_map variables;
  try {
    bpo::store(bpo::command_line_parser(argc, argv)
                 .options(options)
                 .positional(positional_options)
                 .allow_unregistered()
                 .run(),
               variables);

    if (variables.count("help") || variables.empty()) {
      std::cout << "Usage: " << argv[0] << " <test-type> [options]\n";
      options.print(std::cout);
      std::exit(EXIT_SUCCESS);
    }

    bpo::notify(variables);
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

  const auto test_type_iter = TestTypes.find(test_type_string);
  if (test_type_iter == std::end(TestTypes)) {
    std::cerr << "Invalid test-type argument \"" << test_type_string << "\"" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  return Args{ source_directory, output_directory, test_type_iter->second };
}

static std::vector<fs::path>
get_all_las_files_in_directory(const fs::path& directory)
{
  const auto all_files = get_all_files_in_directory(directory, Recursive::Yes);
  const auto is_las_file = [](const fs::path& file) {
    return file.extension() == ".las" || file.extension() == ".laz";
  };
  std::vector<fs::path> las_files;
  std::copy_if(
    std::begin(all_files), std::end(all_files), std::back_inserter(las_files), is_las_file);
  return las_files;
}

static std::string
format_time(std::chrono::nanoseconds ns)
{
  std::stringstream ss;
  ss << unit::format_with_metric_prefix(ns.count() / 1e9, 2) << "s";
  return ss.str();
}

static std::string
format_memory_size(size_t memory)
{
  std::stringstream ss;
  ss << unit::format_with_binary_prefix(memory, 2) << "B";
  return ss.str();
}

struct PerFileData
{
  size_t point_count = 0;
  Vector3<double> min_pos = { std::numeric_limits<double>::max() };
  Vector3<double> max_pos = { std::numeric_limits<double>::lowest() };
};

PerFileData
operator+(const PerFileData& l, const PerFileData& r)
{
  return { l.point_count + r.point_count,
           Vector3<double>::minByAxis(l.min_pos, r.min_pos),
           Vector3<double>::maxByAxis(l.max_pos, r.max_pos) };
}

static void
analyze_file(const fs::path& file_path, PerFileData& data)
{
  const LASFile las_file{ file_path, LASFile::OpenMode::Read };
  const auto& las_header = las_file.get_metadata();
  data.point_count = pc::get_point_count(las_file);

  for (auto point : las_file) {
    const auto position = position_from_las_point(point, las_header);
    data.min_pos = Vector3<double>::minByAxis(data.min_pos, position);
    data.max_pos = Vector3<double>::maxByAxis(data.max_pos, position);
  }
}

static PerFileData
analyze_files(const std::vector<fs::path>& files, size_t num_threads)
{
  std::vector<PerFileData> per_file_data;
  per_file_data.resize(files.size());

  tf::Executor executor{ static_cast<uint32_t>(num_threads) };
  tf::Taskflow taskflow;
  for (size_t idx = 0; idx < files.size(); ++idx) {
    auto& file = files[idx];
    auto& data = per_file_data[idx];
    taskflow.emplace([&data, &file]() { analyze_file(file, data); });
  }
  executor.run(taskflow).wait();

  return std::accumulate(std::begin(per_file_data), std::end(per_file_data), PerFileData{});
}

static void
write_random_file(const fs::path& path, size_t point_count)
{
  LASFile las_file{ path, LASFile::OpenMode::Write };
  auto metadata = las_file.get_metadata();
  metadata.number_of_point_records = metadata.extended_number_of_point_records = point_count;
  metadata.number_of_points_by_return[0] = point_count;
  metadata.min_x = metadata.min_y = metadata.min_z = 0;
  metadata.max_x = metadata.max_y = metadata.max_z = 1;
  metadata.x_offset = metadata.y_offset = metadata.z_offset = 0;
  metadata.x_scale_factor = metadata.y_scale_factor = metadata.z_scale_factor = 0.001f;
  metadata.offset_to_point_data = metadata.header_size;
  metadata.number_of_variable_length_records = 0;
  metadata.point_data_format = 2;
  metadata.point_data_record_length = 26;

  las_file.set_metadata(metadata);

  // Write point_count garbage points. We only care about the throughput, not
  // about the actual data
  std::generate_n(std::begin(las_file), point_count, []() -> laszip_point { return {}; });

  las_file.close();
}

static void
run_read_test(const Args& args, bool sequential)
{
  const auto las_files = get_all_las_files_in_directory(args.source_directory);

  const auto total_size = std::accumulate(
    std::begin(las_files),
    std::end(las_files),
    static_cast<size_t>(0),
    [](size_t total_size, const fs::path& file) { return total_size + fs::file_size(file); });

  std::cout << std::fixed << std::setprecision(2);

  const std::string type_text = sequential ? "sequential" : "parallel";
  const auto num_threads = sequential ? 1ull : std::thread::hardware_concurrency();

  std::cout << "Running read test (" << type_text << "):"
            << "\n\tTotal files:         " << las_files.size()
            << "\n\tTotal size:          " << format_memory_size(total_size) << "\n";

  const auto benchmark_start_time = std::chrono::high_resolution_clock::now();

  const auto result_data = analyze_files(las_files, num_threads);

  const auto benchmark_end_time = std::chrono::high_resolution_clock::now();
  const auto benchmark_time = benchmark_end_time - benchmark_start_time;

  std::cout << "\n\tTotal points:        " << result_data.point_count
            << "\n\tMinimum point:       " << result_data.min_pos
            << "\n\tMaximum point:       " << result_data.max_pos << "\n\n";

  const auto points_per_second =
    static_cast<size_t>(result_data.point_count / (benchmark_time.count() / 1e9));
  const auto bytes_per_second = total_size / (benchmark_time.count() / 1e9);

  std::cout << "Benchmark finished:"
            << "\n\tTotal time:          " << format_time(benchmark_time)
            << "\n\tThroughput (memory): " << format_memory_size(bytes_per_second) << "/s"
            << "\n\tThroughput (points): " << unit::format_with_metric_prefix(points_per_second)
            << "/s\n";
}

static void
run_write_test(const Args& args, bool sequential)
{
  if (!fs::exists(args.output_directory)) {
    std::cerr << "Can't run write test: Output directory does not exist!\n";
    return;
  }

  constexpr static size_t PointCount = 10'000'000;
  const auto type_text = sequential ? ("sequential") : ("parallel");
  const auto num_threads =
    sequential ? static_cast<uint32_t>(1) : std::thread::hardware_concurrency();

  std::cout << "Running write test (" << type_text << ")\n";

  tf::Executor executor{ num_threads };
  tf::Taskflow las_taskflow, laz_taskflow;

  std::vector<fs::path> las_files, laz_files;

  for (uint32_t idx = 0; idx < num_threads; ++idx) {
    const auto las_file_name =
      fs::path{ concat(args.output_directory.string(), "/tmp_", idx, ".las") };
    const auto laz_file_name =
      fs::path{ concat(args.output_directory.string(), "/tmp_", idx, ".laz") };

    las_files.push_back(las_file_name);
    laz_files.push_back(laz_file_name);

    las_taskflow.emplace([las_file_name]() { write_random_file(las_file_name, PointCount); });

    laz_taskflow.emplace([laz_file_name]() { write_random_file(laz_file_name, PointCount); });
  }

  const auto las_run_begin = std::chrono::high_resolution_clock::now();
  executor.run(las_taskflow).wait();
  const auto las_run_end = std::chrono::high_resolution_clock::now();

  const auto laz_run_begin = std::chrono::high_resolution_clock::now();
  executor.run(laz_taskflow).wait();
  const auto laz_run_end = std::chrono::high_resolution_clock::now();

  const auto las_run_time = las_run_end - las_run_begin;
  const auto laz_run_time = laz_run_end - laz_run_begin;

  const auto las_files_total_size = std::accumulate(
    std::begin(las_files), std::end(las_files), size_t{ 0 }, [](size_t size, const fs::path& file) {
      return size + fs::file_size(file);
    });
  const auto laz_files_total_size = std::accumulate(
    std::begin(laz_files), std::end(laz_files), size_t{ 0 }, [](size_t size, const fs::path& file) {
      return size + fs::file_size(file);
    });

  const auto las_memory_throughput = las_files_total_size / (las_run_time.count() / 1e9);
  const auto laz_memory_throughput = laz_files_total_size / (laz_run_time.count() / 1e9);

  const auto las_point_throughput = (num_threads * PointCount) / (las_run_time.count() / 1e9);
  const auto laz_point_throughput = (num_threads * PointCount) / (laz_run_time.count() / 1e9);

  std::cout << "\n\tPoints written:                   " << (num_threads * PointCount);
  std::cout << "\n\tMemory throughput (uncompressed): " << format_memory_size(las_memory_throughput)
            << "/s";
  std::cout << "\n\tMemory throughput (compressed):   " << format_memory_size(laz_memory_throughput)
            << "/s";
  std::cout << "\n\tPoint throughput (uncompressed):  "
            << unit::format_with_metric_prefix(las_point_throughput) << "/s";
  std::cout << "\n\tPoint throughput (compressed):    "
            << unit::format_with_metric_prefix(laz_point_throughput) << "/s\n";

  for (auto& las_file : las_files) {
    fs::remove(las_file);
  }
  for (auto& laz_file : laz_files) {
    fs::remove(laz_file);
  }
}

int
main(int argc, char** argv)
{
  const auto args = parse_args(argc, argv);
  switch (args.test_type) {
    case TestType::ReadSequential:
      run_read_test(args, true);
      break;
    case TestType::ReadParallel:
      run_read_test(args, false);
      break;
    case TestType::WriteSequential:
      run_write_test(args, true);
      break;
    case TestType::WriteParallel:
      run_write_test(args, false);
      break;
  }
  return 0;
}
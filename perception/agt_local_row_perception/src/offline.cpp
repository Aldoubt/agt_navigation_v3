#include "agt_local_row_perception/core.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace row = agt_local_row_perception;
namespace fs = std::filesystem;
namespace
{
using Record = std::map<std::string, std::string>;
std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '"') {
      if (quoted && i + 1 < line.size() && line[i + 1] == '"') {field += '"'; ++i;}
      else {quoted = !quoted;}
    } else if (c == ',' && !quoted) {fields.push_back(field); field.clear();}
    else if (c != '\r') {field += c;}
  }
  if (quoted) {throw std::runtime_error("unterminated CSV quoted field");}
  fields.push_back(field);
  return fields;
}
class CsvReader
{
public:
  explicit CsvReader(const fs::path & path) : input_(path)
  {
    if (!input_) {throw std::runtime_error("cannot open " + path.string());}
    std::string line;
    if (!std::getline(input_, line)) {throw std::runtime_error("CSV missing header: " + path.string());}
    fields_ = split_csv(line);
  }
  bool next(Record & record)
  {
    std::string line;
    while (std::getline(input_, line)) {
      if (line.empty() || line.front() == '#') {continue;}
      const auto values = split_csv(line);
      if (values.size() != fields_.size()) {throw std::runtime_error("CSV column count mismatch");}
      record.clear();
      for (std::size_t i = 0; i < fields_.size(); ++i) {record[fields_[i]] = values[i];}
      return true;
    }
    return false;
  }
private:
  std::ifstream input_;
  std::vector<std::string> fields_;
};
std::string required(const Record & record, const std::string & key)
{
  const auto it = record.find(key);
  if (it == record.end() || it->second.empty()) {throw std::runtime_error("missing CSV column/value " + key);}
  return it->second;
}
double numeric(const Record & record, const std::string & key, double fallback = 0)
{
  const auto it = record.find(key);
  if (it == record.end() || it->second.empty()) {return fallback;}
  std::size_t consumed = 0;
  const double result = std::stod(it->second, &consumed);
  if (consumed != it->second.size() || !std::isfinite(result)) {throw std::runtime_error("invalid finite CSV number " + key);}
  return result;
}
std::uint64_t epoch(const Record & record)
{
  const auto it = record.find("odom_epoch");
  if (it == record.end() || it->second.empty()) {throw std::runtime_error("explicit odom_epoch is required");}
  std::size_t consumed = 0;
  const auto result = std::stoull(it->second, &consumed);
  if (consumed != it->second.size() || it->second.front() == '-') {throw std::runtime_error("invalid odom_epoch");}
  return result;
}
std::vector<row::Point> read_points(const fs::path & path, double base_stamp, bool deskewed)
{
  std::vector<row::Point> points;
  const auto append = [&](const Record & record) {
      row::Point point;
      for (const auto * key : {"x", "y", "z"}) {required(record, key);}
      point.xyz = {numeric(record, "x"), numeric(record, "y"), numeric(record, "z")};
      if (deskewed) {point.stamp = base_stamp;}
      else if (record.count("timestamp") && !record.at("timestamp").empty()) {point.stamp = numeric(record, "timestamp");}
      else if (record.count("offset_time") && !record.at("offset_time").empty()) {
        point.stamp = base_stamp + numeric(record, "offset_time") * 1e-9;
      } else {throw std::runtime_error("point timing missing; declare --input-deskewed only for already compensated data");}
      if (point.stamp <= 0 || point.stamp < base_stamp - 0.05 || point.stamp > base_stamp + 1.0) {
        throw std::runtime_error("point timestamp inconsistent with scan base stamp");
      }
      point.intensity = static_cast<float>(numeric(record, "intensity"));
      points.push_back(point);
    };
  if (path.extension() == ".csv") {
    CsvReader reader(path); Record record;
    while (reader.next(record)) {append(record);}
    return points;
  }
  // ASCII PCD is deliberately supported without a PCL dependency. Binary data must be exported.
  std::ifstream input(path);
  if (!input) {throw std::runtime_error("cannot open cloud " + path.string());}
  std::vector<std::string> fields;
  bool ascii = false;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream row_stream(line);
    std::string key; row_stream >> key;
    if (key == "FIELDS" || key == "FIELD") {
      std::string name; while (row_stream >> name) {fields.push_back(name);}
    } else if (key == "COUNT") {
      unsigned count; while (row_stream >> count) {
        if (count != 1) {throw std::runtime_error("PCD array fields must first be exported to CSV");}
      }
    } else if (key == "DATA") {
      std::string encoding; row_stream >> encoding;
      if (encoding != "ascii") {throw std::runtime_error("only DATA ascii PCD is supported; use CSV export for binary PCD");}
      ascii = true; break;
    }
  }
  if (!ascii || fields.empty()) {throw std::runtime_error("PCD fields/data header missing");}
  while (std::getline(input, line)) {
    if (line.empty()) {continue;}
    std::istringstream row_stream(line); Record record;
    for (const auto & name : fields) {
      std::string cell;
      if (!(row_stream >> cell)) {throw std::runtime_error("PCD row missing field " + name);}
      record[name] = cell;
    }
    append(record);
  }
  return points;
}
std::map<std::uint64_t, std::vector<row::Pose>> read_poses(const fs::path & path)
{
  CsvReader reader(path); Record record;
  std::map<std::uint64_t, std::vector<row::Pose>> poses;
  while (reader.next(record)) {
    for (const auto * key : {"stamp", "x", "y", "z", "qx", "qy", "qz", "qw"}) {required(record, key);}
    row::Pose pose;
    pose.stamp = numeric(record, "stamp"); pose.epoch = epoch(record);
    pose.odom_from_body.translation() = Eigen::Vector3d(
      numeric(record, "x"), numeric(record, "y"), numeric(record, "z"));
    Eigen::Quaterniond q(numeric(record, "qw"), numeric(record, "qx"), numeric(record, "qy"), numeric(record, "qz"));
    if (!std::isfinite(q.norm()) || q.norm() < 1e-6 || pose.stamp <= 0) {
      throw std::runtime_error("invalid pose timestamp or quaternion");
    }
    pose.odom_from_body.linear() = q.normalized().toRotationMatrix();
    auto & group = poses[pose.epoch];
    if (!group.empty() && pose.stamp <= group.back().stamp) {
      throw std::runtime_error("pose CSV must be strictly increasing within each epoch");
    }
    group.push_back(pose);
  }
  return poses;
}
std::string joined_reasons(const std::vector<std::string> & reasons)
{
  std::string result;
  for (const auto & reason : reasons) {if (!result.empty()) {result += ';';} result += reason;}
  return result;
}
const char * cell_color(row::CellClass classification)
{
  switch (classification) {
    case row::CellClass::Free: return "#37c871";
    case row::CellClass::Occupied: return "#ee4444";
    case row::CellClass::Robot: return "#5078de";
    case row::CellClass::Occluded: return "#e78a31";
    case row::CellClass::Stale: return "#888888";
    default: return "#252b36";
  }
}
void svg(const fs::path & path, const row::RowState & state, const row::Config & config)
{
  std::ofstream output(path);
  if (!output) {throw std::runtime_error("cannot write overlay " + path.string());}
  const double scale = 65.0;
  const double width = 2 * config.y_abs_max_m * scale;
  const double height = (config.x_max_m - config.x_min_m) * scale;
  const auto px = [&](double y) {return (config.y_abs_max_m - y) * scale;};
  const auto py = [&](double x) {return (config.x_max_m - x) * scale;};
  output << std::setprecision(9) << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
         << "\" height=\"" << height + 75 << "\" viewBox=\"0 0 " << width << ' ' << height + 75 << "\">\n"
         << "<rect width=\"100%\" height=\"100%\" fill=\"#151923\"/>\n";
  for (const auto & cell : state.cells) {
    output << "<rect x=\"" << px(cell.center.y() + config.grid_resolution_m / 2)
           << "\" y=\"" << py(cell.center.x() + config.grid_resolution_m / 2)
           << "\" width=\"" << config.grid_resolution_m * scale << "\" height=\""
           << config.grid_resolution_m * scale << "\" fill=\"" << cell_color(cell.classification) << "\"/>\n";
    if (cell.evidence_weight > 0) {
      output << "<circle cx=\"" << px(cell.center.y()) << "\" cy=\"" << py(cell.center.x())
             << "\" r=\"2\" fill=\"#ddddaa\"/>\n";
    }
  }
  if (state.left_support > 0) {
    for (const double intercept : {state.line_parameters.y(), state.line_parameters.z(),
      (state.line_parameters.y() + state.line_parameters.z()) / 2})
    {
      const double a = state.line_parameters.x();
      output << "<line x1=\"" << px(a * config.fit_x_min_m + intercept) << "\" y1=\""
             << py(config.fit_x_min_m) << "\" x2=\"" << px(a * config.fit_x_max_m + intercept)
             << "\" y2=\"" << py(config.fit_x_max_m) << "\" stroke=\"#33ddff\" stroke-width=\"2\"/>\n";
    }
  }
  output << "<text x=\"8\" y=\"" << height + 20 << "\" fill=\"white\" font-size=\"13\">stamp="
         << state.stamp << " epoch=" << state.odom_epoch << " row=" << state.row_valid << " Q=" << state.quality
         << "</text><text x=\"8\" y=\"" << height + 40 << "\" fill=\"white\" font-size=\"13\">ey="
         << state.lateral_error_m << " heading=" << state.heading_error_rad << " width=" << state.width_m
         << "</text><text x=\"8\" y=\"" << height + 60 << "\" fill=\"white\" font-size=\"11\">unknown dark / occupied red / self blue / occluded orange</text></svg>\n";
}
void usage()
{
  std::cout << "local_row_offline --scans scans.csv --poses poses.csv --output DIR [--config row.yaml] "
    "[--geometry geometry.yaml] [--extrinsics extrinsics.yaml] [--input-deskewed] "
    "[--allow-identity-extrinsics] [--field-verified]\n"
    "scans CSV: stamp,cloud_path,sensor_frame,odom_epoch; paths relative to scans.csv\n"
    "pose CSV: stamp,x,y,z,qx,qy,qz,qw,odom_epoch; poses are odom<-geometry.frame_id\n"
    "cloud CSV/ASCII PCD: x,y,z,timestamp(seconds) or offset_time(nanoseconds),intensity\n"
    "Extrinsics YAML uses geometry.sensor_transform: {translation_xyz: [...], rpy: [...]}\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    std::map<std::string, std::string> options;
    bool deskewed = false, identity_allowed = false, field_verified = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg(argv[i]);
      if (arg == "--help" || arg == "-h") {usage(); return 0;}
      if (arg == "--input-deskewed") {deskewed = true; continue;}
      if (arg == "--allow-identity-extrinsics") {identity_allowed = true; continue;}
      if (arg == "--field-verified") {field_verified = true; continue;}
      if (arg != "--scans" && arg != "--poses" && arg != "--output" && arg != "--config" &&
        arg != "--geometry" && arg != "--extrinsics") {throw std::runtime_error("unknown argument " + arg);}
      if (i + 1 >= argc) {throw std::runtime_error("missing value for " + arg);}
      options[arg] = argv[++i];
    }
    for (const auto * required_option : {"--scans", "--poses", "--output"}) {
      if (options[required_option].empty()) {usage(); throw std::runtime_error(std::string("missing ") + required_option);}
    }
    const row::Config config = row::load_config(options["--config"]);
    row::Geometry geometry = row::load_geometry(options["--geometry"]);
    if (!options["--extrinsics"].empty()) {
      const auto extrinsics = row::load_geometry(options["--extrinsics"]);
      if (!extrinsics.sensor_transform_supplied) {throw std::runtime_error("extrinsics file lacks sensor_transform");}
      geometry.body_from_sensor = extrinsics.body_from_sensor; geometry.sensor_transform_supplied = true;
    }
    if (!geometry.sensor_transform_supplied && !identity_allowed) {
      throw std::runtime_error("explicit sensor_transform required; --allow-identity-extrinsics only for body-frame data");
    }
    geometry.field_verified = geometry.field_verified && field_verified;
    row::RowPerception perception(config, geometry);
    row::PoseHistory history(config.pose_history_sec, config.pose_max_gap_sec);
    auto poses = read_poses(options["--poses"]);
    std::map<std::uint64_t, std::size_t> cursor;
    std::map<std::uint64_t, double> previous_stamp;
    const fs::path manifest_path = fs::absolute(options["--scans"]);
    const fs::path output_path = fs::absolute(options["--output"]);
    if (fs::exists(output_path)) {
      throw std::runtime_error("output directory already exists; choose a new run directory");
    }
    fs::create_directories(output_path / "overlays");
    std::ofstream states(output_path / "row_states.csv");
    std::ofstream centerlines(output_path / "centerlines.csv");
    if (!states || !centerlines) {throw std::runtime_error("output CSV cannot be written");}
    states << "stamp,odom_epoch,ground_valid,row_valid,clearance_valid,lateral_error_m,heading_error_rad,width_m,"
      "boundary_left_m,boundary_right_m,clearance_left_m,clearance_right_m,clearance_front_m,observed_forward_m,"
      "var_lateral_m2,cov_lateral_heading,var_heading_rad2,quality,quality_uncertainty,quality_support,quality_temporal,"
      "quality_width,left_support,right_support,frame_support,points_input,points_self,points_occluded,points_missing_pose,reasons\n";
    centerlines << "stamp,odom_epoch,index,x,y,z\n";
    states << std::setprecision(17); centerlines << std::setprecision(17);
    CsvReader scans(manifest_path); Record record;
    std::size_t frame_count = 0, valid_rows = 0, valid_clearance = 0;
    std::optional<std::uint64_t> current_epoch;
    std::optional<std::string> sensor_frame;
    while (scans.next(record)) {
      row::Scan scan;
      required(record, "stamp");
      const auto record_sensor_frame = required(record, "sensor_frame");
      if (sensor_frame && *sensor_frame != record_sensor_frame) {
        throw std::runtime_error("one offline run requires a fixed sensor_frame and its explicit extrinsics");
      }
      sensor_frame = record_sensor_frame;
      if (!geometry.sensor_transform_supplied && identity_allowed && record_sensor_frame != geometry.frame_id) {
        throw std::runtime_error("identity extrinsics requires sensor_frame to equal geometry.frame_id");
      }
      scan.stamp = numeric(record, "stamp"); scan.epoch = epoch(record);
      const double base_stamp = scan.stamp;
      auto cloud = fs::path(required(record, "cloud_path"));
      if (cloud.is_relative()) {cloud = manifest_path.parent_path() / cloud;}
      scan.points = read_points(cloud, base_stamp, deskewed);
      for (const auto & point : scan.points) {scan.stamp = std::max(scan.stamp, point.stamp);}
      scan.body_from_sensor = geometry.body_from_sensor;
      if (previous_stamp.count(scan.epoch) && scan.stamp <= previous_stamp[scan.epoch]) {
        throw std::runtime_error("scans must be strictly increasing within each epoch");
      }
      previous_stamp[scan.epoch] = scan.stamp;
      if (!current_epoch || *current_epoch != scan.epoch) {
        history.reset(scan.epoch); perception.reset(scan.epoch); current_epoch = scan.epoch;
        cursor[scan.epoch] = 0;
      }
      auto & sequence = poses[scan.epoch]; auto & index = cursor[scan.epoch];
      while (index < sequence.size() && (history.latest_stamp() < scan.stamp ||
        sequence[index].stamp <= scan.stamp)) {history.insert(sequence[index++]);}
      double speed_mps = std::numeric_limits<double>::quiet_NaN();
      const auto upper = std::lower_bound(sequence.begin(), sequence.end(), scan.stamp,
        [](const row::Pose & pose, double source_time) {return pose.stamp < source_time;});
      if (upper != sequence.begin() && upper != sequence.end()) {
        const auto lower = std::prev(upper);
        const double dt = upper->stamp - lower->stamp;
        const auto reference = history.at(scan.stamp, scan.epoch);
        if (reference && dt > 0 && dt <= config.pose_max_gap_sec) {
          const Eigen::Vector3d velocity = reference->odom_from_body.linear().transpose() *
            (upper->odom_from_body.translation() - lower->odom_from_body.translation()) / dt;
          speed_mps = std::copysign(velocity.head<2>().norm(), velocity.x());
        }
      }
      const auto state = perception.process(scan, history, speed_mps);
      states << state.stamp << ',' << state.odom_epoch << ',' << state.ground_valid << ',' << state.row_valid << ','
        << state.clearance_valid << ',' << state.lateral_error_m << ',' << state.heading_error_rad << ',' << state.width_m << ','
        << state.boundary_left_m << ',' << state.boundary_right_m << ',' << state.clearance_left_m << ',' << state.clearance_right_m << ','
        << state.clearance_front_m << ',' << state.observed_forward_m << ',' << state.error_covariance(0, 0) << ','
        << state.error_covariance(0, 1) << ',' << state.error_covariance(1, 1) << ',' << state.quality << ','
        << state.quality_uncertainty << ',' << state.quality_support << ',' << state.quality_temporal << ',' << state.quality_width << ','
        << state.left_support << ',' << state.right_support << ',' << state.frame_support << ',' << state.points_input << ','
        << state.points_self << ',' << state.points_occluded << ',' << state.points_missing_pose << ',' << joined_reasons(state.reasons) << '\n';
      for (std::size_t p = 0; p < state.odom_centerline.size(); ++p) {
        const auto & xyz = state.odom_centerline[p];
        centerlines << state.stamp << ',' << state.odom_epoch << ',' << p << ',' << xyz.x() << ',' << xyz.y() << ',' << xyz.z() << '\n';
      }
      std::ostringstream overlay_name; overlay_name << std::setfill('0') << std::setw(7) << frame_count << ".svg";
      svg(output_path / "overlays" / overlay_name.str(), state, config);
      ++frame_count; valid_rows += state.row_valid; valid_clearance += state.clearance_valid;
    }
    YAML::Emitter manifest;
    manifest << YAML::BeginMap << YAML::Key << "schema" << YAML::Value << "agt.local_row.offline.v1"
      << YAML::Key << "scans" << YAML::Value << manifest_path.string()
      << YAML::Key << "poses" << YAML::Value << fs::absolute(options["--poses"]).string()
      << YAML::Key << "geometry_frame" << YAML::Value << geometry.frame_id
      << YAML::Key << "sensor_frame" << YAML::Value << sensor_frame.value_or("")
      << YAML::Key << "field_verified" << YAML::Value << geometry.field_verified
      << YAML::Key << "input_deskewed" << YAML::Value << deskewed
      << YAML::Key << "speed_source" << YAML::Value << "pose finite difference; unavailable speed denies clearance"
      << YAML::Key << "identity_extrinsics_explicitly_allowed" << YAML::Value << identity_allowed
      << YAML::Key << "frame_count" << YAML::Value << frame_count
      << YAML::Key << "row_valid_frames" << YAML::Value << valid_rows
      << YAML::Key << "clearance_valid_frames" << YAML::Value << valid_clearance
      << YAML::Key << "quality_interpretation" << YAML::Value << "uncalibrated observability score, not probability"
      << YAML::Key << "closed_loop_acceptance" << YAML::Value << "not assessed" << YAML::EndMap;
    std::ofstream run_manifest(output_path / "manifest.yaml"); run_manifest << manifest.c_str() << '\n';
    for (const auto & option : {"--config", "--geometry", "--extrinsics"}) {
      if (!options[option].empty()) {
        fs::copy_file(options[option], output_path / (std::string(option).substr(2) + ".yaml"),
          fs::copy_options::overwrite_existing);
      }
    }
    std::cout << "Processed " << frame_count << " frames; row_valid=" << valid_rows
              << "; clearance_valid=" << valid_clearance << "; output=" << output_path << '\n';
    return 0;
  } catch (const std::exception & e) {std::cerr << "local_row_offline: " << e.what() << '\n'; return 1;}
}

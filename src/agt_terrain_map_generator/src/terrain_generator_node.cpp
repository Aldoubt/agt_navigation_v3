#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "agt_terrain_map_generator/elevation_builder.hpp"
#include "agt_terrain_map_generator/geometry/robot_geometry_provider.hpp"
#include "agt_terrain_map_generator/ground_segmenter.hpp"
#include "agt_terrain_map_generator/map_exporter.hpp"
#include "agt_terrain_map_generator/mapping_assets.hpp"
#include "agt_terrain_map_generator/obstacle_builder.hpp"
#include "agt_terrain_map_generator/patch_set_source.hpp"
#include "agt_terrain_map_generator/patchworkpp_ground_segmenter.hpp"
#include "agt_terrain_map_generator/preprocess/terrain_preprocessor.hpp"
#include "agt_terrain_map_generator/slope_builder.hpp"
#include "agt_terrain_map_generator/terrain/traversability_builder.hpp"
#include "agt_terrain_map_generator/terrain_types.hpp"
#include "agt_terrain_map_generator/trajectory/trajectory_carver.hpp"
#include "rclcpp/rclcpp.hpp"

class TerrainGeneratorNode : public rclcpp::Node
{
public:
  TerrainGeneratorNode()
  : Node("agt_terrain_map_generator")
  {
    pipeline_enabled_ = declare_parameter<bool>("pipeline.enabled", false);

    source_type_ = declare_parameter<std::string>("source.type", "patch_set");
    source_map_directory_ = declare_parameter<std::string>("source.map_directory", "");
    source_global_map_pcd_ = declare_parameter<std::string>("source.global_map_pcd", "");
    source_poses_txt_ = declare_parameter<std::string>("source.poses_txt", "");
    source_patches_directory_ = declare_parameter<std::string>("source.patches_directory", "");
    output_root_ = declare_parameter<std::string>("output_root", "~/.ros/agt_maps/latest");
    map_cleaning_enabled_ = declare_parameter<bool>("map_cleaning.enabled", true);
    robot_mesh_filter_enabled_ = declare_parameter<bool>("robot_mesh_filter.enabled", true);
    robot_mesh_filter_frame_ = declare_parameter<std::string>(
      "robot_mesh_filter.robot_frame", "base_link");
    robot_mesh_filter_lidar_frame_ = declare_parameter<std::string>(
      "robot_mesh_filter.lidar_frame", "lidar_link");
    robot_mesh_filter_urdf_path_ = declare_parameter<std::string>(
      "robot_mesh_filter.urdf_path", "");

    robot_body_center_ = declare_parameter<std::vector<double>>(
      "robot_geometry.body_box.center_xyz", {0.0, 0.0, 0.20});
    robot_body_size_ = declare_parameter<std::vector<double>>(
      "robot_geometry.body_box.size_xyz", {1.023, 0.778, 0.400});
    robot_body_padding_ = declare_parameter<double>("robot_geometry.body_box.padding_m", 0.05);
    self_filter_enabled_ = declare_parameter<bool>("self_filter.enabled", true);
    rear_filter_enabled_ = declare_parameter<bool>("rear_filter.enabled", false);
    rear_x_min_ = declare_parameter<double>("rear_filter.box.x_min", -2.0);
    rear_x_max_ = declare_parameter<double>("rear_filter.box.x_max", 0.0);
    rear_y_min_ = declare_parameter<double>("rear_filter.box.y_min", -0.8);
    rear_y_max_ = declare_parameter<double>("rear_filter.box.y_max", 0.8);
    rear_z_min_ = declare_parameter<double>("rear_filter.box.z_min", -0.5);
    rear_z_max_ = declare_parameter<double>("rear_filter.box.z_max", 2.0);
    voxel_enabled_ = declare_parameter<bool>("voxel.enabled", true);
    voxel_leaf_size_ = declare_parameter<double>("voxel.leaf_size_m", 0.20);

    ground_segmenter_type_ = declare_parameter<std::string>(
      "ground_segmenter.type", "patchworkpp");
    patchworkpp_sensor_height_ = declare_parameter<double>(
      "ground_segmenter.patchworkpp.sensor_height_m", 0.0);
    patchworkpp_min_range_ = declare_parameter<double>(
      "ground_segmenter.patchworkpp.min_range_m", 1.0);
    patchworkpp_max_range_ = declare_parameter<double>(
      "ground_segmenter.patchworkpp.max_range_m", 80.0);
    patchworkpp_verbose_ = declare_parameter<bool>(
      "ground_segmenter.patchworkpp.verbose", false);
    patchworkpp_enable_rnr_ = declare_parameter<bool>(
      "ground_segmenter.patchworkpp.enable_rnr", false);
    patchworkpp_enable_rvpf_ = declare_parameter<bool>(
      "ground_segmenter.patchworkpp.enable_rvpf", true);
    patchworkpp_enable_tgr_ = declare_parameter<bool>(
      "ground_segmenter.patchworkpp.enable_tgr", true);

    grid_resolution_ = declare_parameter<double>("grid.resolution_m", 0.05);
    tile_size_cells_ = declare_parameter<std::int64_t>("grid.tile_size_cells", 512);
    elevation_method_ = declare_parameter<std::string>("elevation.method", "median");
    elevation_min_points_ = declare_parameter<std::int64_t>(
      "elevation.min_points_per_cell", 3);
    elevation_max_variance_ = declare_parameter<double>(
      "elevation.max_variance_m2", 0.04);
    obstacle_min_height_ = declare_parameter<double>("obstacle.min_height_m", 0.25);
    obstacle_max_height_ = declare_parameter<double>("obstacle.max_height_m", 3.0);
    max_slope_deg_ = declare_parameter<double>("slope.max_angle_deg", 20.0);
    traversability_slope_safe_ = declare_parameter<double>("slope.safe_degree", 10.0);
    traversability_slope_warning_ = declare_parameter<double>("slope.warning_degree", 20.0);
    traversability_slope_blocked_ = declare_parameter<double>("slope.blocked_degree", 30.0);
    min_confidence_ = declare_parameter<double>("unknown.min_confidence", 0.5);
    trajectory_enabled_ = declare_parameter<bool>("trajectory.enabled", true);
    trajectory_robot_radius_ = declare_parameter<double>("trajectory.robot_radius_m", 0.45);
    trajectory_inflation_ = declare_parameter<double>("trajectory.inflation_m", 0.20);
    traversability_slope_weight_ = declare_parameter<double>("traversability.slope_weight", 0.35);
    traversability_obstacle_weight_ = declare_parameter<double>("traversability.obstacle_weight", 0.35);
    traversability_roughness_weight_ = declare_parameter<double>("traversability.roughness_weight", 0.15);
    traversability_confidence_weight_ = declare_parameter<double>("traversability.confidence_weight", 0.15);
    traversability_trajectory_bonus_ = declare_parameter<double>("traversability.trajectory_bonus", 0.10);
    traversability_obstacle_ignore_ = declare_parameter<double>(
      "traversability.obstacle_ignore_height_m", 0.15);
    traversability_obstacle_blocked_ = declare_parameter<double>(
      "traversability.obstacle_blocked_height_m", 0.50);
    traversability_roughness_variance_ = declare_parameter<double>(
      "traversability.roughness_variance_m2", 0.04);
    traversability_min_confidence_ = declare_parameter<double>(
      "traversability.min_confidence", 0.50);

    declare_parameter<bool>("output.save_elevation", true);
    declare_parameter<bool>("output.save_slope", true);
    declare_parameter<bool>("output.save_obstacle", true);
    declare_parameter<bool>("output.save_confidence", true);

    validate_parameters();
    initialize_terrain_preprocessor();
    initialize_v03_components();

    RCLCPP_INFO(
      get_logger(),
      "Terrain generator boundary ready: source=%s robot_mesh=%s ground=%s preprocess(self=%s rear=%s voxel=%s) trajectory=%s traversability=%.1f/%.1f/%.1fdeg resolution=%.3fm tile=%ld elevation=%s",
      source_type_.c_str(), (map_cleaning_enabled_ && robot_mesh_filter_enabled_) ? "on" : "off",
      ground_segmenter_type_.c_str(),
      self_filter_enabled_ ? "on" : "off", rear_filter_enabled_ ? "on" : "off",
      voxel_enabled_ ? "on" : "off", trajectory_enabled_ ? "on" : "off",
      traversability_slope_safe_, traversability_slope_warning_, traversability_slope_blocked_,
      grid_resolution_, static_cast<long>(tile_size_cells_), elevation_method_.c_str());

    if (pipeline_enabled_) {
      throw std::runtime_error(
              "pipeline.enabled=true is not allowed yet: PatchSetSource and terrain builders are not fully wired");
    }
  }

private:
  void validate_parameters() const
  {
    if (source_type_ != "patch_set" && source_type_ != "global_pcd") {
      throw std::runtime_error("source.type must be 'patch_set' or 'global_pcd'");
    }
    if (robot_mesh_filter_enabled_ &&
      (robot_mesh_filter_frame_.empty() || robot_mesh_filter_lidar_frame_.empty()))
    {
      throw std::runtime_error("robot_mesh_filter robot_frame and lidar_frame must be non-empty");
    }
    if (grid_resolution_ <= 0.0) {
      throw std::runtime_error("grid.resolution_m must be > 0");
    }
    if (tile_size_cells_ < 32) {
      throw std::runtime_error("grid.tile_size_cells must be >= 32");
    }
    if (elevation_min_points_ < 1) {
      throw std::runtime_error("elevation.min_points_per_cell must be >= 1");
    }
    if (elevation_max_variance_ <= 0.0) {
      throw std::runtime_error("elevation.max_variance_m2 must be > 0");
    }
    if (obstacle_min_height_ < 0.0 || obstacle_max_height_ <= obstacle_min_height_) {
      throw std::runtime_error("invalid obstacle height limits");
    }
    if (max_slope_deg_ <= 0.0 || max_slope_deg_ >= 90.0) {
      throw std::runtime_error("slope.max_angle_deg must be in (0, 90)");
    }
    if (traversability_slope_safe_ < 0.0 || traversability_slope_safe_ >= traversability_slope_warning_ ||
      traversability_slope_warning_ >= traversability_slope_blocked_ || traversability_slope_blocked_ >= 90.0)
    {
      throw std::runtime_error("invalid traversability slope bands");
    }
    if (trajectory_robot_radius_ <= 0.0 || trajectory_inflation_ < 0.0) {
      throw std::runtime_error("invalid trajectory robot radius or inflation");
    }
    if (traversability_slope_weight_ < 0.0 || traversability_obstacle_weight_ < 0.0 ||
      traversability_roughness_weight_ < 0.0 || traversability_confidence_weight_ < 0.0 ||
      traversability_trajectory_bonus_ < 0.0 || traversability_obstacle_ignore_ < 0.0 ||
      traversability_obstacle_ignore_ >= traversability_obstacle_blocked_ ||
      traversability_roughness_variance_ <= 0.0 || traversability_min_confidence_ < 0.0 ||
      traversability_min_confidence_ > 1.0)
    {
      throw std::runtime_error("invalid traversability parameters");
    }
    if (min_confidence_ < 0.0 || min_confidence_ > 1.0) {
      throw std::runtime_error("unknown.min_confidence must be in [0, 1]");
    }
    if (patchworkpp_min_range_ < 0.0 || patchworkpp_max_range_ <= patchworkpp_min_range_) {
      throw std::runtime_error("invalid Patchwork++ range limits");
    }
    if (patchworkpp_enable_rnr_) {
      throw std::runtime_error(
              "Patchwork++ RNR must stay disabled while mapping patches are XYZ-only");
    }
    if (robot_body_center_.size() != 3U || robot_body_size_.size() != 3U) {
      throw std::runtime_error("robot_geometry.body_box center_xyz and size_xyz must each have 3 values");
    }
    if (robot_body_padding_ < 0.0 || robot_body_size_[0] <= 0.0 ||
      robot_body_size_[1] <= 0.0 || robot_body_size_[2] <= 0.0)
    {
      throw std::runtime_error("invalid robot_geometry.body_box");
    }
    if (rear_x_min_ >= rear_x_max_ || rear_y_min_ >= rear_y_max_ || rear_z_min_ >= rear_z_max_) {
      throw std::runtime_error("invalid rear_filter.box limits");
    }
    if (voxel_enabled_ && voxel_leaf_size_ <= 0.0) {
      throw std::runtime_error("voxel.leaf_size_m must be > 0 when voxel is enabled");
    }
  }

  void initialize_terrain_preprocessor()
  {
    agt_terrain_map_generator::RobotBodyBox body_box;
    for (std::size_t index = 0; index < 3U; ++index) {
      body_box.center_xyz[index] = robot_body_center_[index];
      body_box.size_xyz[index] = robot_body_size_[index];
    }
    body_box.padding_m = robot_body_padding_;

    agt_terrain_map_generator::TerrainPreprocessorOptions options;
    options.self_filter.enabled = self_filter_enabled_;
    options.rear_filter.enabled = rear_filter_enabled_;
    options.rear_filter.box = {rear_x_min_, rear_x_max_, rear_y_min_, rear_y_max_, rear_z_min_, rear_z_max_};
    options.voxel.enabled = voxel_enabled_;
    options.voxel.leaf_size_m = voxel_leaf_size_;

    robot_geometry_ = std::make_unique<agt_terrain_map_generator::StaticRobotGeometryProvider>(body_box);
    terrain_preprocessor_ = std::make_unique<agt_terrain_map_generator::TerrainPreprocessor>(
      options, *robot_geometry_);
  }

  void initialize_v03_components()
  {
    agt_terrain_map_generator::TrajectoryCarverOptions trajectory_options;
    trajectory_options.enabled = trajectory_enabled_;
    trajectory_options.robot_radius_m = trajectory_robot_radius_;
    trajectory_options.inflation_m = trajectory_inflation_;
    trajectory_carver_ = std::make_unique<agt_terrain_map_generator::TrajectoryFreeCarver>(trajectory_options);

    agt_terrain_map_generator::TraversabilityOptions traversability_options;
    traversability_options.slope_weight = traversability_slope_weight_;
    traversability_options.obstacle_weight = traversability_obstacle_weight_;
    traversability_options.roughness_weight = traversability_roughness_weight_;
    traversability_options.confidence_weight = traversability_confidence_weight_;
    traversability_options.trajectory_bonus = traversability_trajectory_bonus_;
    traversability_options.slope_safe_degree = traversability_slope_safe_;
    traversability_options.slope_warning_degree = traversability_slope_warning_;
    traversability_options.slope_blocked_degree = traversability_slope_blocked_;
    traversability_options.obstacle_ignore_height_m = traversability_obstacle_ignore_;
    traversability_options.obstacle_blocked_height_m = traversability_obstacle_blocked_;
    traversability_options.roughness_variance_m2 = traversability_roughness_variance_;
    traversability_options.min_confidence = traversability_min_confidence_;
    traversability_builder_ = std::make_unique<agt_terrain_map_generator::TraversabilityBuilder>(
      traversability_options);
  }

  bool pipeline_enabled_{};
  std::string source_type_;
  std::string source_map_directory_;
  std::string source_global_map_pcd_;
  std::string source_poses_txt_;
  std::string source_patches_directory_;
  std::string output_root_;
  bool map_cleaning_enabled_{};
  bool robot_mesh_filter_enabled_{};
  std::string robot_mesh_filter_frame_;
  std::string robot_mesh_filter_lidar_frame_;
  std::string robot_mesh_filter_urdf_path_;

  std::vector<double> robot_body_center_;
  std::vector<double> robot_body_size_;
  double robot_body_padding_{};
  bool self_filter_enabled_{};
  bool rear_filter_enabled_{};
  double rear_x_min_{};
  double rear_x_max_{};
  double rear_y_min_{};
  double rear_y_max_{};
  double rear_z_min_{};
  double rear_z_max_{};
  bool voxel_enabled_{};
  double voxel_leaf_size_{};
  std::unique_ptr<agt_terrain_map_generator::StaticRobotGeometryProvider> robot_geometry_;
  std::unique_ptr<agt_terrain_map_generator::TerrainPreprocessor> terrain_preprocessor_;

  std::string ground_segmenter_type_;
  double patchworkpp_sensor_height_{};
  double patchworkpp_min_range_{};
  double patchworkpp_max_range_{};
  bool patchworkpp_verbose_{};
  bool patchworkpp_enable_rnr_{};
  bool patchworkpp_enable_rvpf_{};
  bool patchworkpp_enable_tgr_{};

  double grid_resolution_{};
  std::int64_t tile_size_cells_{};
  std::string elevation_method_;
  std::int64_t elevation_min_points_{};
  double elevation_max_variance_{};
  double obstacle_min_height_{};
  double obstacle_max_height_{};
  double max_slope_deg_{};
  double traversability_slope_safe_{};
  double traversability_slope_warning_{};
  double traversability_slope_blocked_{};
  double min_confidence_{};
  bool trajectory_enabled_{};
  double trajectory_robot_radius_{};
  double trajectory_inflation_{};
  double traversability_slope_weight_{};
  double traversability_obstacle_weight_{};
  double traversability_roughness_weight_{};
  double traversability_confidence_weight_{};
  double traversability_trajectory_bonus_{};
  double traversability_obstacle_ignore_{};
  double traversability_obstacle_blocked_{};
  double traversability_roughness_variance_{};
  double traversability_min_confidence_{};
  std::unique_ptr<agt_terrain_map_generator::TrajectoryFreeCarver> trajectory_carver_;
  std::unique_ptr<agt_terrain_map_generator::TraversabilityBuilder> traversability_builder_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}

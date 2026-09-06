#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "agt_terrain_map_generator/elevation_builder.hpp"
#include "agt_terrain_map_generator/ground_segmenter.hpp"
#include "agt_terrain_map_generator/map_exporter.hpp"
#include "agt_terrain_map_generator/mapping_assets.hpp"
#include "agt_terrain_map_generator/obstacle_builder.hpp"
#include "agt_terrain_map_generator/patch_set_source.hpp"
#include "agt_terrain_map_generator/patchworkpp_ground_segmenter.hpp"
#include "agt_terrain_map_generator/slope_builder.hpp"
#include "agt_terrain_map_generator/terrain_types.hpp"
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
    min_confidence_ = declare_parameter<double>("unknown.min_confidence", 0.5);

    declare_parameter<bool>("output.save_elevation", true);
    declare_parameter<bool>("output.save_slope", true);
    declare_parameter<bool>("output.save_obstacle", true);
    declare_parameter<bool>("output.save_confidence", true);

    validate_parameters();

    RCLCPP_INFO(
      get_logger(),
      "Terrain generator boundary ready: source=%s ground=%s resolution=%.3fm tile=%ld elevation=%s slope_limit=%.1fdeg",
      source_type_.c_str(), ground_segmenter_type_.c_str(), grid_resolution_,
      static_cast<long>(tile_size_cells_), elevation_method_.c_str(), max_slope_deg_);

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
  }

  bool pipeline_enabled_{};
  std::string source_type_;
  std::string source_map_directory_;
  std::string source_global_map_pcd_;
  std::string source_poses_txt_;
  std::string source_patches_directory_;
  std::string output_root_;

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
  double min_confidence_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}

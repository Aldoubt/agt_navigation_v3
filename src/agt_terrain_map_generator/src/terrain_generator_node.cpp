#include <memory>
#include <stdexcept>
#include <string>

#include "agt_terrain_map_generator/elevation_builder.hpp"
#include "agt_terrain_map_generator/ground_segmenter.hpp"
#include "agt_terrain_map_generator/map_exporter.hpp"
#include "agt_terrain_map_generator/obstacle_builder.hpp"
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
    input_pcd_ = declare_parameter<std::string>("input_pcd", "");
    output_root_ = declare_parameter<std::string>("output_root", "~/.ros/agt_maps/latest");
    ground_segmenter_type_ = declare_parameter<std::string>(
      "ground_segmenter.type", "patchworkpp");
    grid_resolution_ = declare_parameter<double>("grid.resolution_m", 0.05);
    elevation_method_ = declare_parameter<std::string>("elevation.method", "median");
    obstacle_min_height_ = declare_parameter<double>("obstacle.min_height_m", 0.25);
    obstacle_max_height_ = declare_parameter<double>("obstacle.max_height_m", 3.0);
    max_slope_deg_ = declare_parameter<double>("slope.max_angle_deg", 20.0);

    if (grid_resolution_ <= 0.0) {
      throw std::runtime_error("grid.resolution_m must be > 0");
    }
    if (obstacle_min_height_ < 0.0 || obstacle_max_height_ <= obstacle_min_height_) {
      throw std::runtime_error("invalid obstacle height limits");
    }
    if (max_slope_deg_ <= 0.0 || max_slope_deg_ >= 90.0) {
      throw std::runtime_error("slope.max_angle_deg must be in (0, 90)");
    }

    RCLCPP_INFO(
      get_logger(),
      "Terrain map generator skeleton ready: ground=%s resolution=%.3fm elevation=%s slope_limit=%.1fdeg",
      ground_segmenter_type_.c_str(), grid_resolution_, elevation_method_.c_str(), max_slope_deg_);

    if (pipeline_enabled_) {
      RCLCPP_WARN(
        get_logger(),
        "pipeline.enabled=true, but v0.1 is interface-only. Keep this disabled until the ground-segmentation adapter and builders are installed.");
    }
  }

private:
  bool pipeline_enabled_{};
  std::string input_pcd_;
  std::string output_root_;
  std::string ground_segmenter_type_;
  double grid_resolution_{};
  std::string elevation_method_;
  double obstacle_min_height_{};
  double obstacle_max_height_{};
  double max_slope_deg_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}

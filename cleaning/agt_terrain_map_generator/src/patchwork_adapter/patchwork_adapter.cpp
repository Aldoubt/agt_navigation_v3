#include "agt_terrain_map_generator/patchwork_adapter/patchwork_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace agt_terrain_map_generator
{
namespace
{
struct CellKey
{
  std::int64_t x;
  std::int64_t y;
  bool operator==(const CellKey & other) const {return x == other.x && y == other.y;}
};
struct CellKeyHash
{
  std::size_t operator()(const CellKey & key) const
  {
    return std::hash<std::int64_t>{}(key.x) ^ (std::hash<std::int64_t>{}(key.y) << 1U);
  }
};
bool finite(const pcl::PointXYZ & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}
}  // namespace

PatchworkAdapter::PatchworkAdapter(PatchworkAdapterOptions options)
: options_(options)
{
  if (!std::isfinite(options_.sensor_height_m) || options_.sensor_height_m <= 0.0 ||
    !std::isfinite(options_.cell_size_m) || options_.cell_size_m <= 0.0 ||
    !std::isfinite(options_.ground_tolerance_m) || options_.ground_tolerance_m < 0.0)
  {
    throw std::invalid_argument("invalid PatchworkAdapter options");
  }
}

bool PatchworkAdapter::segment(
  const pcl::PointCloud<pcl::PointXYZ> & input,
  GroundCloud & ground,
  NonGroundCloud & nonground)
{
  ground.clear();
  nonground.clear();
  if (input.empty()) {
    return false;
  }
  std::unordered_map<CellKey, float, CellKeyHash> lowest;
  for (const auto & point : input.points) {
    if (!finite(point)) {continue;}
    const CellKey key{static_cast<std::int64_t>(std::floor(point.x / options_.cell_size_m)),
      static_cast<std::int64_t>(std::floor(point.y / options_.cell_size_m))};
    const auto found = lowest.find(key);
    if (found == lowest.end() || point.z < found->second) {lowest[key] = point.z;}
  }
  if (lowest.empty()) {return false;}
  ground.reserve(input.size());
  nonground.reserve(input.size());
  for (const auto & point : input.points) {
    if (!finite(point)) {continue;}
    const CellKey key{static_cast<std::int64_t>(std::floor(point.x / options_.cell_size_m)),
      static_cast<std::int64_t>(std::floor(point.y / options_.cell_size_m))};
    if (point.z <= lowest.at(key) + options_.ground_tolerance_m) {ground.push_back(point);}
    else {nonground.push_back(point);}
  }
  ground.width = static_cast<std::uint32_t>(ground.size()); ground.height = 1U; ground.is_dense = true;
  nonground.width = static_cast<std::uint32_t>(nonground.size()); nonground.height = 1U; nonground.is_dense = true;
  return !ground.empty() || !nonground.empty();
}

}  // namespace agt_terrain_map_generator

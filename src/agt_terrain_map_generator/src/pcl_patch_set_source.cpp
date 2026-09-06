#include "agt_terrain_map_generator/pcl_patch_set_source.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace agt_terrain_map_generator
{
namespace fs = std::filesystem;

class PclPatchSetSource::Impl
{
public:
  std::vector<PatchAsset> patches;
  std::size_t cursor{0U};
};

PclPatchSetSource::PclPatchSetSource()
: impl_(std::make_unique<Impl>())
{
}

PclPatchSetSource::~PclPatchSetSource() = default;

bool PclPatchSetSource::open(const MappingAssetSet & assets, std::string & error)
{
  error.clear();
  impl_->patches.clear();
  impl_->cursor = 0U;

  try {
    fs::path map_dir;
    if (!assets.map_directory.empty()) {
      map_dir = fs::absolute(fs::path(assets.map_directory));
    }

    const fs::path poses_path = !assets.poses_txt.empty() ?
      fs::absolute(fs::path(assets.poses_txt)) : map_dir / "poses.txt";
    const fs::path patches_dir = !assets.patches_directory.empty() ?
      fs::absolute(fs::path(assets.patches_directory)) : map_dir / "patches";

    if (!fs::is_regular_file(poses_path)) {
      error = "poses.txt not found: " + poses_path.string();
      return false;
    }
    if (!fs::is_directory(patches_dir)) {
      error = "patches directory not found: " + patches_dir.string();
      return false;
    }

    std::ifstream stream(poses_path);
    if (!stream) {
      error = "failed to open poses file: " + poses_path.string();
      return false;
    }

    std::unordered_set<std::string> names;
    std::string line;
    std::size_t line_number = 0U;
    while (std::getline(stream, line)) {
      ++line_number;
      if (line.empty() || line.front() == '#') {
        continue;
      }

      std::istringstream row(line);
      PatchAsset patch;
      auto & t = patch.map_from_body;
      if (!(row >> patch.patch_name >> t.tx >> t.ty >> t.tz >> t.qw >> t.qx >> t.qy >> t.qz)) {
        error = "invalid poses row at line " + std::to_string(line_number);
        return false;
      }
      std::string extra;
      if (row >> extra) {
        error = "unexpected extra field in poses row at line " + std::to_string(line_number);
        return false;
      }
      if (!t.has_valid_rotation()) {
        error = "invalid quaternion in poses row at line " + std::to_string(line_number);
        return false;
      }
      if (!names.insert(patch.patch_name).second) {
        error = "duplicate patch name in poses.txt: " + patch.patch_name;
        return false;
      }

      const fs::path patch_path = patches_dir / patch.patch_name;
      if (!fs::is_regular_file(patch_path)) {
        error = "patch referenced by poses.txt is missing: " + patch_path.string();
        return false;
      }
      patch.patch_pcd = patch_path.string();
      impl_->patches.push_back(std::move(patch));
    }

    if (impl_->patches.empty()) {
      error = "poses.txt contains no usable patch records";
      return false;
    }
    return true;
  } catch (const std::exception & ex) {
    error = ex.what();
    impl_->patches.clear();
    impl_->cursor = 0U;
    return false;
  }
}

void PclPatchSetSource::reset()
{
  impl_->cursor = 0U;
}

bool PclPatchSetSource::next(
  PatchAsset & asset,
  PointCloud & body_cloud,
  std::string & error)
{
  error.clear();
  body_cloud.clear();

  if (impl_->cursor >= impl_->patches.size()) {
    return false;
  }

  asset = impl_->patches[impl_->cursor++];
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(asset.patch_pcd, cloud) != 0) {
    error = "failed to load patch PCD: " + asset.patch_pcd;
    return false;
  }

  body_cloud.reserve(cloud.size());
  for (const auto & point : cloud) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      body_cloud.push_back(Point3f{point.x, point.y, point.z});
    }
  }

  if (body_cloud.empty()) {
    error = "patch contains no finite XYZ points: " + asset.patch_pcd;
    return false;
  }
  return true;
}

}  // namespace agt_terrain_map_generator

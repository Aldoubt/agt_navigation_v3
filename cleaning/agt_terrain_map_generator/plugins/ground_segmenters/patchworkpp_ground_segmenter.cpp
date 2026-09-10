#include "agt_terrain_map_generator/patchworkpp_ground_segmenter.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>
#include <patchwork/patchworkpp.h>

namespace agt_terrain_map_generator
{
namespace
{

patchwork::Params make_params(const PatchworkppOptions & options)
{
  if (!std::isfinite(options.sensor_height_m) || options.sensor_height_m <= 0.0) {
    throw std::invalid_argument(
            "Patchwork++ sensor_height_m must be measured for the body-frame patch origin and be > 0");
  }
  if (!std::isfinite(options.min_range_m) || !std::isfinite(options.max_range_m) ||
    options.min_range_m < 0.0 || options.max_range_m <= options.min_range_m)
  {
    throw std::invalid_argument("invalid Patchwork++ range limits");
  }
  if (options.enable_rnr) {
    throw std::invalid_argument(
            "Patchwork++ RNR cannot be enabled for AGT XYZ-only mapping patches because intensity is unavailable");
  }

  patchwork::Params params;
  params.verbose = options.verbose;
  params.enable_RNR = false;
  params.enable_RVPF = options.enable_rvpf;
  params.enable_TGR = options.enable_tgr;
  params.sensor_height = options.sensor_height_m;
  params.min_range = options.min_range_m;
  params.max_range = options.max_range_m;
  return params;
}

PointCloud from_eigen(const Eigen::MatrixX3f & cloud)
{
  PointCloud output;
  output.reserve(static_cast<std::size_t>(cloud.rows()));
  for (Eigen::Index i = 0; i < cloud.rows(); ++i) {
    const float x = cloud(i, 0);
    const float y = cloud(i, 1);
    const float z = cloud(i, 2);
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      output.push_back(Point3f{x, y, z});
    }
  }
  return output;
}

}  // namespace

class PatchworkppGroundSegmenter::Impl
{
public:
  explicit Impl(const PatchworkppOptions & options)
  : params(make_params(options)), segmenter(params)
  {
  }

  patchwork::Params params;
  patchwork::PatchWorkpp segmenter;
};

PatchworkppGroundSegmenter::PatchworkppGroundSegmenter(const PatchworkppOptions & options)
: impl_(std::make_unique<Impl>(options))
{
}

PatchworkppGroundSegmenter::~PatchworkppGroundSegmenter() = default;

std::string PatchworkppGroundSegmenter::name() const
{
  return "patchworkpp";
}

bool PatchworkppGroundSegmenter::process(
  const PointCloud & input,
  PointCloud & ground,
  PointCloud & non_ground)
{
  ground.clear();
  non_ground.clear();
  if (input.empty()) {
    return false;
  }

  try {
    // Upstream Patchwork++ expects x/y/z/intensity. AGT mapping patches currently
    // expose XYZ only, so use a neutral synthetic intensity and keep RNR disabled.
    Eigen::MatrixXf cloud(static_cast<Eigen::Index>(input.size()), 4);
    for (std::size_t i = 0; i < input.size(); ++i) {
      const auto & point = input[i];
      cloud(static_cast<Eigen::Index>(i), 0) = point.x;
      cloud(static_cast<Eigen::Index>(i), 1) = point.y;
      cloud(static_cast<Eigen::Index>(i), 2) = point.z;
      cloud(static_cast<Eigen::Index>(i), 3) = 1.0F;
    }

    impl_->segmenter.estimateGround(std::move(cloud));
    ground = from_eigen(impl_->segmenter.getGround());
    non_ground = from_eigen(impl_->segmenter.getNonground());
    return !ground.empty() || !non_ground.empty();
  } catch (const std::exception &) {
    ground.clear();
    non_ground.clear();
    return false;
  }
}

}  // namespace agt_terrain_map_generator

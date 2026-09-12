#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace agt_relocalization {

struct PolarContextParams {
  int rings{20};
  int sectors{60};
  double max_radius{35.0};
  double min_radius{0.5};
  double z_offset{3.0};
  double max_height{30.0};
};

struct PolarContext {
  std::vector<double> ring_key;
  std::vector<double> sector_key;
};

struct PolarContextEntry {
  std::string patch_name;
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  PolarContext descriptor;
};

inline double wrap_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double yaw_from_quaternion(const Eigen::Quaterniond& q_in) {
  const Eigen::Quaterniond q = q_in.normalized();
  const double siny = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny, cosy);
}

inline PolarContext make_polar_context(
    const std::vector<Eigen::Vector3d>& points,
    const PolarContextParams& p = PolarContextParams()) {
  if (p.rings <= 0 || p.sectors <= 0 || p.max_radius <= p.min_radius) {
    throw std::runtime_error("invalid polar-context parameters");
  }

  std::vector<double> cells(static_cast<std::size_t>(p.rings * p.sectors), 0.0);
  for (const auto& point : points) {
    if (!point.allFinite()) continue;
    const double radius = std::hypot(point.x(), point.y());
    if (radius < p.min_radius || radius >= p.max_radius) continue;

    double angle = std::atan2(point.y(), point.x());
    if (angle < 0.0) angle += 2.0 * M_PI;
    const int ring = std::clamp(
      static_cast<int>(radius / p.max_radius * p.rings), 0, p.rings - 1);
    const int sector = std::clamp(
      static_cast<int>(angle / (2.0 * M_PI) * p.sectors), 0, p.sectors - 1);

    const double height = std::clamp(point.z() + p.z_offset, 0.0, p.max_height);
    double& cell = cells[static_cast<std::size_t>(ring * p.sectors + sector)];
    cell = std::max(cell, height);
  }

  PolarContext out;
  out.ring_key.assign(static_cast<std::size_t>(p.rings), 0.0);
  out.sector_key.assign(static_cast<std::size_t>(p.sectors), 0.0);

  for (int r = 0; r < p.rings; ++r) {
    for (int s = 0; s < p.sectors; ++s) {
      const double v = cells[static_cast<std::size_t>(r * p.sectors + s)];
      out.ring_key[static_cast<std::size_t>(r)] += v;
      out.sector_key[static_cast<std::size_t>(s)] += v;
    }
    out.ring_key[static_cast<std::size_t>(r)] /= static_cast<double>(p.sectors);
  }
  for (double& v : out.sector_key) {
    v /= static_cast<double>(p.rings);
  }
  return out;
}

inline std::vector<Eigen::Vector3d> level_patch_points(
    const std::vector<Eigen::Vector3d>& body_points,
    const Eigen::Quaterniond& map_q_body) {
  const Eigen::Quaterniond q = map_q_body.normalized();
  const double yaw = yaw_from_quaternion(q);
  const Eigen::AngleAxisd undo_yaw(-yaw, Eigen::Vector3d::UnitZ());
  const Eigen::Matrix3d rotation = undo_yaw.toRotationMatrix() * q.toRotationMatrix();

  std::vector<Eigen::Vector3d> out;
  out.reserve(body_points.size());
  for (const auto& p : body_points) {
    if (p.allFinite()) out.push_back(rotation * p);
  }
  return out;
}

inline double ring_key_distance(const PolarContext& a, const PolarContext& b) {
  if (a.ring_key.size() != b.ring_key.size() || a.ring_key.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double sum = 0.0;
  for (std::size_t i = 0; i < a.ring_key.size(); ++i) {
    const double d = a.ring_key[i] - b.ring_key[i];
    sum += d * d;
  }
  return std::sqrt(sum / static_cast<double>(a.ring_key.size()));
}

inline double cosine_similarity_shifted(
    const std::vector<double>& query,
    const std::vector<double>& candidate,
    int shift) {
  if (query.size() != candidate.size() || query.empty()) return -1.0;
  const int n = static_cast<int>(query.size());
  double dot = 0.0, nq = 0.0, nc = 0.0;
  for (int i = 0; i < n; ++i) {
    const int j = (i + shift + n) % n;
    const double a = query[static_cast<std::size_t>(i)];
    const double b = candidate[static_cast<std::size_t>(j)];
    dot += a * b;
    nq += a * a;
    nc += b * b;
  }
  if (nq <= 1e-12 || nc <= 1e-12) return -1.0;
  return dot / std::sqrt(nq * nc);
}

inline std::pair<double, int> best_sector_shift(
    const PolarContext& query,
    const PolarContext& candidate) {
  if (query.sector_key.size() != candidate.sector_key.size() ||
      query.sector_key.empty()) {
    return {-1.0, 0};
  }
  double best = -1.0;
  int best_shift = 0;
  const int n = static_cast<int>(query.sector_key.size());
  for (int shift = 0; shift < n; ++shift) {
    const double sim = cosine_similarity_shifted(
      query.sector_key, candidate.sector_key, shift);
    if (sim > best) {
      best = sim;
      best_shift = shift;
    }
  }
  if (best_shift > n / 2) best_shift -= n;
  return {best, best_shift};
}

inline void save_polar_context_db(
    const std::string& path,
    const std::vector<PolarContextEntry>& entries,
    const PolarContextParams& p) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("failed to create polar-context database: " + path);
  out.precision(10);
  out << "# AGT_POLAR_CONTEXT_V1 "
      << p.rings << " " << p.sectors << " "
      << p.max_radius << " " << p.min_radius << " "
      << p.z_offset << " " << p.max_height << "\n";
  for (const auto& e : entries) {
    out << e.patch_name << " "
        << e.translation.x() << " " << e.translation.y() << " " << e.translation.z() << " "
        << e.orientation.w() << " " << e.orientation.x() << " "
        << e.orientation.y() << " " << e.orientation.z();
    for (double v : e.descriptor.ring_key) out << " " << v;
    for (double v : e.descriptor.sector_key) out << " " << v;
    out << "\n";
  }
}

inline std::vector<PolarContextEntry> load_polar_context_db(
    const std::string& path,
    PolarContextParams& params) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("failed to open polar-context database: " + path);

  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("empty polar-context database");
  {
    std::istringstream header(line);
    std::string hash, tag;
    header >> hash >> tag
           >> params.rings >> params.sectors
           >> params.max_radius >> params.min_radius
           >> params.z_offset >> params.max_height;
    if (hash != "#" || tag != "AGT_POLAR_CONTEXT_V1" ||
        params.rings <= 0 || params.sectors <= 0) {
      throw std::runtime_error("unsupported polar-context database header");
    }
  }

  std::vector<PolarContextEntry> entries;
  while (std::getline(in, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::istringstream row(line);
    PolarContextEntry e;
    double qw, qx, qy, qz;
    if (!(row >> e.patch_name
              >> e.translation.x() >> e.translation.y() >> e.translation.z()
              >> qw >> qx >> qy >> qz)) {
      continue;
    }
    e.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
    e.descriptor.ring_key.resize(static_cast<std::size_t>(params.rings));
    e.descriptor.sector_key.resize(static_cast<std::size_t>(params.sectors));
    bool ok = true;
    for (double& v : e.descriptor.ring_key) ok = ok && static_cast<bool>(row >> v);
    for (double& v : e.descriptor.sector_key) ok = ok && static_cast<bool>(row >> v);
    if (ok) entries.push_back(std::move(e));
  }
  if (entries.empty()) throw std::runtime_error("polar-context database contains no entries");
  return entries;
}

}  // namespace agt_relocalization

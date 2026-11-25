// frontier_node.cpp  (v2)
// Frontier detection on /octomap_full (free space) with a switchable obstacle source:
//   obstacle_source = "octomap"    : occupied octomap leaves, grid rebuilt each cycle from the configured extent
//   obstacle_source = "laser_map"  : dense point cloud (e.g. /Laser_map), accumulated into a persistent grid
//
// Obstacles are voxelised into a dense grid over the planner box and inflated by a sphere of
// `inflation_radius`. Inflation is applied ONCE per newly occupied voxel, so a 10M point cloud costs
// one cheap lookup per point per message and nothing in the frontier cycle itself.
//
// Frontier rules:
//   cell    = free octomap voxel, not inside inflated obstacle space
//   unknown = octomap has no node there AND it is not inside inflated obstacle space AND inside the box
//   view ray blocked by: octomap occupied, grid obstacle (raw), or unknown

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <Eigen/Eigen>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

namespace plan_env
{

using KeySet = std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>;
using Vec3 = Eigen::Vector3d;

struct Viewpoint
{
  Vec3 pos;
  double yaw;
  int visib;
};

struct Frontier
{
  std::vector<Vec3> cells;
  std::vector<Vec3> filtered;
  Vec3 avg{0, 0, 0}, box_min{0, 0, 0}, box_max{0, 0, 0};
  std::vector<Viewpoint> viewpoints;
};

static double wrapAngle(double a)
{
  while (a < -M_PI) a += 2 * M_PI;
  while (a > M_PI) a -= 2 * M_PI;
  return a;
}

// Dense voxel grid over the planner box: raw obstacles + sphere-inflated obstacles.
struct ObstacleGrid
{
  Vec3 origin{0, 0, 0};
  double res{0.1}, inv{10.0};
  int nx{0}, ny{0}, nz{0};
  std::vector<uint8_t> occ;  // raw obstacle voxels
  std::vector<uint8_t> inf;  // raw + inflated
  std::vector<std::array<int, 3>> sphere;

  void init(const Vec3 & lo, const Vec3 & hi, double resolution, double inflation_radius)
  {
    origin = lo;
    res = resolution;
    inv = 1.0 / res;
    nx = std::max(1, static_cast<int>(std::ceil((hi.x() - lo.x()) * inv)));
    ny = std::max(1, static_cast<int>(std::ceil((hi.y() - lo.y()) * inv)));
    nz = std::max(1, static_cast<int>(std::ceil((hi.z() - lo.z()) * inv)));
    const size_t n = static_cast<size_t>(nx) * ny * nz;
    occ.assign(n, 0);
    inf.assign(n, 0);

    sphere.clear();
    const int rv = static_cast<int>(std::ceil(inflation_radius * inv));
    for (int dx = -rv; dx <= rv; ++dx)
      for (int dy = -rv; dy <= rv; ++dy)
        for (int dz = -rv; dz <= rv; ++dz)
          if ((dx * dx + dy * dy + dz * dz) * res * res <= inflation_radius * inflation_radius + 1e-9)
            sphere.push_back({dx, dy, dz});
    if (sphere.empty()) sphere.push_back({0, 0, 0});
  }

  void clear()
  {
    std::fill(occ.begin(), occ.end(), 0);
    std::fill(inf.begin(), inf.end(), 0);
  }

  size_t at(int x, int y, int z) const { return (static_cast<size_t>(x) * ny + y) * nz + z; }

  bool toIdx(const Vec3 & p, int & x, int & y, int & z) const
  {
    x = static_cast<int>(std::floor((p.x() - origin.x()) * inv));
    y = static_cast<int>(std::floor((p.y() - origin.y()) * inv));
    z = static_cast<int>(std::floor((p.z() - origin.z()) * inv));
    return x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz;
  }

  void toIdxClamped(const Vec3 & p, int & x, int & y, int & z) const
  {
    x = std::min(std::max(static_cast<int>(std::floor((p.x() - origin.x()) * inv)), 0), nx - 1);
    y = std::min(std::max(static_cast<int>(std::floor((p.y() - origin.y()) * inv)), 0), ny - 1);
    z = std::min(std::max(static_cast<int>(std::floor((p.z() - origin.z()) * inv)), 0), nz - 1);
  }

  Vec3 center(int x, int y, int z) const
  {
    return Vec3(origin.x() + (x + 0.5) * res, origin.y() + (y + 0.5) * res, origin.z() + (z + 0.5) * res);
  }

  // returns true if the voxel was newly occupied (and therefore inflated)
  bool add(int x, int y, int z)
  {
    const size_t i = at(x, y, z);
    if (occ[i]) return false;
    occ[i] = 1;
    for (const auto & o : sphere) {
      const int a = x + o[0], b = y + o[1], c = z + o[2];
      if (a < 0 || b < 0 || c < 0 || a >= nx || b >= ny || c >= nz) continue;
      inf[at(a, b, c)] = 1;
    }
    return true;
  }

  bool occAt(const Vec3 & p) const
  {
    int x, y, z;
    return toIdx(p, x, y, z) && occ[at(x, y, z)];
  }

  bool infAt(const Vec3 & p) const
  {
    int x, y, z;
    return toIdx(p, x, y, z) && inf[at(x, y, z)];
  }
};

class FrontierNode : public rclcpp::Node
{
public:
  FrontierNode() : Node("frontier_node")
  {
    // ---- planner box (also the obstacle grid extent) ----
    auto bmin = declare_parameter<std::vector<double>>("out_min", {-21.0, -15.0, -0.2});
    auto bmax = declare_parameter<std::vector<double>>("out_max", {21.0, 15.0, 7.8});
    out_min_ = Vec3(bmin[0], bmin[1], bmin[2]);
    out_max_ = Vec3(bmax[0], bmax[1], bmax[2]);

    // ---- obstacles ----
    obstacle_source_ = declare_parameter<std::string>("obstacle_source", "octomap");  // octomap | laser_map
    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.3);
    grid_res_ = declare_parameter<double>("grid_resolution", 0.1);
    obs_zmin_ = declare_parameter<double>("obstacle_z_min", -1000.0);  // use to drop floor points
    obs_zmax_ = declare_parameter<double>("obstacle_z_max", 1000.0);
    laser_stride_ = declare_parameter<int>("laser_stride", 1);          // use every Nth point
    laser_min_period_ = declare_parameter<double>("laser_min_period", 0.5);  // s, throttle heavy clouds
    laser_replace_ = declare_parameter<bool>("laser_replace", false);   // false: accumulate, true: clear per msg
    laser_best_effort_ = declare_parameter<bool>("laser_best_effort", false);

    // ---- detection ----
    iterate_range_ = declare_parameter<double>("iterate_range", 12.0);
    cluster_min_ = declare_parameter<int>("cluster_min", 30);
    cluster_size_xy_ = declare_parameter<double>("cluster_size_xy", 3.0);
    min_z_ = declare_parameter<double>("min_z", 0.4);
    min_unknown_ = declare_parameter<int>("min_unknown_nbrs", 1);
    down_sample_ = declare_parameter<int>("down_sample", 3);

    // ---- viewpoints ----
    cand_rmin_ = declare_parameter<double>("candidate_rmin", 1.5);
    cand_rmax_ = declare_parameter<double>("candidate_rmax", 4.0);
    cand_rnum_ = declare_parameter<int>("candidate_rnum", 3);
    cand_dphi_ = declare_parameter<double>("candidate_dphi", 0.26);
    unk_clear_xy_ = declare_parameter<double>("unknown_clearance_xy", 0.3);
    unk_clear_z_ = declare_parameter<double>("unknown_clearance_z", 0.2);
    min_visib_ = declare_parameter<int>("min_visib_num", 15);
    max_cells_eval_ = declare_parameter<int>("max_cells_eval", 60);
    hfov_ = declare_parameter<double>("hfov", 1.396);
    vfov_ = declare_parameter<double>("vfov", 1.047);
    max_view_ = declare_parameter<double>("max_view_range", 6.0);

    // ---- goal ----
    publish_goal_ = declare_parameter<bool>("publish_goal", false);
    goal_min_dist_ = declare_parameter<double>("goal_min_dist", 1.0);
    goal_max_dist_ = declare_parameter<double>("goal_max_dist", 8.0);
    goal_reach_dist_ = declare_parameter<double>("goal_reach_dist", 0.6);
    goal_timeout_ = declare_parameter<double>("goal_timeout", 15.0);
    visib_weight_ = declare_parameter<double>("visib_weight", 0.02);

    const double period = declare_parameter<double>("period", 0.5);

    if (obstacle_source_ != "octomap" && obstacle_source_ != "laser_map") {
      RCLCPP_WARN(get_logger(), "obstacle_source '%s' unknown, using octomap", obstacle_source_.c_str());
      obstacle_source_ = "octomap";
    }
    use_laser_ = (obstacle_source_ == "laser_map");

    grid_.init(out_min_, out_max_, grid_res_, inflation_radius_);
    RCLCPP_INFO(get_logger(), "obstacle source: %s | grid %dx%dx%d (%.0f MB) | inflation %.2f m (%zu offsets)",
                obstacle_source_.c_str(), grid_.nx, grid_.ny, grid_.nz,
                2.0 * grid_.occ.size() / 1e6, inflation_radius_, grid_.sphere.size());

    octo_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        "octomap_full", rclcpp::QoS(1),
        [this](octomap_msgs::msg::Octomap::ConstSharedPtr m) {
          std::lock_guard<std::mutex> l(mtx_);
          octo_ = m;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr o) {
          std::lock_guard<std::mutex> l(mtx_);
          pos_ = Vec3(o->pose.pose.position.x, o->pose.pose.position.y, o->pose.pose.position.z);
          has_odom_ = true;
        });

    if (use_laser_) {
      laser_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions opt;
      opt.callback_group = laser_group_;
      rclcpp::QoS qos(1);
      if (laser_best_effort_) qos.best_effort();
      laser_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "laser_map", qos,
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { laserCallback(m); }, opt);
    }

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("frontier/markers", 1);
    centers_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("frontier/centers", 1);
    views_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("frontier/viewpoints", 1);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("frontier/goal", 1);
    inflated_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("frontier/inflated_obstacles", 1);

    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] { process(); });
  }

private:
  // ================================================================== obstacle sources
  // Laser map: accumulate into the persistent grid. Only NEW voxels cost an inflation.
  void laserCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const rclcpp::Time t = now();
    if ((t - last_laser_).seconds() < laser_min_period_) return;  // skip: heavy cloud arrives too fast
    last_laser_ = t;

    int ox = -1, oy = -1, oz = -1;
    for (const auto & f : msg->fields) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) continue;
      if (f.name == "x") ox = static_cast<int>(f.offset);
      if (f.name == "y") oy = static_cast<int>(f.offset);
      if (f.name == "z") oz = static_cast<int>(f.offset);
    }
    if (ox < 0 || oy < 0 || oz < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "laser_map needs float32 x,y,z fields");
      return;
    }
    const size_t n = static_cast<size_t>(msg->width) * msg->height;
    if (n == 0 || msg->data.size() < n * msg->point_step) return;
    const size_t stride = static_cast<size_t>(std::max(1, laser_stride_));

    const auto t0 = std::chrono::steady_clock::now();
    size_t added = 0;
    {
      std::lock_guard<std::mutex> gl(grid_mtx_);
      if (laser_replace_) grid_.clear();
      const uint8_t * base = msg->data.data();
      for (size_t i = 0; i < n; i += stride) {
        const uint8_t * p = base + i * msg->point_step;
        float x, y, z;
        std::memcpy(&x, p + ox, 4);
        std::memcpy(&y, p + oy, 4);
        std::memcpy(&z, p + oz, 4);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        if (z < obs_zmin_ || z > obs_zmax_) continue;
        int ix, iy, iz;
        if (!grid_.toIdx(Vec3(x, y, z), ix, iy, iz)) continue;
        if (grid_.add(ix, iy, iz)) ++added;
      }
      laser_received_ = true;
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(get_logger(), "laser_map: %zu pts, +%zu new voxels, %.1f ms", n, added, ms);
  }

  // Octomap obstacles: rebuild grid from occupied leaves around the drone (octomap can un-occupy cells).
  void rebuildFromOctomap(const Vec3 & pos, bool has_odom)
  {
    grid_.clear();
    const int dims[3] = {grid_.nx, grid_.ny, grid_.nz};
    const double org[3] = {grid_.origin.x(), grid_.origin.y(), grid_.origin.z()};

    auto addLeaf = [&](const auto & it) {
      if (!tree_->isNodeOccupied(*it)) return;
      const double h = it.getSize() * 0.5;
      const octomap::point3d c = it.getCoordinate();
      int lo[3], hi[3];
      for (int i = 0; i < 3; ++i) {
        lo[i] = static_cast<int>(std::floor((c(i) - h - org[i]) * grid_.inv + 1e-3));
        hi[i] = static_cast<int>(std::ceil((c(i) + h - org[i]) * grid_.inv - 1e-3)) - 1;
        if (hi[i] < lo[i]) hi[i] = lo[i];
        if (hi[i] < 0 || lo[i] >= dims[i]) return;
        lo[i] = std::max(lo[i], 0);
        hi[i] = std::min(hi[i], dims[i] - 1);
      }
      for (int x = lo[0]; x <= hi[0]; ++x)
        for (int y = lo[1]; y <= hi[1]; ++y)
          for (int z = lo[2]; z <= hi[2]; ++z) grid_.add(x, y, z);
    };

    if (has_odom && iterate_range_ > 0.0) {
      // obstacles that can inflate into the iteration box: range + inflation radius + margin
      const double m = iterate_range_ + inflation_radius_ + 2 * res_;
      const octomap::point3d bmin(pos.x() - m, pos.y() - m, pos.z() - m);
      const octomap::point3d bmax(pos.x() + m, pos.y() + m, pos.z() + m);
      for (auto it = tree_->begin_leafs_bbx(bmin, bmax), end = tree_->end_leafs_bbx(); it != end; ++it)
        addLeaf(it);
    } else {
      for (auto it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it) addLeaf(it);
    }
  }

  // ================================================================== main cycle
  void process()
  {
    octomap_msgs::msg::Octomap::ConstSharedPtr msg;
    Vec3 pos;
    bool has_odom;
    {
      std::lock_guard<std::mutex> l(mtx_);
      msg = octo_;
      octo_.reset();
      pos = pos_;
      has_odom = has_odom_;
    }
    if (!msg) return;

    const auto t0 = std::chrono::steady_clock::now();

    std::unique_ptr<octomap::AbstractOcTree> abs(octomap_msgs::fullMsgToMap(*msg));
    auto * t = dynamic_cast<octomap::OcTree *>(abs.get());
    if (!t) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "need octomap_full holding an OcTree");
      return;
    }
    abs.release();
    tree_.reset(t);
    res_ = tree_->getResolution();

    std::lock_guard<std::mutex> gl(grid_mtx_);  // grid is shared with the laser callback

    if (use_laser_) {
      if (!laser_received_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for first laser_map message");
        return;
      }
    } else {
      rebuildFromOctomap(pos, has_odom);
    }

    KeySet cells;
    detectCells(pos, has_odom, cells);

    std::vector<Frontier> raw = cluster(cells);
    std::vector<Frontier> frontiers;
    for (auto & f : raw) splitFrontier(f, frontiers, 0);

    if (has_odom)
      for (auto & f : frontiers) sampleViewpoints(f);

    publish(frontiers, msg->header);
    publishInflated(pos, has_odom, msg->header);
    if (publish_goal_ && has_odom) selectGoal(frontiers, pos, msg->header);

    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "[%s] frontiers: %zu cells: %zu time: %.1f ms",
                         obstacle_source_.c_str(), frontiers.size(), cells.size(), ms);
  }

  // ================================================================== voxel predicates
  bool inOutBox(const Vec3 & p) const
  {
    return (p.array() >= out_min_.array()).all() && (p.array() <= out_max_.array()).all();
  }

  // unknown = no octomap node, not inside inflated obstacle space, inside the planner box
  bool isUnknownAt(const Vec3 & p) const
  {
    int x, y, z;
    if (!grid_.toIdx(p, x, y, z)) return false;  // beyond the planner box there is nothing to explore
    if (tree_->search(p.x(), p.y(), p.z()) != nullptr) return false;
    return grid_.inf[grid_.at(x, y, z)] == 0;
  }

  // ================================================================== detection
  void detectCells(const Vec3 & pos, bool has_odom, KeySet & cells)
  {
    static const int D[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

    auto handleLeaf = [&](const auto & it) {
      if (tree_->isNodeOccupied(*it)) return;
      const double size = it.getSize(), half = size * 0.5;
      const int n = std::max(1, static_cast<int>(std::lround(size / res_)));
      const octomap::point3d c = it.getCoordinate();
      const double x0 = c.x() - half, y0 = c.y() - half, z0 = c.z() - half;

      for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
          const bool edge_ij = (i == 0 || i == n - 1 || j == 0 || j == n - 1);
          const int step = edge_ij ? 1 : std::max(1, n - 1);
          for (int k = 0; k < n; k += step) {
            const Vec3 p(x0 + (i + 0.5) * res_, y0 + (j + 0.5) * res_, z0 + (k + 0.5) * res_);
            if (p.z() < min_z_ || !inOutBox(p)) continue;
            if (grid_.infAt(p)) continue;  // inside inflated obstacle: not a usable frontier cell
            int unk = 0;
            for (const auto & d : D) {
              const int a = i + d[0], b = j + d[1], e = k + d[2];
              if (a >= 0 && a < n && b >= 0 && b < n && e >= 0 && e < n) continue;
              if (isUnknownAt(Vec3(x0 + (a + 0.5) * res_, y0 + (b + 0.5) * res_, z0 + (e + 0.5) * res_)))
                ++unk;
            }
            if (unk < min_unknown_) continue;
            cells.insert(tree_->coordToKey(p.x(), p.y(), p.z()));
          }
        }
    };

    if (has_odom && iterate_range_ > 0.0) {
      const octomap::point3d bmin(pos.x() - iterate_range_, pos.y() - iterate_range_, pos.z() - iterate_range_);
      const octomap::point3d bmax(pos.x() + iterate_range_, pos.y() + iterate_range_, pos.z() + iterate_range_);
      for (auto it = tree_->begin_leafs_bbx(bmin, bmax), end = tree_->end_leafs_bbx(); it != end; ++it)
        handleLeaf(it);
    } else {
      for (auto it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it) handleLeaf(it);
    }
  }

  // ================================================================== clustering
  std::vector<Frontier> cluster(const KeySet & cells)
  {
    KeySet visited;
    std::vector<Frontier> out;
    for (const auto & seed : cells) {
      if (visited.count(seed)) continue;
      Frontier f;
      std::queue<octomap::OcTreeKey> q;
      q.push(seed);
      visited.insert(seed);
      while (!q.empty()) {
        const auto cur = q.front();
        q.pop();
        const octomap::point3d p = tree_->keyToCoord(cur);
        f.cells.emplace_back(p.x(), p.y(), p.z());
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz) {
              if (!dx && !dy && !dz) continue;
              octomap::OcTreeKey nb(cur[0] + dx, cur[1] + dy, cur[2] + dz);
              if (cells.count(nb) && visited.insert(nb).second) q.push(nb);
            }
      }
      if (static_cast<int>(f.cells.size()) < cluster_min_) continue;
      computeInfo(f);
      out.push_back(std::move(f));
    }
    return out;
  }

  void computeInfo(Frontier & f)
  {
    f.avg.setZero();
    f.box_min = f.box_max = f.cells.front();
    for (const auto & c : f.cells) {
      f.avg += c;
      f.box_min = f.box_min.cwiseMin(c);
      f.box_max = f.box_max.cwiseMax(c);
    }
    f.avg /= static_cast<double>(f.cells.size());
    downsample(f.cells, f.filtered);
  }

  void downsample(const std::vector<Vec3> & in, std::vector<Vec3> & out)
  {
    const double leaf = res_ * std::max(1, down_sample_);
    std::map<std::array<long, 3>, std::pair<Vec3, int>> grid;
    for (const auto & p : in) {
      std::array<long, 3> k = {static_cast<long>(std::floor(p.x() / leaf)),
                               static_cast<long>(std::floor(p.y() / leaf)),
                               static_cast<long>(std::floor(p.z() / leaf))};
      auto & g = grid[k];
      if (g.second == 0) g.first.setZero();
      g.first += p;
      g.second++;
    }
    out.clear();
    for (const auto & kv : grid) out.push_back(kv.second.first / kv.second.second);
  }

  void splitFrontier(const Frontier & f, std::vector<Frontier> & out, int depth)
  {
    const Eigen::Vector2d mean = f.avg.head<2>();
    bool need = false;
    for (const auto & c : f.filtered)
      if ((c.head<2>() - mean).norm() > cluster_size_xy_) {
        need = true;
        break;
      }
    if (!need || depth >= 6 || static_cast<int>(f.cells.size()) < 2 * cluster_min_) {
      out.push_back(f);
      return;
    }

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto & c : f.filtered) {
      const Eigen::Vector2d d = c.head<2>() - mean;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(f.filtered.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
    const Eigen::Vector2d pc = es.eigenvectors().col(1);

    Frontier a, b;
    for (const auto & c : f.cells) ((c.head<2>() - mean).dot(pc) >= 0 ? a : b).cells.push_back(c);
    if (a.cells.empty() || b.cells.empty()) {
      out.push_back(f);
      return;
    }
    computeInfo(a);
    computeInfo(b);
    splitFrontier(a, out, depth + 1);
    splitFrontier(b, out, depth + 1);
  }

  // ================================================================== viewpoints
  // known free in octomap, outside inflated obstacles (O(1)), no unknown voxel nearby
  bool safeSample(const Vec3 & s)
  {
    if (!inOutBox(s)) return false;
    if (grid_.infAt(s)) return false;
    auto * n = tree_->search(s.x(), s.y(), s.z());
    if (!n || tree_->isNodeOccupied(n)) return false;

    const int vxy = std::max(1, static_cast<int>(std::ceil(unk_clear_xy_ / res_)));
    const int vz = std::max(1, static_cast<int>(std::ceil(unk_clear_z_ / res_)));
    const int sxy = std::max(1, vxy / 2), sz = std::max(1, vz / 2);
    for (int dx = -vxy; dx <= vxy; dx += sxy)
      for (int dy = -vxy; dy <= vxy; dy += sxy)
        for (int dz = -vz; dz <= vz; dz += sz)
          if (isUnknownAt(Vec3(s.x() + dx * res_, s.y() + dy * res_, s.z() + dz * res_))) return false;
    return true;
  }

  // a view ray is blocked by octomap-occupied, raw grid obstacle, or unknown space
  bool keyBlocked(const octomap::OcTreeKey & key)
  {
    auto * n = tree_->search(key);
    if (n && tree_->isNodeOccupied(n)) return true;
    const octomap::point3d c = tree_->keyToCoord(key);
    if (grid_.occAt(Vec3(c.x(), c.y(), c.z()))) return true;
    return n == nullptr;
  }

  int countVisible(const Vec3 & pos, double yaw, const std::vector<Vec3> & cells)
  {
    const double hh = 0.5 * hfov_, hv = 0.5 * vfov_;
    const int stride = std::max(1, static_cast<int>(cells.size()) / std::max(1, max_cells_eval_));
    const octomap::point3d o(pos.x(), pos.y(), pos.z());
    octomap::KeyRay ray;
    int vis = 0;
    for (size_t idx = 0; idx < cells.size(); idx += stride) {
      const Vec3 d = cells[idx] - pos;
      const double dist = d.norm();
      if (dist < 1e-3 || dist > max_view_) continue;
      if (hfov_ < 6.2 && std::fabs(wrapAngle(std::atan2(d.y(), d.x()) - yaw)) > hh) continue;
      if (std::fabs(std::asin(d.z() / dist)) > hv) continue;
      const octomap::point3d e(cells[idx].x(), cells[idx].y(), cells[idx].z());
      if (!tree_->computeRayKeys(o, e, ray)) continue;
      bool ok = true;
      for (auto it = ray.begin(); it != ray.end(); ++it)
        if (keyBlocked(*it)) {
          ok = false;
          break;
        }
      if (ok) ++vis;
    }
    return vis * stride;
  }

  void sampleViewpoints(Frontier & f)
  {
    const int rnum = std::max(1, cand_rnum_);
    const double dr = (cand_rmax_ - cand_rmin_) / rnum;
    for (int ri = 0; ri <= rnum; ++ri) {
      const double rc = cand_rmin_ + ri * dr;
      for (double phi = -M_PI; phi < M_PI; phi += cand_dphi_) {
        Vec3 s = f.avg + rc * Vec3(std::cos(phi), std::sin(phi), 0.0);
        s.z() = std::max(s.z(), min_z_ + 0.3);
        if (!safeSample(s)) continue;
        const double yaw = std::atan2(f.avg.y() - s.y(), f.avg.x() - s.x());
        const int v = countVisible(s, yaw, f.filtered);
        if (v >= min_visib_) f.viewpoints.push_back({s, yaw, v});
      }
    }
    std::sort(f.viewpoints.begin(), f.viewpoints.end(),
              [](const Viewpoint & a, const Viewpoint & b) { return a.visib > b.visib; });
  }

  // ================================================================== goal
  void selectGoal(const std::vector<Frontier> & fs, const Vec3 & pos, const std_msgs::msg::Header & h)
  {
    if (have_goal_) {
      const bool reached = (pos - goal_).norm() < goal_reach_dist_;
      const bool timeout = (now() - goal_time_).seconds() > goal_timeout_;
      bool still_valid = false;
      for (const auto & f : fs)
        for (const auto & v : f.viewpoints)
          if ((v.pos - goal_).norm() < 1.5) still_valid = true;
      if (!reached && !timeout && still_valid) return;
    }

    double best_cost = 1e9;
    const Viewpoint * best = nullptr;
    for (const auto & f : fs)
      for (const auto & v : f.viewpoints) {
        const double d = (v.pos - pos).norm();
        if (d < goal_min_dist_ || d > goal_max_dist_) continue;
        const double cost = d - visib_weight_ * v.visib;
        if (cost < best_cost) {
          best_cost = cost;
          best = &v;
        }
        break;
      }
    if (!best) {
      have_goal_ = false;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "no reachable frontier viewpoint");
      return;
    }

    geometry_msgs::msg::PoseStamped g;
    g.header = h;
    g.pose.position.x = best->pos.x();
    g.pose.position.y = best->pos.y();
    g.pose.position.z = best->pos.z();
    g.pose.orientation.z = std::sin(best->yaw * 0.5);
    g.pose.orientation.w = std::cos(best->yaw * 0.5);
    goal_pub_->publish(g);
    goal_ = best->pos;
    goal_time_ = now();
    have_goal_ = true;
  }

  // ================================================================== output
  // Debug: inflated voxels in the selected bounds (only built if someone subscribes)
  void publishInflated(const Vec3 & pos, bool has_odom, const std_msgs::msg::Header & h)
  {
    if (inflated_pub_->get_subscription_count() == 0) return;
    Vec3 lo = out_min_, hi = out_max_;
    if (has_odom && iterate_range_ > 0.0) {
      lo = lo.cwiseMax(pos - Vec3::Constant(iterate_range_));
      hi = hi.cwiseMin(pos + Vec3::Constant(iterate_range_));
    }
    int x0, y0, z0, x1, y1, z1;
    grid_.toIdxClamped(lo, x0, y0, z0);
    grid_.toIdxClamped(hi, x1, y1, z1);

    std::vector<Vec3> pts;
    for (int x = x0; x <= x1; ++x)
      for (int y = y0; y <= y1; ++y)
        for (int z = z0; z <= z1; ++z)
          if (grid_.inf[grid_.at(x, y, z)]) pts.push_back(grid_.center(x, y, z));

    sensor_msgs::msg::PointCloud2 m;
    m.header = h;
    sensor_msgs::PointCloud2Modifier mod(m);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ix(m, "x"), iy(m, "y"), iz(m, "z");
    for (const auto & p : pts) {
      *ix = static_cast<float>(p.x());
      *iy = static_cast<float>(p.y());
      *iz = static_cast<float>(p.z());
      ++ix;
      ++iy;
      ++iz;
    }
    inflated_pub_->publish(m);
  }

  void publish(const std::vector<Frontier> & fs, const std_msgs::msg::Header & h)
  {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray arr;
    Marker clear;
    clear.action = Marker::DELETEALL;
    arr.markers.push_back(clear);

    geometry_msgs::msg::PoseArray centers, views;
    centers.header = h;
    views.header = h;

    int id = 0;
    for (const auto & f : fs) {
      Marker m;
      m.header = h;
      m.ns = "frontier_cells";
      m.id = id;
      m.type = Marker::POINTS;
      m.scale.x = m.scale.y = 0.08;
      m.color.a = 1.0;
      m.color.r = 0.2f + 0.8f * ((id * 37) % 10) / 10.f;
      m.color.g = 0.2f + 0.8f * ((id * 53) % 10) / 10.f;
      m.color.b = 0.9f;
      for (const auto & c : f.cells) {
        geometry_msgs::msg::Point p;
        p.x = c.x();
        p.y = c.y();
        p.z = c.z();
        m.points.push_back(p);
      }
      arr.markers.push_back(m);

      geometry_msgs::msg::Pose pc;
      // A geometric centroid can lie in a wall or in unknown space.
      const auto center = std::min_element(f.cells.begin(), f.cells.end(),
        [&](const Vec3 & a, const Vec3 & b) {
          return (a - f.avg).squaredNorm() < (b - f.avg).squaredNorm();
        });
      pc.position.x = center->x();
      pc.position.y = center->y();
      pc.position.z = center->z();
      pc.orientation.w = 1.0;
      centers.poses.push_back(pc);

      if (!f.viewpoints.empty()) {
        const auto & v = f.viewpoints.front();
        geometry_msgs::msg::Pose pv;
        pv.position.x = v.pos.x();
        pv.position.y = v.pos.y();
        pv.position.z = v.pos.z();
        pv.orientation.z = std::sin(v.yaw * 0.5);
        pv.orientation.w = std::cos(v.yaw * 0.5);
        views.poses.push_back(pv);

        Marker a;
        a.header = h;
        a.ns = "frontier_views";
        a.id = id;
        a.type = Marker::ARROW;
        a.pose = pv;
        a.scale.x = 0.6;
        a.scale.y = 0.1;
        a.scale.z = 0.1;
        a.color.a = 1.0;
        a.color.g = 1.0;
        arr.markers.push_back(a);
      }
      ++id;
    }
    marker_pub_->publish(arr);
    centers_pub_->publish(centers);
    views_pub_->publish(views);
  }

  // ================================================================== members
  Vec3 out_min_, out_max_;
  std::string obstacle_source_;
  bool use_laser_{false};
  double inflation_radius_, grid_res_, obs_zmin_, obs_zmax_, laser_min_period_;
  int laser_stride_;
  bool laser_replace_, laser_best_effort_;

  double iterate_range_, cluster_size_xy_, min_z_, res_{0.1};
  int cluster_min_, min_unknown_, down_sample_;

  double cand_rmin_, cand_rmax_, cand_dphi_, unk_clear_xy_, unk_clear_z_, hfov_, vfov_, max_view_;
  int cand_rnum_, min_visib_, max_cells_eval_;

  bool publish_goal_;
  double goal_min_dist_, goal_max_dist_, goal_reach_dist_, goal_timeout_, visib_weight_;
  bool have_goal_{false};
  Vec3 goal_{0, 0, 0};
  rclcpp::Time goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_laser_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<octomap::OcTree> tree_;
  ObstacleGrid grid_;
  bool laser_received_{false};

  std::mutex mtx_;       // octomap msg + odom
  std::mutex grid_mtx_;  // obstacle grid (laser callback <-> frontier cycle)
  octomap_msgs::msg::Octomap::ConstSharedPtr octo_;
  Vec3 pos_{0, 0, 0};
  bool has_odom_{false};

  rclcpp::CallbackGroup::SharedPtr laser_group_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octo_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr centers_pub_, views_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace plan_env

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<plan_env::FrontierNode>();
  // 2 threads: the heavy laser_map callback must not block the frontier timer's message handling
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
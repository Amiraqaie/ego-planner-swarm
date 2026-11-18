// frontier_node.cpp
// Frontier detection + clustering + viewpoint sampling directly on /octomap_full (ROS 2).
// Ported from FUEL's FrontierFinder. No dependency on GridMap buffers.
//
// Pipeline each cycle (recomputed from scratch):
//   1. deserialize octomap
//   2. free leaves near the drone whose 6-neighbour is unknown (tree->search == nullptr) -> frontier cells
//   3. 26-connected region growing in key space -> clusters
//   4. split big clusters along first principal axis (FUEL splitHorizontally)
//   5. sample viewpoints on circles around each cluster, check clearance + visibility
//   6. publish markers / viewpoints, optionally publish one goal for EGO planner

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <Eigen/Eigen>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
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
  std::vector<Vec3> cells;     // all cells
  std::vector<Vec3> filtered;  // downsampled cells (used for splitting / visibility)
  Vec3 avg{0, 0, 0}, box_min{0, 0, 0}, box_max{0, 0, 0};
  std::vector<Viewpoint> viewpoints;  // sorted, best first
};

static double wrapAngle(double a)
{
  while (a < -M_PI) a += 2 * M_PI;
  while (a > M_PI) a -= 2 * M_PI;
  return a;
}

class FrontierNode : public rclcpp::Node
{
public:
  FrontierNode() : Node("frontier_node")
  {
    // ---- detection ----
    iterate_range_ = declare_parameter<double>("iterate_range", 12.0);  // compute box only, not a map crop
    cluster_min_ = declare_parameter<int>("cluster_min", 30);
    cluster_size_xy_ = declare_parameter<double>("cluster_size_xy", 3.0);
    min_z_ = declare_parameter<double>("min_z", 0.4);
    min_unknown_ = declare_parameter<int>("min_unknown_nbrs", 1);
    down_sample_ = declare_parameter<int>("down_sample", 3);  // leaf = resolution * down_sample
    auto bmin = declare_parameter<std::vector<double>>("out_min", {-21.0, -15.0, -0.2});
    auto bmax = declare_parameter<std::vector<double>>("out_max", {21.0, 15.0, 7.8});
    out_min_ = Vec3(bmin[0], bmin[1], bmin[2]);
    out_max_ = Vec3(bmax[0], bmax[1], bmax[2]);

    // ---- viewpoints ----
    cand_rmin_ = declare_parameter<double>("candidate_rmin", 1.5);
    cand_rmax_ = declare_parameter<double>("candidate_rmax", 4.0);
    cand_rnum_ = declare_parameter<int>("candidate_rnum", 3);
    cand_dphi_ = declare_parameter<double>("candidate_dphi", 0.26);  // rad
    clear_xy_ = declare_parameter<double>("clearance_xy", 0.5);
    clear_z_ = declare_parameter<double>("clearance_z", 0.3);
    min_visib_ = declare_parameter<int>("min_visib_num", 15);
    max_cells_eval_ = declare_parameter<int>("max_cells_eval", 60);
    hfov_ = declare_parameter<double>("hfov", 1.396);  // 80 deg; use 6.283 for a 360 deg LiDAR
    vfov_ = declare_parameter<double>("vfov", 1.047);  // 60 deg
    max_view_ = declare_parameter<double>("max_view_range", 6.0);

    // ---- goal publishing (off by default) ----
    publish_goal_ = declare_parameter<bool>("publish_goal", false);
    goal_min_dist_ = declare_parameter<double>("goal_min_dist", 1.0);
    goal_max_dist_ = declare_parameter<double>("goal_max_dist", 8.0);  // keep < EGO free_keep_range
    goal_reach_dist_ = declare_parameter<double>("goal_reach_dist", 0.6);
    goal_timeout_ = declare_parameter<double>("goal_timeout", 15.0);
    visib_weight_ = declare_parameter<double>("visib_weight", 0.02);

    const double period = declare_parameter<double>("period", 0.5);

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

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("frontier/markers", 1);
    centers_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("frontier/centers", 1);
    views_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("frontier/viewpoints", 1);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("frontier/goal", 1);

    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] { process(); });
  }

private:
  // ------------------------------------------------------------------ main cycle
  void process()
  {
    octomap_msgs::msg::Octomap::ConstSharedPtr msg;
    Vec3 pos;
    bool has_odom;
    {
      std::lock_guard<std::mutex> l(mtx_);
      msg = octo_;
      octo_.reset();  // skip stale maps: only process each message once
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

    KeySet cells;
    detectCells(pos, has_odom, cells);

    std::vector<Frontier> raw = cluster(cells);
    std::vector<Frontier> frontiers;
    for (auto & f : raw) splitFrontier(f, frontiers, 0);

    if (has_odom) {
      for (auto & f : frontiers) sampleViewpoints(f);
      // frontiers without any viewpoint are "dormant": still drawn, never used as goals
    }

    publish(frontiers, msg->header);
    if (publish_goal_ && has_odom) selectGoal(frontiers, pos, msg->header);

    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "frontiers: %zu cells: %zu time: %.1f ms", frontiers.size(), cells.size(), ms);
  }

  // ------------------------------------------------------------------ detection
  bool inOutBox(const Vec3 & p) const
  {
    return (p.array() >= out_min_.array()).all() && (p.array() <= out_max_.array()).all();
  }

  // Free leaf voxels with an unknown 6-neighbour. Only the ITERATION is cropped to a box
  // around the drone; the unknown test always uses the full tree, so no fake frontier ring.
  void detectCells(const Vec3 & pos, bool has_odom, KeySet & cells)
  {
    static const int D[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

    auto handleLeaf = [&](const auto & it) {
      if (tree_->isNodeOccupied(*it)) return;  // only free leaves
      const double size = it.getSize(), half = size * 0.5;
      const int n = std::max(1, static_cast<int>(std::lround(size / res_)));
      const octomap::point3d c = it.getCoordinate();
      const double x0 = c.x() - half, y0 = c.y() - half, z0 = c.z() - half;

      // for pruned (large) leaves only the surface voxels can touch unknown space
      for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
          const bool edge_ij = (i == 0 || i == n - 1 || j == 0 || j == n - 1);
          const int step = edge_ij ? 1 : std::max(1, n - 1);
          for (int k = 0; k < n; k += step) {
            const Vec3 p(x0 + (i + 0.5) * res_, y0 + (j + 0.5) * res_, z0 + (k + 0.5) * res_);
            if (p.z() < min_z_ || !inOutBox(p)) continue;
            int unk = 0;
            for (const auto & d : D) {
              const int a = i + d[0], b = j + d[1], e = k + d[2];
              if (a >= 0 && a < n && b >= 0 && b < n && e >= 0 && e < n) continue;  // inside leaf
              if (tree_->search(x0 + (a + 0.5) * res_, y0 + (b + 0.5) * res_, z0 + (e + 0.5) * res_) == nullptr)
                ++unk;
            }
            if (unk < min_unknown_) continue;
            cells.insert(tree_->coordToKey(p.x(), p.y(), p.z()));
          }
        }
    };

    if (has_odom) {
      const octomap::point3d bmin(pos.x() - iterate_range_, pos.y() - iterate_range_, pos.z() - iterate_range_);
      const octomap::point3d bmax(pos.x() + iterate_range_, pos.y() + iterate_range_, pos.z() + iterate_range_);
      for (auto it = tree_->begin_leafs_bbx(bmin, bmax), end = tree_->end_leafs_bbx(); it != end; ++it)
        handleLeaf(it);
    } else {
      for (auto it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it) handleLeaf(it);
    }
  }

  // ------------------------------------------------------------------ clustering
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

  // FUEL splitHorizontally: split along the first principal axis while too wide.
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
    const Eigen::Vector2d pc = es.eigenvectors().col(1);  // largest eigenvalue (ascending order)

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

  // ------------------------------------------------------------------ viewpoints
  // known free + no occupied and no unknown voxel within the clearance box
  bool safeSample(const Vec3 & s)
  {
    if (!inOutBox(s)) return false;
    const int vxy = std::max(1, static_cast<int>(std::ceil(clear_xy_ / res_)));
    const int vz = std::max(1, static_cast<int>(std::ceil(clear_z_ / res_)));
    const int sxy = std::max(1, vxy / 3), sz = std::max(1, vz / 2);
    for (int dx = -vxy; dx <= vxy; dx += sxy)
      for (int dy = -vxy; dy <= vxy; dy += sxy)
        for (int dz = -vz; dz <= vz; dz += sz) {
          auto * n = tree_->search(s.x() + dx * res_, s.y() + dy * res_, s.z() + dz * res_);
          if (!n || tree_->isNodeOccupied(n)) return false;
        }
    return true;
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
      for (auto it = ray.begin(); it != ray.end(); ++it) {
        auto * n = tree_->search(*it);
        if (!n || tree_->isNodeOccupied(n)) {  // unknown or occupied blocks the view
          ok = false;
          break;
        }
      }
      if (ok) ++vis;
    }
    return vis * stride;  // scale back so min_visib_num is in "cells"
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

  // ------------------------------------------------------------------ goal selection
  void selectGoal(const std::vector<Frontier> & fs, const Vec3 & pos, const std_msgs::msg::Header & h)
  {
    if (have_goal_) {
      const bool reached = (pos - goal_).norm() < goal_reach_dist_;
      const bool timeout = (now() - goal_time_).seconds() > goal_timeout_;
      bool still_valid = false;
      for (const auto & f : fs)
        for (const auto & v : f.viewpoints)
          if ((v.pos - goal_).norm() < 1.5) still_valid = true;
      if (!reached && !timeout && still_valid) return;  // keep current goal (no flip-flopping)
    }

    double best_cost = 1e9;
    const Viewpoint * best = nullptr;
    for (const auto & f : fs)
      for (const auto & v : f.viewpoints) {  // sorted by visib: first one in range wins
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

  // ------------------------------------------------------------------ output
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
      pc.position.x = f.avg.x();
      pc.position.y = f.avg.y();
      pc.position.z = f.avg.z();
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

  // ------------------------------------------------------------------ members
  double iterate_range_, cluster_size_xy_, min_z_, res_{0.1};
  int cluster_min_, min_unknown_, down_sample_;
  Vec3 out_min_, out_max_;

  double cand_rmin_, cand_rmax_, cand_dphi_, clear_xy_, clear_z_, hfov_, vfov_, max_view_;
  int cand_rnum_, min_visib_, max_cells_eval_;

  bool publish_goal_;
  double goal_min_dist_, goal_max_dist_, goal_reach_dist_, goal_timeout_, visib_weight_;
  bool have_goal_{false};
  Vec3 goal_{0, 0, 0};
  rclcpp::Time goal_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<octomap::OcTree> tree_;

  std::mutex mtx_;
  octomap_msgs::msg::Octomap::ConstSharedPtr octo_;
  Vec3 pos_{0, 0, 0};
  bool has_odom_{false};

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octo_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr centers_pub_, views_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace plan_env

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plan_env::FrontierNode>());
  rclcpp::shutdown();
  return 0;
}
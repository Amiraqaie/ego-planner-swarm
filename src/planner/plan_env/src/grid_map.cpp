#include "plan_env/grid_map.h"
#include "plan_env/octomap_bounds.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  /* get parameter */
  double x_size, y_size, z_size;
  node_->declare_parameter("grid_map/resolution", -1.0);
  node_->declare_parameter("grid_map/map_size_x", -1.0);
  node_->declare_parameter("grid_map/map_size_y", -1.0);
  node_->declare_parameter("grid_map/map_size_z", -1.0);
  node_->declare_parameter("grid_map/local_update_range_x", -1.0);
  node_->declare_parameter("grid_map/local_update_range_y", -1.0);
  node_->declare_parameter("grid_map/local_update_range_z", -1.0);
  node_->declare_parameter("grid_map/obstacles_inflation", -1.0);
  node_->declare_parameter("grid_map/fx", -1.0);
  node_->declare_parameter("grid_map/fy", -1.0);
  node_->declare_parameter("grid_map/cx", -1.0);
  node_->declare_parameter("grid_map/cy", -1.0);
  node_->declare_parameter("grid_map/use_depth_filter", true);
  node_->declare_parameter("grid_map/depth_filter_tolerance", -1.0);
  node_->declare_parameter("grid_map/depth_filter_maxdist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_mindist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_margin", -1);
  node_->declare_parameter("grid_map/k_depth_scaling_factor", -1.0);
  node_->declare_parameter("grid_map/skip_pixel", -1);
  node_->declare_parameter("grid_map/p_hit", 0.70);
  node_->declare_parameter("grid_map/p_miss", 0.35);
  node_->declare_parameter("grid_map/p_min", 0.12);
  node_->declare_parameter("grid_map/p_max", 0.97);
  node_->declare_parameter("grid_map/p_occ", 0.80);
  node_->declare_parameter("grid_map/min_ray_length", -0.1);
  node_->declare_parameter("grid_map/max_ray_length", -0.1);
  node_->declare_parameter("grid_map/visualization_truncate_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yp", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yn", -0.1);
  node_->declare_parameter("grid_map/show_occ_time", false);
  node_->declare_parameter("grid_map/pose_type", 1);
  node_->declare_parameter("grid_map/frame_id", "map");
  node_->declare_parameter("grid_map/local_map_margin", 1);
  mp_.full_map_updates_ = node_->declare_parameter("grid_map/full_map_updates", false);
  const auto obstacle_source =
      node_->declare_parameter<std::string>("grid_map/obstacle_source", "cloud_depth");
  mp_.obstacles_from_octomap_ = obstacle_source == "octomap";
  mp_.soft_clearance_radius_ = node_->declare_parameter("grid_map/soft_clearance_radius", 1.0);
  mp_.soft_clearance_weight_ = node_->declare_parameter("grid_map/soft_clearance_weight", 2.0);
  if (!std::isfinite(mp_.soft_clearance_radius_) || mp_.soft_clearance_radius_ < 0 ||
      !std::isfinite(mp_.soft_clearance_weight_) || mp_.soft_clearance_weight_ < 0)
    throw std::invalid_argument("Soft clearance radius and weight must be finite and nonnegative");
  const bool use_cloud = obstacle_source == "cloud" || obstacle_source == "cloud_depth";
  const bool use_depth = obstacle_source == "depth" || obstacle_source == "cloud_depth";
  if (!mp_.obstacles_from_octomap_ && !use_cloud && !use_depth)
    throw std::invalid_argument("grid_map/obstacle_source must be octomap, cloud, depth or cloud_depth");
  node_->declare_parameter("grid_map/ground_height", 1.0);
  node_->declare_parameter("grid_map/odom_depth_timeout", 1.0);
  node_->declare_parameter("grid_map/use_free_space_only", false);
  node_->declare_parameter("grid_map/free_around_odom", 0.5);
  node_->declare_parameter("grid_map/free_dilate_angle", 0.01);
  node_->declare_parameter("grid_map/free_dilate_max", 2);
  node_->declare_parameter("grid_map/free_keep_range", 15.0);
  node_->declare_parameter("grid_map/free_publish_range", 6.0);
  node_->declare_parameter("grid_map/free_publish_stride", 1);
  node_->declare_parameter("grid_map/free_astar_stride", 2);
  node_->declare_parameter("grid_map/free_astar_max_nodes", 400000);
  node_->declare_parameter("grid_map/free_astar_max_seg", 3.0);
  node_->declare_parameter("grid_map/free_source", std::string("scan_cloud"));

  node_->get_parameter("grid_map/free_astar_stride", mp_.free_astar_stride_);
  node_->get_parameter("grid_map/free_astar_max_nodes", mp_.free_astar_max_nodes_);
  node_->get_parameter("grid_map/free_astar_max_seg", mp_.free_astar_max_seg_);
  node_->get_parameter("grid_map/resolution", mp_.resolution_);
  node_->get_parameter("grid_map/map_size_x", x_size);
  node_->get_parameter("grid_map/map_size_y", y_size);
  node_->get_parameter("grid_map/map_size_z", z_size);
  node_->get_parameter("grid_map/local_update_range_x", mp_.local_update_range_(0));
  node_->get_parameter("grid_map/local_update_range_y", mp_.local_update_range_(1));
  node_->get_parameter("grid_map/local_update_range_z", mp_.local_update_range_(2));
  node_->get_parameter("grid_map/obstacles_inflation", mp_.obstacles_inflation_);
  node_->get_parameter("grid_map/fx", mp_.fx_);
  node_->get_parameter("grid_map/fy", mp_.fy_);
  node_->get_parameter("grid_map/cx", mp_.cx_);
  node_->get_parameter("grid_map/cy", mp_.cy_);
  node_->get_parameter("grid_map/use_depth_filter", mp_.use_depth_filter_);
  node_->get_parameter("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_);
  node_->get_parameter("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_);
  node_->get_parameter("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_);
  node_->get_parameter("grid_map/depth_filter_margin", mp_.depth_filter_margin_);
  node_->get_parameter("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_);
  node_->get_parameter("grid_map/skip_pixel", mp_.skip_pixel_);
  node_->get_parameter("grid_map/p_hit", mp_.p_hit_);
  node_->get_parameter("grid_map/p_miss", mp_.p_miss_);
  node_->get_parameter("grid_map/p_min", mp_.p_min_);
  node_->get_parameter("grid_map/p_max", mp_.p_max_);
  node_->get_parameter("grid_map/p_occ", mp_.p_occ_);
  node_->get_parameter("grid_map/min_ray_length", mp_.min_ray_length_);
  node_->get_parameter("grid_map/max_ray_length", mp_.max_ray_length_);
  node_->get_parameter("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_);
  node_->get_parameter("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_);
  node_->get_parameter("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_);
  node_->get_parameter("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_);
  node_->get_parameter("grid_map/show_occ_time", mp_.show_occ_time_);
  node_->get_parameter("grid_map/pose_type", mp_.pose_type_);
  node_->get_parameter("grid_map/frame_id", mp_.frame_id_);
  node_->get_parameter("grid_map/local_map_margin", mp_.local_map_margin_);
  node_->get_parameter("grid_map/ground_height", mp_.ground_height_);
  node_->get_parameter("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_);
  node_->get_parameter("grid_map/use_free_space_only", mp_.use_free_space_only_);
  node_->get_parameter("grid_map/free_around_odom", mp_.free_around_odom_);
  node_->get_parameter("grid_map/free_dilate_angle", mp_.free_dilate_angle_);
  node_->get_parameter("grid_map/free_dilate_max", mp_.free_dilate_max_);
  node_->get_parameter("grid_map/free_keep_range", mp_.free_keep_range_);
  node_->get_parameter("grid_map/free_publish_range", mp_.free_publish_range_);
  node_->get_parameter("grid_map/free_publish_stride", mp_.free_publish_stride_);
  std::string free_source;
  node_->get_parameter("grid_map/free_source", free_source);
  mp_.free_from_octomap_ = (free_source == "octomap");
  if (!mp_.free_from_octomap_ && free_source != "scan_cloud")
    RCLCPP_WARN(node_->get_logger(), "free_source '%s' unknown, using scan_cloud", free_source.c_str());
  mp_.free_publish_stride_ = std::max(1, mp_.free_publish_stride_);
  RCLCPP_INFO(node_->get_logger(), "Map input: obstacles=%s, free=%s, updates=%s",
              obstacle_source.c_str(), mp_.use_free_space_only_ ? free_source.c_str() : "disabled",
              usesOctomapBounds() ? "OctoMap bounds" :
              mp_.full_map_updates_ ? "full planner bounds" : "local ranges");

  if (!usesOctomapBounds() && mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  if (usesOctomapBounds())
  {
    // Keep a single unknown voxel until the first snapshot supplies the map bounds.
    allocateGrid(Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(mp_.resolution_),
                 Eigen::Vector3i::Ones());
  }
  else
  {
    const Eigen::Vector3d size(x_size, y_size, z_size);
    allocateGrid(Eigen::Vector3d(-x_size / 2., -y_size / 2., mp_.ground_height_), size,
                 (size.array() * mp_.resolution_inv_).ceil().cast<int>());
  }

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  if (use_depth)
  {
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node_, "grid_map/depth", rclcpp::QoS(50).get_rmw_qos_profile());

    extrinsic_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/vins_estimator/extrinsic", 10,
        std::bind(&GridMap::extrinsicCallback, this, std::placeholders::_1));

    if (mp_.pose_type_ == POSE_STAMPED)
    {
      pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
          node_, "grid_map/pose", rclcpp::QoS(25).get_rmw_qos_profile());

      sync_image_pose_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImagePose>>(
          SyncPolicyImagePose(100), *depth_sub_, *pose_sub_);
      sync_image_pose_->registerCallback(
          std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else if (mp_.pose_type_ == ODOMETRY)
    {
      odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
          node_, "grid_map/odom", rclcpp::QoS(100).get_rmw_qos_profile());

      sync_image_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(
          SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_);
      sync_image_odom_->registerCallback(
          std::bind(&GridMap::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

  if (use_cloud)
  {
    indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "grid_map/cloud", 10, std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));
  }

  indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "grid_map/odom", 10, std::bind(&GridMap::odomCallback, this, std::placeholders::_1));

  if (mp_.obstacles_from_octomap_ || (mp_.use_free_space_only_ && mp_.free_from_octomap_))
  {
    free_octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
        "grid_map/free_octomap", rclcpp::QoS(1),
        std::bind(&GridMap::freeOctomapCallback, this, std::placeholders::_1));
  }
  if (mp_.use_free_space_only_ && !mp_.free_from_octomap_)
  {
    free_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "grid_map/free_cloud", rclcpp::QoS(5),
        std::bind(&GridMap::freeCloudCallback, this, std::placeholders::_1));
  }

  free_trim_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1), std::bind(&GridMap::trimFreeBuffer, this));
  free_space_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "grid_map/free_space", 10);
  free_pub_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&GridMap::publishFreeSpace, this));

  if (use_depth)
  {
    occ_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(0.05),
        std::bind(&GridMap::updateOccupancyCallback, this));
  }

  vis_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.11),
      std::bind(&GridMap::visCallback, this));

  // 发布者
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);
  if (node_->declare_parameter("grid_map/enable_current_floor_cloud", false))
  {
    current_floor_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "grid_map/occupancy_inflate_current_floor", 1);
    drone_cube_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "grid_map/occupancy_inflate_drone_cube", 1);
    node_->declare_parameter("grid_map/ceiling_display_margin", 0.30);
    node_->declare_parameter("grid_map/hide_lower_floors", false);
    node_->declare_parameter("grid_map/nominal_floor_height", 3.0);
    ceiling_height_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
        "grid_map/detected_ceiling_height", rclcpp::QoS(1).transient_local());
  }

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.local_bound_min_.setZero();
  md_.local_bound_max_.setZero();
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

bool GridMap::usesOctomapBounds() const
{
  return mp_.full_map_updates_ && mp_.obstacles_from_octomap_ && mp_.free_from_octomap_ &&
         mp_.use_free_space_only_;
}

void GridMap::allocateGrid(const Eigen::Vector3d &origin, const Eigen::Vector3d &size,
                          const Eigen::Vector3i &cells)
{
  const auto count = static_cast<size_t>(cells.x()) * cells.y() * cells.z();
  std::vector<double> occupancy(count, mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::vector<char> inflated(count, 0), free(count, 0);

  // The OctoMap import does not use the depth raycast buffers.
  if (!usesOctomapBounds())
  {
    md_.count_hit_and_miss_.assign(count, 0);
    md_.count_hit_.assign(count, 0);
    md_.flag_rayend_.assign(count, -1);
    md_.flag_traverse_.assign(count, -1);
  }
  md_.occupancy_buffer_.swap(occupancy);
  md_.occupancy_buffer_inflate_.swap(inflated);
  md_.free_buffer_.swap(free);
  clearance_cost_.clear();
  clearance_obstacles_.clear();
  mp_.map_origin_ = origin;
  mp_.map_size_ = size;
  mp_.map_voxel_num_ = cells;
  mp_.map_min_boundary_ = origin;
  mp_.map_max_boundary_ = origin + size;
  md_.local_bound_min_.setZero();
  md_.local_bound_max_ = cells - Eigen::Vector3i::Ones();
}

bool GridMap::isKnownFreePosition(const Eigen::Vector3d &pos)
{
  if (!pos.allFinite() || !isInMap(pos))
    return false;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  const int address = toAddress(id);
  return md_.free_buffer_[address] == 1 && md_.occupancy_buffer_inflate_[address] == 0;
}

void GridMap::updateSoftClearance()
{
  if (mp_.soft_clearance_radius_ <= 0 || mp_.soft_clearance_weight_ <= 0)
    return;

  const auto &occupancy = md_.occupancy_buffer_;
  std::vector<int> obstacles;
  for (size_t i = 0; i < occupancy.size(); ++i)
    if (occupancy[i] > mp_.min_occupancy_log_)
      obstacles.push_back(static_cast<int>(i));
  if (obstacles == clearance_obstacles_)
    return;
  clearance_obstacles_.swap(obstacles);
  if (clearance_obstacles_.empty())
  {
    clearance_cost_.clear();
    return;
  }

  const double radius = mp_.soft_clearance_radius_;
  const double limit = std::pow(radius * mp_.resolution_inv_ + 0.5, 2);
  clearance_cost_.assign(occupancy.size(), limit);
  for (const int i : clearance_obstacles_)
    clearance_cost_[i] = 0;

  // The lower envelope of parabolas gives squared Euclidean distances in
  // linear time per axis. The radius cap limits the resulting cost field.
  const Eigen::Vector3i dims = mp_.map_voxel_num_;
  std::vector<int> sites(dims.maxCoeff());
  std::vector<double> breaks(dims.maxCoeff() + 1), input(dims.maxCoeff());
  for (int axis = 0; axis < 3; ++axis)
  {
    const int b = (axis + 1) % 3, c = (axis + 2) % 3;
    Eigen::Vector3i index = Eigen::Vector3i::Zero();
    Eigen::Vector3i offset = Eigen::Vector3i::Unit(axis);
    const int stride = toAddress(offset);
    for (index[b] = 0; index[b] < dims[b]; ++index[b])
      for (index[c] = 0; index[c] < dims[c]; ++index[c])
      {
        const int base = toAddress(index);
        for (int q = 0; q < dims[axis]; ++q)
          input[q] = clearance_cost_[base + q * stride];
        int k = 0;
        sites[0] = 0;
        breaks[0] = -std::numeric_limits<double>::infinity();
        for (int q = 1; q < dims[axis]; ++q)
        {
          auto intersection = [&]() {
            const double p = sites[k];
            return ((input[q] - input[sites[k]]) / (q - p) + q + p) * 0.5;
          };
          double x = intersection();
          while (x <= breaks[k])
          {
            --k;
            x = intersection();
          }
          sites[++k] = q;
          breaks[k] = x;
        }
        breaks[k + 1] = std::numeric_limits<double>::infinity();
        k = 0;
        for (int q = 0; q < dims[axis]; ++q)
        {
          while (breaks[k + 1] < q)
            ++k;
          const double d = q - sites[k];
          clearance_cost_[base + q * stride] = std::min(limit, input[sites[k]] + d * d);
        }
      }
  }
  for (float &value : clearance_cost_)
  {
    const double distance = std::max(0.0, (std::sqrt(value) - 0.5) * mp_.resolution_);
    const double proximity = std::max(0.0, 1.0 - distance / radius);
    value = mp_.soft_clearance_weight_ * proximity * proximity;
  }
}

double GridMap::softClearanceCost(const Eigen::Vector3d &pos, Eigen::Vector3d &gradient)
{
  gradient.setZero();
  if (!hasSoftClearance() || !pos.allFinite() || !isInMap(pos))
    return 0.0;
  const Eigen::Array3d coord = (pos - mp_.map_origin_).array() * mp_.resolution_inv_ - 0.5;
  const Eigen::Array3d cell = coord.floor();
  const Eigen::Array3d fraction = coord - cell;
  const Eigen::Vector3i lo = cell.cast<int>().matrix();
  double cost = 0.0;
  for (int corner = 0; corner < 8; ++corner)
  {
    Eigen::Vector3i id = lo;
    Eigen::Array3d weight, derivative;
    for (int axis = 0; axis < 3; ++axis)
    {
      const bool upper = (corner & (1 << axis)) != 0;
      id[axis] = std::clamp(id[axis] + upper, 0, mp_.map_voxel_num_[axis] - 1);
      weight[axis] = upper ? fraction[axis] : 1.0 - fraction[axis];
      derivative[axis] = upper ? mp_.resolution_inv_ : -mp_.resolution_inv_;
    }
    const double value = clearance_cost_[toAddress(id)];
    cost += value * weight.prod();
    for (int axis = 0; axis < 3; ++axis)
      gradient[axis] += value * derivative[axis] * weight[(axis + 1) % 3] * weight[(axis + 2) % 3];
  }
  return cost;
}

float GridMap::segmentCost(const Eigen::Vector3d &a, const Eigen::Vector3d &b, bool known_free,
                           const Eigen::Vector3i *escape_voxel)
{
  const float infinity = std::numeric_limits<float>::infinity();
  if (!a.allFinite() || !b.allFinite() || !isInMap(a) || !isInMap(b))
    return infinity;
  const double len = (b - a).norm();
  const int samples = std::max(1, static_cast<int>(std::ceil(len / (0.5 * mp_.resolution_))));
  double penalty = 0.0;
  for (int i = 0; i <= samples; ++i)
  {
    Eigen::Vector3i id;
    posToIndex(a + (b - a) * (static_cast<double>(i) / samples), id);
    if (!isInMap(id))
      return infinity;
    const int address = toAddress(id);
    if (md_.occupancy_buffer_inflate_[address] != 0 ||
        (known_free && md_.free_buffer_[address] != 1 && !(escape_voxel && id == *escape_voxel)))
      return infinity;
    if (hasSoftClearance())
      penalty += clearance_cost_[address] * (i == 0 || i == samples ? 0.5 : 1.0);
  }
  return len * mp_.resolution_inv_ * (1.0 + penalty / samples);
}

bool GridMap::searchFreePath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                             std::vector<Eigen::Vector3d> &path, bool exact,
                             std::chrono::steady_clock::time_point deadline,
                             bool *search_limit_hit, std::vector<Eigen::Vector3d> *raw_path)
{
  return searchPath(start, goal, path, exact, deadline, search_limit_hit, raw_path, false);
}

bool GridMap::searchLocalPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                             std::vector<Eigen::Vector3d> &path,
                             std::chrono::steady_clock::time_point deadline)
{
  return searchPath(start, goal, path, true, deadline, nullptr, nullptr, true);
}

bool GridMap::searchPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                             std::vector<Eigen::Vector3d> &path, bool exact,
                             std::chrono::steady_clock::time_point deadline,
                             bool *search_limit_hit, std::vector<Eigen::Vector3d> *raw_path, bool local)
{
  path.clear();
  if (raw_path)
    raw_path->clear();
  const bool known_free = !local || useFreeSpaceOnly();
  if (search_limit_hit)
    *search_limit_hit = false;
  if (!start.allFinite() || !goal.allFinite() || !isInMap(start) || !isInMap(goal) ||
      (exact && (known_free ? !isKnownFreePosition(goal) : getInflateOccupancy(goal) != 0)))
    return false;

  const int s = std::max(1, mp_.free_astar_stride_);
  const int ny = mp_.map_voxel_num_(1);
  const int nz = mp_.map_voxel_num_(2);
  const bool soft = hasSoftClearance();
  const bool use_metric_cost = local || soft;
  const float infinity = std::numeric_limits<float>::infinity();

  auto passable = [&](const Eigen::Vector3i &id) {
    if (!isInMap(id))
      return false;
    const int a = toAddress(id);
    return (!known_free || md_.free_buffer_[a] == 1) && md_.occupancy_buffer_inflate_[a] == 0;
  };
  auto toId = [&](int a) {
    return Eigen::Vector3i(a / (ny * nz), (a / nz) % ny, a % nz);
  };

  Eigen::Vector3i si, gi;
  posToIndex(start, si);
  posToIndex(goal, gi);
  // Only the global search may leave an unobserved starting voxel.
  auto edgeCost = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    return segmentCost(a, b, known_free, exact && !local ? &si : nullptr);
  };
  const float direct = exact && (goal - start).norm() <= mp_.free_astar_max_seg_
      ? edgeCost(start, goal) : infinity;
  if (std::isfinite(direct) && (!soft || direct <= (goal - start).norm() * mp_.resolution_inv_ + 1e-5))
  {
    path = {start, goal};
    if (raw_path)
      *raw_path = path;
    return true;
  }
  auto h = [&](const Eigen::Vector3i &id) -> float {
    if (!use_metric_cost)
      return (id - gi).cast<float>().norm();
    Eigen::Vector3d p;
    indexToPos(id, p);
    if (id == si)
      p = start;
    return (p - goal).norm() * mp_.resolution_inv_;
  };

  struct Q { float f; int adr; };
  auto cmp = [](const Q &a, const Q &b) { return a.f > b.f; };
  std::priority_queue<Q, std::vector<Q>, decltype(cmp)> open(cmp);
  std::unordered_map<int, float> g;
  std::unordered_map<int, int> parent;

  const int s_adr = toAddress(si);
  g[s_adr] = 0.f;
  parent[s_adr] = -1;
  open.push({h(si), s_adr});

  int best = s_adr, goal_adr = std::isfinite(direct) ? s_adr : -1, expanded = 0;
  float goal_cost = direct;
  float best_h = h(si);

  while (!open.empty() && expanded < mp_.free_astar_max_nodes_)
  {
    const Q cur = open.top();
    open.pop();
    const Eigen::Vector3i id = toId(cur.adr);
    const float gc = g[cur.adr];
    if (cur.f - h(id) > gc + 1e-3f)
      continue;   // stale queue entry
    if (use_metric_cost && exact && cur.f >= goal_cost)
      break;
    ++expanded;
    if (exact && (local || expanded % 128 == 0) && std::chrono::steady_clock::now() >= deadline)
    {
      if (search_limit_hit)
        *search_limit_hit = true;
      if (goal_adr < 0)
        return false;
      break;
    }

    const float hc = h(id);
    if (hc < best_h) { best_h = hc; best = cur.adr; }
    Eigen::Vector3d current;
    indexToPos(id, current);
    if (cur.adr == s_adr)
      current = start;
    if ((id - gi).cwiseAbs().maxCoeff() <= s)
    {
      const float tail = exact ? edgeCost(current, goal) : 0.0f;
      if (std::isfinite(tail) && gc + tail < goal_cost)
      {
        goal_adr = cur.adr;
        goal_cost = gc + tail;
        if (!use_metric_cost || !exact)
          break;
      }
    }

    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz)
        {
          if (dx == 0 && dy == 0 && dz == 0)
            continue;
          const Eigen::Vector3i nb = id + Eigen::Vector3i(dx, dy, dz) * s;
          if (!passable(nb))
            continue;
          float step_cost = s * std::sqrt(static_cast<float>(dx * dx + dy * dy + dz * dz));
          if (exact || soft)
          {
            Eigen::Vector3d neighbor;
            indexToPos(nb, neighbor);
            const float checked = edgeCost(current, neighbor);
            if (!std::isfinite(checked))
              continue;
            if (use_metric_cost)
              step_cost = checked;
          }
          const int na = toAddress(nb);
          const float ng = gc + step_cost;
          auto it = g.find(na);
          if (it != g.end() && it->second <= ng)
            continue;
          g[na] = ng;
          parent[na] = cur.adr;
          open.push({ng + h(nb), na});
        }
  }

  const bool reached = goal_adr >= 0;
  if (search_limit_hit && !reached && !open.empty() && expanded >= mp_.free_astar_max_nodes_)
    *search_limit_hit = true;
  if (exact && !reached)
    return false;
  std::vector<Eigen::Vector3d> raw;
  for (int a = reached ? goal_adr : best; a != -1; a = parent[a])
  {
    Eigen::Vector3d p;
    indexToPos(toId(a), p);
    raw.push_back(p);
  }
  std::reverse(raw.begin(), raw.end());
  raw.front() = start;
  if (exact && reached)
  {
    if ((raw.back() - goal).norm() > 1e-6)
      raw.push_back(goal);
  }
  else if (reached)
  {
    raw.back() = goal;
  }
  if (raw.size() < 2)
    return false;
  if (raw_path)
    *raw_path = raw;
  if (local)
  {
    path = std::move(raw);
    return reached;
  }

  // Keep a shortcut only if it preserves the clearance benefit of the search.
  std::vector<double> prefix_cost;
  if (soft)
  {
    prefix_cost.assign(raw.size(), 0.0);
    for (size_t i = 1; i < raw.size(); ++i)
      prefix_cost[i] = prefix_cost[i - 1] + edgeCost(raw[i - 1], raw[i]);
  }

  path.push_back(raw.front());
  size_t i = 0;
  while (i + 1 < raw.size())
  {
    size_t j = raw.size() - 1;
    while (j > i + 1)
    {
      if ((raw[j] - raw[i]).norm() <= mp_.free_astar_max_seg_)
      {
        const float shortcut = edgeCost(raw[i], raw[j]);
        if (std::isfinite(shortcut) && (!soft || shortcut <= prefix_cost[j] - prefix_cost[i] + 1e-4))
          break;
      }
      --j;
    }
    path.push_back(raw[j]);
    i = j;
  }
  return reached;
}

void GridMap::resetBuffer()
{
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  rclcpp::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!mp_.full_map_updates_ && !in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  if (mp_.full_map_updates_)
  {
    // Keep accumulated depth observations and rebuild inflation over the whole map.
    md_.local_bound_min_.setZero();
    md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  }

  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            if (isInMap(inf_pt))
              md_.occupancy_buffer_inflate_[toAddress(inf_pt)] = 1;
          }
        }
      }

  applyVirtualCeiling();
}

void GridMap::applyVirtualCeiling()
{
  if (mp_.virtual_ceil_height_ <= -0.5)
    return;
  const int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
  if (ceil_id < 0 || ceil_id >= mp_.map_voxel_num_(2))
    return;
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      md_.occupancy_buffer_inflate_[toAddress(Eigen::Vector3i(x, y, ceil_id))] = 1;
}

void GridMap::visCallback()
{
  publishMapInflate(true);
  publishMap();
}

void GridMap::publishFreeSpace()
{
  if (!mp_.use_free_space_only_ || free_space_pub_->get_subscription_count() <= 0)
    return;

  Eigen::Vector3i lo(0, 0, 0);
  Eigen::Vector3i hi = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  if (!mp_.full_map_updates_ && mp_.free_publish_range_ > 0.0 && md_.has_odom_)
  {
    const Eigen::Vector3d r = Eigen::Vector3d::Constant(mp_.free_publish_range_);
    posToIndex(md_.camera_pos_ - r, lo);
    posToIndex(md_.camera_pos_ + r, hi);
    boundIndex(lo);
    boundIndex(hi);
  }

  const int st = mp_.full_map_updates_ ? 1 : mp_.free_publish_stride_;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;
  Eigen::Vector3d pos;

  for (int x = lo(0); x <= hi(0); x += st)
    for (int y = lo(1); y <= hi(1); y += st)
      for (int z = lo(2); z <= hi(2); z += st)
      {
        const int a = toAddress(x, y, z);
        // known free and not an (inflated) obstacle: same rule as isKnownFree()
        if (md_.free_buffer_[a] != 1 || md_.occupancy_buffer_inflate_[a] != 0)
          continue;

        indexToPos(Eigen::Vector3i(x, y, z), pos);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = node_->now();
  free_space_pub_->publish(msg);
}

void GridMap::updateOccupancyCallback()
{
  if (md_.last_occ_update_time_.seconds() < 1.0)
    md_.last_occ_update_time_ = node_->now();

  if (!md_.occ_need_update_)
  {
    if (md_.flag_use_depth_fusion &&
        (node_->now() - md_.last_occ_update_time_).seconds() > mp_.odom_depth_timeout_)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "odom or depth lost! now=%f, last_occ_update_time=%f, odom_depth_timeout=%f",
                   node_->now().seconds(),
                   md_.last_occ_update_time_.seconds(),
                   mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }
  md_.last_occ_update_time_ = node_->now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                                const geometry_msgs::msg::PoseStamped::ConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

void GridMap::freeCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!mp_.use_free_space_only_)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  Eigen::Vector3i id;
  for (const auto &p : cloud.points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    Eigen::Vector3d pos(p.x, p.y, p.z);
    if (!isInMap(pos))
      continue;
    posToIndex(pos, id);

    // widen far points to close the gaps between diverging rays
    const double dist = md_.has_odom_ ? (pos - md_.camera_pos_).norm() : 0.0;
    const int h = std::min(
        mp_.free_dilate_max_,
        static_cast<int>(0.5 * dist * mp_.free_dilate_angle_ * mp_.resolution_inv_));
    for (int dx = -h; dx <= h; ++dx)
      for (int dy = -h; dy <= h; ++dy)
        for (int dz = -h; dz <= h; ++dz)
        {
          Eigen::Vector3i n(id(0) + dx, id(1) + dy, id(2) + dz);
          if (!isInMap(n))
            continue;
          md_.free_buffer_[toAddress(n)] = 1;
        }
  }

  // lidar min range leaves unknown voxels around the body: force them free
  forceFreeAroundOdom();
}

bool GridMap::isUnseen(const Eigen::Vector3d &pos)
{
  if (!isInMap(pos))
    return true;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  const int a = toAddress(id);
  return md_.free_buffer_[a] == 0 && md_.occupancy_buffer_inflate_[a] == 0;
}

void GridMap::forceFreeAroundOdom()
{
  // lidar min range leaves unknown voxels around the body: force them free
  if (!md_.has_odom_ || !md_.camera_pos_.allFinite() || mp_.free_around_odom_ <= 0.0)
    return;
  const Eigen::Vector3d radius = Eigen::Vector3d::Constant(mp_.free_around_odom_);
  const Eigen::Vector3d lo = (md_.camera_pos_ - radius).cwiseMax(mp_.map_min_boundary_);
  const Eigen::Vector3d hi = (md_.camera_pos_ + radius).cwiseMin(mp_.map_max_boundary_);
  if ((lo.array() >= hi.array()).any())
    return;

  Eigen::Vector3i lo_id, hi_id;
  posToIndex(lo, lo_id);
  posToIndex(hi, hi_id);
  boundIndex(lo_id);
  boundIndex(hi_id);
  for (int x = lo_id(0); x <= hi_id(0); ++x)
    for (int y = lo_id(1); y <= hi_id(1); ++y)
      for (int z = lo_id(2); z <= hi_id(2); ++z)
        md_.free_buffer_[toAddress(x, y, z)] = 1;
}

void GridMap::freeOctomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr msg)
{
  const bool import_free = mp_.use_free_space_only_ && mp_.free_from_octomap_;
  const bool import_obstacles = mp_.obstacles_from_octomap_;
  if ((!import_free && !import_obstacles) || msg->header.frame_id != mp_.frame_id_)
    return;

  std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::fullMsgToMap(*msg));
  auto *tree = dynamic_cast<octomap::OcTree *>(abstract.get());
  if (!tree)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "OctoMap input must contain a full OcTree");
    return;
  }

  if (usesOctomapBounds())
  {
    const auto bounds = plan_env::octomapBounds(*tree, mp_.resolution_, mp_.obstacles_inflation_);
    bool usable = bounds.has_value();
    if (usable && (bounds->origin != mp_.map_origin_ || bounds->cells != mp_.map_voxel_num_))
    {
      try
      {
        allocateGrid(bounds->origin, bounds->cells.cast<double>() * mp_.resolution_, bounds->cells);
      }
      catch (const std::bad_alloc &)
      {
        usable = false;
      }
    }
    if (!usable)
    {
      // Never keep a traversable copy of a stale snapshot when the input cannot be represented.
      std::fill(md_.free_buffer_.begin(), md_.free_buffer_.end(), 0);
      std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(),
                mp_.clamp_min_log_ - mp_.unknown_flag_);
      resetBuffer();
      clearance_cost_.clear();
      clearance_obstacles_.clear();
      md_.has_cloud_ = false;
      if (tree->size() != 0)
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                              "OctoMap bounds exceed the planner's grid capacity");
      return;
    }
  }

  // A full OctoMap is a snapshot. Clear old cells so changes and deletions are visible.
  if (import_free)
    std::fill(md_.free_buffer_.begin(), md_.free_buffer_.end(), 0);
  if (import_obstacles)
  {
    std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(),
              mp_.clamp_min_log_ - mp_.unknown_flag_);
    resetBuffer();
    md_.has_cloud_ = true;
  }

  constexpr double index_epsilon = 1e-3;
  const int inflate_xy = std::max(0, static_cast<int>(std::ceil(mp_.obstacles_inflation_ * mp_.resolution_inv_)));
  const Eigen::Vector3i inflation(inflate_xy, inflate_xy, inflate_xy > 0 ? 1 : 0);
  auto fillBox = [&](auto &buffer, const Eigen::Vector3i &lo, const Eigen::Vector3i &hi, auto value) {
    for (int x = lo.x(); x <= hi.x(); ++x)
      for (int y = lo.y(); y <= hi.y(); ++y)
        std::fill_n(buffer.begin() + toAddress(Eigen::Vector3i(x, y, lo.z())),
                    hi.z() - lo.z() + 1, value);
  };
  auto importLeaf = [&](const auto &it) {
    const bool occupied = tree->isNodeOccupied(*it);
    if (!import_obstacles && (occupied || !import_free))
      return;
    const auto center = it.getCoordinate();
    const double half = it.getSize() * 0.5;
    Eigen::Vector3i lo, hi;
    for (int axis = 0; axis < 3; ++axis)
    {
      lo(axis) = static_cast<int>(std::floor(
          (center(axis) - half - mp_.map_origin_(axis)) * mp_.resolution_inv_ + index_epsilon));
      hi(axis) = std::max(lo(axis), static_cast<int>(std::ceil(
          (center(axis) + half - mp_.map_origin_(axis)) * mp_.resolution_inv_ - index_epsilon)) - 1);
      if (hi(axis) < 0 || lo(axis) >= mp_.map_voxel_num_(axis))
        return;
    }
    boundIndex(lo);
    boundIndex(hi);
    if (import_obstacles)
    {
      fillBox(md_.occupancy_buffer_, lo, hi, occupied ? mp_.clamp_max_log_ : mp_.clamp_min_log_);
      if (occupied)
      {
        Eigen::Vector3i inflated_lo = lo - inflation;
        Eigen::Vector3i inflated_hi = hi + inflation;
        boundIndex(inflated_lo);
        boundIndex(inflated_hi);
        fillBox(md_.occupancy_buffer_inflate_, inflated_lo, inflated_hi, 1);
      }
    }
    if (!occupied && import_free)
      fillBox(md_.free_buffer_, lo, hi, 1);
  };

  if (!mp_.full_map_updates_ && mp_.free_keep_range_ > 0.0 && md_.has_odom_)
  {
    const auto d = Eigen::Vector3d::Constant(mp_.free_keep_range_);
    const Eigen::Vector3d lo = md_.camera_pos_ - d, hi = md_.camera_pos_ + d;
    const octomap::point3d min(lo.x(), lo.y(), lo.z()), max(hi.x(), hi.y(), hi.z());
    for (auto it = tree->begin_leafs_bbx(min, max), end = tree->end_leafs_bbx(); it != end; ++it)
      importLeaf(it);
  }
  else
  {
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
      importLeaf(it);
  }
  if (import_obstacles)
  {
    applyVirtualCeiling();
    updateSoftClearance();
  }
  if (import_free)
    forceFreeAroundOdom();
}

void GridMap::trimFreeBuffer()
{
  if (mp_.full_map_updates_ || mp_.free_from_octomap_ || !mp_.use_free_space_only_ ||
      mp_.free_keep_range_ <= 0.0 || !md_.has_odom_)
    return;

  const Eigen::Vector3d r = Eigen::Vector3d::Constant(mp_.free_keep_range_);
  Eigen::Vector3i lo, hi;
  posToIndex(md_.camera_pos_ - r, lo);
  posToIndex(md_.camera_pos_ + r, hi);
  boundIndex(lo);
  boundIndex(hi);

  const int nx = mp_.map_voxel_num_(0);
  const int ny = mp_.map_voxel_num_(1);
  const int nz = mp_.map_voxel_num_(2);

  for (int x = 0; x < nx; ++x)
  {
    char *xs = md_.free_buffer_.data() + static_cast<size_t>(x) * ny * nz;
    if (x < lo(0) || x > hi(0))
    {
      std::fill(xs, xs + static_cast<size_t>(ny) * nz, 0);
      continue;
    }
    for (int y = 0; y < ny; ++y)
    {
      char *ys = xs + static_cast<size_t>(y) * nz;
      if (y < lo(1) || y > hi(1))
      {
        std::fill(ys, ys + nz, 0);
        continue;
      }
      std::fill(ys, ys + lo(2), 0);
      std::fill(ys + hi(2) + 1, ys + nz, 0);
    }
  }
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &img)
{

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!mp_.full_map_updates_ && !md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (!mp_.full_map_updates_ && latest_cloud.empty())
    return;

  if (mp_.full_map_updates_)
    resetBuffer();
  else
  {
    if (!md_.camera_pos_.allFinite())
      return;
    resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                md_.camera_pos_ + mp_.local_update_range_);
  }

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    if (!p3d.allFinite())
      continue;
    Eigen::Vector3i inf_pt;

    if (mp_.full_map_updates_ ||
        ((p3d - md_.camera_pos_).cwiseAbs().array() < mp_.local_update_range_.array()).all())
    {

      /* inflate the point */
      // 点云膨胀
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt))
              continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  if (!mp_.full_map_updates_)
  {
    min_x = min(min_x, md_.camera_pos_(0));
    min_y = min(min_y, md_.camera_pos_(1));
    min_z = min(min_z, md_.camera_pos_(2));

    max_x = max(max_x, md_.camera_pos_(0));
    max_y = max(max_y, md_.camera_pos_(1));
    max_z = max(max_z, md_.camera_pos_(2));

    max_z = max(max_z, mp_.ground_height_);

    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

    // 更新局部地图边界
    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);
  }
  applyVirtualCeiling();
}

void GridMap::publishMap()
{

  if (map_pub_->get_subscription_count() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (!usesOctomapBounds() && pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_->publish(cloud_msg);
}

std::optional<double> GridMap::findDisplaySlab(const Eigen::Vector3d &position, bool ceiling)
{
  const int direction = ceiling ? 1 : -1;
  Eigen::Vector3i drone;
  posToIndex(position, drone);
  const int spacing = std::max(1, static_cast<int>(std::round(0.35 / mp_.resolution_)));
  // A downward probe starts just below the drone so it can see a thin
  // slab even when the drone is close to the floor.
  const int offset = ceiling ? std::max(1, static_cast<int>(std::ceil(0.25 / mp_.resolution_))) : 1;
  const int first = drone.z() + direction * offset;
  const int range = static_cast<int>(4.0 / mp_.resolution_);
  std::vector<int> hits;
  for (int dx = -2; dx <= 2; ++dx)
    for (int dy = -2; dy <= 2; ++dy)
    {
      Eigen::Vector3i cell(drone.x() + dx * spacing, drone.y() + dy * spacing, first);
      for (; isInMap(cell) && std::abs(cell.z() - drone.z()) <= range; cell.z() += direction)
        if (md_.occupancy_buffer_[toAddress(cell)] > mp_.min_occupancy_log_)
        {
          hits.push_back(direction * cell.z());
          break;
        }
    }
  std::sort(hits.begin(), hits.end());
  const int band = std::max(1, static_cast<int>(std::round(0.15 / mp_.resolution_)));
  for (auto low = hits.begin(), high = hits.begin(); low != hits.end(); ++low)
  {
    while (high != hits.end() && *high - *low <= band)
      ++high;
    if (high - low >= 9)
      return mp_.map_origin_.z() + (direction * *low + (ceiling ? 0 : 1)) * mp_.resolution_;
  }
  return std::nullopt;
}

void GridMap::updateDisplayFloor()
{
  const auto now = std::chrono::steady_clock::now();
  if (now - last_ceiling_check_ < std::chrono::seconds(1))
    return;
  last_ceiling_check_ = now;

  const auto &position = md_.camera_pos_;
  if (mp_.obstacles_from_octomap_ && md_.has_cloud_ && md_.has_odom_ &&
      position.allFinite() && isInMap(position))
  {
    constexpr double slab_tolerance = 0.2;
    constexpr double min_floor_change = 0.6;
    auto confirmed = [&](std::optional<double> found, std::optional<double> &pending, int &count) {
      if (!found)
      {
        pending.reset();
        count = 0;
        return false;
      }
      count = pending && std::abs(*pending - *found) <= slab_tolerance ? count + 1 : 1;
      pending = found;
      if (count < 3)
        return false;
      pending.reset();
      count = 0;
      return true;
    };

    const double nominal = node_->get_parameter("grid_map/nominal_floor_height").as_double();
    const double room_height = confirmed_room_height_.value_or(
        std::isfinite(nominal) && nominal >= 0.8 ? nominal : 3.0);
    const auto observed_floor = findDisplaySlab(position, false);
    auto floor = observed_floor;
    if (floor && !floor_height_ && ceiling_height_ &&
        position.z() >= *ceiling_height_ + slab_tolerance &&
        *floor < *ceiling_height_ - slab_tolerance)
      floor.reset(); // A gap can expose the previous floor through the stairs.
    if (floor && floor_height_)
    {
      const double difference = *floor - *floor_height_;
      const bool above_current = difference >= min_floor_change &&
          (!ceiling_height_ || position.z() > *ceiling_height_ - slab_tolerance);
      const bool below_current = difference <= -min_floor_change &&
          position.z() < *floor_height_ + slab_tolerance;
      if (!above_current && !below_current)
        floor.reset();
    }

    if (confirmed(floor, pending_floor_, floor_confirmations_))
    {
      const bool changed_floor = floor_height_.has_value() ||
          (ceiling_height_ && *floor >= *ceiling_height_ - slab_tolerance);
      floor_height_ = floor;
      if (changed_floor)
      {
        // Transfer the last measured floor-to-ceiling distance to the new
        // floor. It stays available even when the OctoMap has a ceiling gap.
        ceiling_inferred_ = true;
        ceiling_height_ = *floor_height_ + room_height;
        pending_ceiling_.reset();
        ceiling_confirmations_ = 0;
      }
    }

    auto ceiling = findDisplaySlab(position, true);
    const bool on_active_floor = floor_height_ && observed_floor &&
        std::abs(*floor_height_ - *observed_floor) <= slab_tolerance;
    const bool can_correct_inferred = ceiling_inferred_ && on_active_floor;
    const bool above_active = ceiling_height_ && !can_correct_inferred &&
        position.z() >= *ceiling_height_ + slab_tolerance;
    const double active_floor = floor_height_.value_or(
        ceiling_height_ ? *ceiling_height_ - room_height : -std::numeric_limits<double>::infinity());
    const bool below_active = position.z() < active_floor - slab_tolerance;
    // The floor below a stair landing may be unobserved. A stable ceiling
    // at another level can confirm the transition if the drone has crossed
    // the active band's boundary and is within the candidate room.
    const bool switch_by_ceiling = ceiling && ceiling_height_ &&
        position.z() >= *ceiling - room_height - slab_tolerance &&
        ((above_active && *ceiling >= *ceiling_height_ + min_floor_change) ||
         (below_active && *ceiling <= *ceiling_height_ - min_floor_change));
    if (!switch_by_ceiling && (above_active || below_active ||
        (!can_correct_inferred && ceiling && floor_height_ && confirmed_room_height_ &&
         *ceiling - *floor_height_ > *confirmed_room_height_ + min_floor_change)))
      ceiling.reset();

    if (confirmed(ceiling, pending_ceiling_, ceiling_confirmations_))
    {
      if (switch_by_ceiling)
      {
        floor_height_.reset();
        pending_floor_.reset();
        floor_confirmations_ = 0;
      }
      ceiling_height_ = ceiling;
      ceiling_inferred_ = false;
      if (floor_height_ && *ceiling_height_ - *floor_height_ >= 0.8)
        confirmed_room_height_ = *ceiling_height_ - *floor_height_;
    }
  }

  std_msgs::msg::Float64 height;
  height.data = ceiling_height_.value_or(std::numeric_limits<double>::quiet_NaN());
  ceiling_height_pub_->publish(height);
}

void GridMap::publishMapInflate(bool all_info)
{

  const bool publish_full = map_inf_pub_->get_subscription_count() > 0;
  const bool publish_floor = current_floor_pub_ && current_floor_pub_->get_subscription_count() > 0;
  const bool hide_lower = publish_floor && node_->get_parameter("grid_map/hide_lower_floors").as_bool();
  if (ceiling_height_pub_ && (publish_floor || ceiling_height_pub_->get_subscription_count() > 0))
    updateDisplayFloor();

  auto publish = [&](auto &points, const auto &publisher) {
    points.width = points.points.size();
    points.height = 1;
    points.is_dense = true;
    points.header.frame_id = mp_.frame_id_;
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(points, message);
    publisher->publish(message);
  };

  if (drone_cube_pub_ && drone_cube_pub_->get_subscription_count() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ> cube;
    const auto &center = md_.camera_pos_;
    if (md_.has_odom_ && center.allFinite() && isInMap(center))
    {
      const Eigen::Vector3d half = Eigen::Vector3d::Ones();
      Eigen::Vector3i first, last;
      posToIndex(center - half, first);
      posToIndex(center + half, last);
      boundIndex(first);
      boundIndex(last);
      for (int x = first.x(); x <= last.x(); ++x)
        for (int y = first.y(); y <= last.y(); ++y)
          for (int z = first.z(); z <= last.z(); ++z)
          {
            if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
              continue;
            Eigen::Vector3d point;
            indexToPos(Eigen::Vector3i(x, y, z), point);
            if ((point - center).cwiseAbs().maxCoeff() <= 1.0)
              cube.push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
          }
    }
    publish(cube, drone_cube_pub_);
  }

  if (!publish_full && !publish_floor)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud, floor_cloud;
  double cutoff = -std::numeric_limits<double>::infinity();
  if (publish_floor && ceiling_height_)
  {
    // Allow for uneven mapped surfaces as well as vertical voxel inflation.
    const double margin = std::max(mp_.resolution_,
        node_->get_parameter("grid_map/ceiling_display_margin").as_double());
    cutoff = *ceiling_height_ - margin;
  }

  double lower_cutoff = -std::numeric_limits<double>::infinity();
  if (hide_lower)
  {
    if (floor_height_)
      lower_cutoff = *floor_height_ - mp_.resolution_;
    else if (ceiling_height_)
    {
      // After a restart the ceiling can be confirmed before the floor.
      // Use a provisional band instead of showing every lower level.
      const double nominal = node_->get_parameter("grid_map/nominal_floor_height").as_double();
      const double room_height = confirmed_room_height_.value_or(
          std::isfinite(nominal) && nominal >= 0.8 ? nominal : 3.0);
      lower_cutoff = std::min(*ceiling_height_ - room_height - mp_.resolution_,
                             md_.camera_pos_.z() - mp_.resolution_);
    }
  }

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);
  if (!publish_full && !ceiling_height_)
    max_cut.z() = min_cut.z() - 1;
  if (!publish_full && std::isfinite(lower_cutoff))
    min_cut.z() = std::max(min_cut.z(), static_cast<int>(
        std::floor((lower_cutoff - mp_.map_origin_.z()) * mp_.resolution_inv_)));
  if (!publish_full && std::isfinite(cutoff))
    max_cut.z() = std::min(max_cut.z(), static_cast<int>(
        std::floor((cutoff - mp_.map_origin_.z()) * mp_.resolution_inv_)));

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (!usesOctomapBounds() && pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        if (publish_full)
          cloud.push_back(pt);
        if (publish_floor && pos.z() >= lower_cutoff && pos.z() < cutoff)
          floor_cloud.push_back(pt);
      }

  if (publish_full)
    publish(cloud, map_inf_pub_);
  if (publish_floor)
    publish(floor_cloud, current_floor_pub_);
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::msg::Odometry::ConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                                const nav_msgs::msg::Odometry::ConstPtr &odom)
{
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}

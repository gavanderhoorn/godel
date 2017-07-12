#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <godel_msgs/PathPlanning.h>
#include <mesh_importer/mesh_importer.h>
#include <path_planning_plugins/openveronoi_plugins.h>
#include <pluginlib/class_list_macros.h>
#include <profilometer/profilometer_scan.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>

// Together these constants define a 5cm approach and departure path for the laser scans
const static int SCAN_APPROACH_STEP_COUNT = 5;
const static double SCAN_APPROACH_STEP_DISTANCE = 0.01; // 1cm

const static std::string PARAM_BASE = "/process_planning_params/";
const static std::string SCAN_PARAM_BASE = "scan_params/";
const static std::string BLEND_PARAM_BASE = "blend_params/";

//const static std::string SPINDLE_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "spindle_speed";

//const static std::string APPROACH_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "approach_speed";
//const static std::string BLENDING_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "blending_speed";
//const static std::string RETRACT_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "retract_speed";
//const static std::string TRAVERSE_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "traverse_speed";
//const static std::string Z_ADJUST_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "z_adjust";

//const static std::string TOOL_RADIUS_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "tool_radius";
//const static std::string TOOL_OVERLAP_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "overlap";
const static std::string DISCRETIZATION_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "discretization";

const static std::string APPROACH_DISTANCE_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "approach_distance";
//const static std::string QUALITY_METRIC_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "quality_metric";
//const static std::string WINDOW_WIDTH_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "window_width";
//const static std::string MIN_QA_VALUE_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "min_qa_value";
//const static std::string MAX_QA_VALUE_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "max_qa_value";

const static std::string SCAN_OVERLAP_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "overlap";
const static std::string SCAN_WIDTH_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "scan_width";

namespace path_planning_plugins
{
// Scan Planner

template<typename T>
static void loadOrThrow(ros::NodeHandle& nh, const std::string& key, T& value)
{
  if (!nh.getParam(key, value))
  {
    throw std::runtime_error("Unable to load parameter: " + nh.resolveName(key));
  }
}

void openveronoi::ScanPlanner::init(pcl::PolygonMesh mesh)
{
  mesh_ = mesh;
}

bool openveronoi::ScanPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path)
{
  using godel_process_path::PolygonBoundaryCollection;
  using godel_process_path::PolygonBoundary;

  path.clear();

  std::unique_ptr<mesh_importer::MeshImporter> mesh_importer_ptr(new mesh_importer::MeshImporter(false));

  ros::NodeHandle nh;
  godel_msgs::PathPlanningParameters params;
  try
  {
    loadOrThrow(nh, DISCRETIZATION_PARAM, params.discretization);
    params.margin = 0.0;
    loadOrThrow(nh, SCAN_OVERLAP_PARAM, params.overlap);
    loadOrThrow(nh, APPROACH_DISTANCE_PARAM, params.traverse_height);
    loadOrThrow(nh, SCAN_WIDTH_PARAM, params.scan_width);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR_STREAM("Unable to populate path planning parameters" << e.what());
    return false;
  }

  // 0 - Calculate boundaries for a surface
  if (mesh_importer_ptr->calculateSimpleBoundary(mesh_))
  {
    // 1 - Read & filter boundaries that are ill-formed or too small
    PolygonBoundaryCollection filtered_boundaries = filterPolygonBoundaries(mesh_importer_ptr->getBoundaries());

    // 2 - Read boundary pose
    geometry_msgs::Pose boundary_pose;
    mesh_importer_ptr->getPose(boundary_pose);

    // 3 - Skip if boundaries are empty
    if (filtered_boundaries.empty())
      return false;

    geometry_msgs::PoseArray scan_poses;

    // 4 - Generate scan polygon boundary
    PolygonBoundary scan_boundary = scan::generateProfilometerScanPath(filtered_boundaries.front(), params);

    // 5 - Get boundary pose eigen
    Eigen::Affine3d boundary_pose_eigen;
    tf::poseMsgToEigen(boundary_pose, boundary_pose_eigen);

    // 6 - Transform points to world frame and generate pose
    std::vector<geometry_msgs::Point> points;

    for(const auto& pt : scan_boundary)
    {
      geometry_msgs::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.0;

      points.push_back(p);
    }

    std::transform(points.begin(), points.end(), std::back_inserter(scan_poses.poses),
                   [boundary_pose_eigen] (const geometry_msgs::Point& point) {
      geometry_msgs::Pose pose;
      Eigen::Affine3d r = boundary_pose_eigen * Eigen::Translation3d(point.x, point.y, point.z);
      tf::poseEigenToMsg(r, pose);
      return pose;
    });

    // 8 - return result
    path.push_back(scan_poses);
    return true;
  }
  else
    ROS_WARN_STREAM("Could not calculate boundary for mesh");
  return false;
}
} // end namespace path_planning_plugins

PLUGINLIB_EXPORT_CLASS(path_planning_plugins::openveronoi::ScanPlanner, path_planning_plugins_base::PathPlanningBase)

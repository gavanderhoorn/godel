#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <godel_msgs/PathPlanning.h>
#include <mesh_importer/mesh_importer.h>
#include <path_planning_plugins/openveronoi_plugins.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>

const static std::string PATH_GENERATION_SERVICE = "process_path_generator";

const static std::string PARAM_BASE = "/process_planning_params/";
const static std::string SCAN_PARAM_BASE = "scan_params/";
const static std::string BLEND_PARAM_BASE = "blend_params/";

const static std::string SPINDLE_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "spindle_speed";

const static std::string APPROACH_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "approach_speed";
const static std::string BLENDING_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "blending_speed";
const static std::string RETRACT_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "retract_speed";
const static std::string TRAVERSE_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "traverse_speed";
const static std::string Z_ADJUST_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "z_adjust";
const static std::string TRAVERSE_HEIGHT_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "traverse_height";

const static std::string TOOL_RADIUS_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "tool_radius";
const static std::string TOOL_OVERLAP_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "overlap";
const static std::string DISCRETIZATION_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "discretization";

namespace path_planning_plugins
{
typedef  godel_msgs::PathPlanningParameters PlanningParams;

template<typename T>
static void loadOrThrow(ros::NodeHandle& nh, const std::string& key, T& value)
{
  if (!nh.getParam(key, value))
  {
    throw std::runtime_error("Unable to load parameter: " + nh.resolveName(key));
  }
}

void openveronoi::BlendPlanner::init(pcl::PolygonMesh mesh)
{
  mesh_ = mesh;
}

bool openveronoi::BlendPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path)
{
  using godel_process_path::PolygonBoundaryCollection;
  using godel_process_path::PolygonBoundary;

  path.clear();

  std::unique_ptr<mesh_importer::MeshImporter> mesh_importer_ptr(new mesh_importer::MeshImporter(false));
  ros::NodeHandle nh;
  ros::ServiceClient process_path_client = nh.serviceClient<godel_msgs::PathPlanning>(PATH_GENERATION_SERVICE);
  godel_msgs::PathPlanningParameters params;



  try
  {
    loadOrThrow(nh, DISCRETIZATION_PARAM, params.discretization);
    params.margin = 0;
    loadOrThrow(nh, TOOL_OVERLAP_PARAM, params.overlap);
    loadOrThrow(nh, SAFE_TRAVERSE_HEIGHT, params.traverse_height);
    loadOrThrow(nh, TOOL_RADIUS_PARAM, params.tool_radius);
//    loadOrThrow(nh, SCAN_WIDTH, params.scan_width);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR_STREAM("Unable to populate path planning parameters" << e.what());
    return false;
  }


  // Calculate boundaries for a surface
  if (mesh_importer_ptr->calculateSimpleBoundary(mesh_))
  {
    // Read & filter boundaries that are ill-formed or too small
    PolygonBoundaryCollection filtered_boundaries = filterPolygonBoundaries(mesh_importer_ptr->getBoundaries());

    // Read pose
    geometry_msgs::Pose boundary_pose;
    mesh_importer_ptr->getPose(boundary_pose);

    // Send request to blend path generation service
    godel_msgs::PathPlanning srv;
    srv.request.params = params;
    godel_process_path::utils::translations::godelToGeometryMsgs(srv.request.surface.boundaries, filtered_boundaries);
    tf::poseTFToMsg(tf::Transform::getIdentity(), srv.request.surface.pose);

    if (!process_path_client.call(srv))
    {
      ROS_ERROR_STREAM("Process path cilent failed");
      return false;
    }

    // blend process path calculations suceeded. Save data into results.
    geometry_msgs::PoseArray blend_poses;
    geometry_msgs::PoseArray path_local = srv.response.poses;
    geometry_msgs::Pose p;
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 1.0;

    // Transform points to world frame and generate pose
    Eigen::Affine3d boundary_pose_eigen;
    Eigen::Affine3d eigen_p;
    Eigen::Affine3d result;

    tf::poseMsgToEigen(boundary_pose, boundary_pose_eigen);

    for (const auto& pose : path_local.poses)
    {
      p.position.x = pose.position.x;
      p.position.y = pose.position.y;
      p.position.z = pose.position.z;

      tf::poseMsgToEigen(p, eigen_p);
      result = boundary_pose_eigen*eigen_p;
      tf::poseEigenToMsg(result, p);
      p.orientation = boundary_pose.orientation;
      blend_poses.poses.push_back(p);
    }

    path.push_back(blend_poses);
    return true;
  }
  else
    ROS_WARN_STREAM("Could not calculate boundary for mesh");

  return false;

}
} // end namespace

PLUGINLIB_EXPORT_CLASS(path_planning_plugins::openveronoi::BlendPlanner, path_planning_plugins_base::PathPlanningBase)

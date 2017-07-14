#include <godel_process_execution/abb_blend_process_service.h>

#include <industrial_robot_simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <fstream>

#include "process_utils.h"
#include "rapid_generator/rapid_emitter.h"
#include "abb_file_suite/ExecuteProgram.h"
#include <godel_utils/ensenso_guard.h>

// hack
#include "sensor_msgs/JointState.h"
#include "ros/topic.h"

const static double DEFAULT_JOINT_TOPIC_WAIT_TIME = 5.0; // seconds
const static double DEFAULT_TRAJECTORY_BUFFER_TIME = 5.0; // seconds
const static std::string JOINT_TOPIC_NAME = "/joint_states";

const static std::string THIS_SERVICE_NAME = "blend_process_execution";
const static std::string EXECUTION_SERVICE_NAME = "execute_program";
const static std::string SIMULATION_SERVICE_NAME = "simulate_path";
const static std::string PROCESS_EXE_ACTION_SERVER_NAME = "blend_process_execution_as";

static inline bool compare(const std::vector<double>& a, const std::vector<double>& b,
                           double eps = 0.01)
{
  if (a.size() != b.size())
  {
    ROS_ERROR_STREAM("Can't compare joint vectors of unequal length (" << a.size() << " vs "
                                                                       << b.size() << ")");
    return false;
  }

  double diff = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    diff += std::abs(a[i] - b[i]);
  }

  return diff < eps;
}

static bool waitForExecution(const std::vector<double>& end_goal, const ros::Duration& wait_for,
                             const ros::Duration& time_out)
{
  ensenso::EnsensoGuard guard;
  sensor_msgs::JointStateConstPtr state;
  ros::Time end_time = ros::Time::now() + time_out;

  // wait a fixed amount of time
  wait_for.sleep();

  while (ros::Time::now() < end_time)
  {
    state = ros::topic::waitForMessage<sensor_msgs::JointState>(
        JOINT_TOPIC_NAME, ros::Duration(DEFAULT_JOINT_TOPIC_WAIT_TIME));
    if (!state)
    {
      ROS_WARN("Could not get a joint_state in time");
      return false;
    }
    if (compare(state->position, end_goal))
    {
      ROS_INFO("Goal in tolerance. Returning control.");
      return true;
    }
  }
  return false;
}

static double toDegrees(double rads) { return rads * 180.0 / M_PI; }

static std::vector<double> toDegrees(const std::vector<double>& rads)
{
  std::vector<double> degrees;
  degrees.reserve(rads.size());

  for (std::size_t i = 0; i < rads.size(); ++i)
  {
    degrees.push_back(toDegrees(rads[i]));
  }
  return degrees;
}

static std::vector<rapid_emitter::TrajectoryPt>
toRapidTrajectory(const std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator start,
                  const std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator end,
                  const bool j23_coupled)
{
  std::vector<rapid_emitter::TrajectoryPt> rapid_pts;
  rapid_pts.reserve(std::distance(start, end));

  ros::Duration last_from_start;
  for (auto it = start; it < end; ++it)
  {
    // Retrieve and convert joint values to degrees
    std::vector<double> angles = toDegrees(it->positions);

    // Account for coupling if necessary
    if (j23_coupled)
    {
      ROS_ASSERT(it->positions.size() > 2);
      angles[2] += angles[1];
    }

    // Calculate between point timing
    double duration = 0.0;
    if (it != start)
    {
      duration = (it->time_from_start - last_from_start).toSec();
    }
    last_from_start = it->time_from_start;

    // Now we have all the info we need
    rapid_emitter::TrajectoryPt rapid_point(angles, duration);
    rapid_pts.push_back(rapid_point);
  }
  return rapid_pts;
}

static std::vector<rapid_emitter::TrajectoryPt>
toRapidTrajectory(const trajectory_msgs::JointTrajectory& traj, bool j23_coupled)
{
  return toRapidTrajectory(traj.points.begin(), traj.points.end(), j23_coupled);
}

static bool writeRapidFile(const std::string& path,
                           const std::vector<rapid_emitter::TrajectoryPt>& approach,
                           const std::vector<rapid_emitter::TrajectoryPt>& depart,
                           const std::vector<rapid_emitter::TrajectorySegment>& process_segments,
                           const rapid_emitter::ProcessParams& params)
{
  std::ofstream fp("/tmp/blend.mod");
  if (!fp)
  {
    ROS_ERROR_STREAM("Unable to create file: " << path);
    return false;
  }

  if (!rapid_emitter::emitRapidFile(fp, approach, depart, process_segments, params))
  {
    ROS_ERROR("Unable to write to RAPID file for blending process.");
    return false;
  }

  fp.flush();
  return true;
}

godel_process_execution::AbbBlendProcessService::AbbBlendProcessService(ros::NodeHandle& nh)
  : nh_(nh)
  , process_exe_action_server_(nh_, PROCESS_EXE_ACTION_SERVER_NAME,
                               boost::bind(&godel_process_execution::AbbBlendProcessService::executionCallback, this, _1),
                               false)
{
  // Load Robot Specific Parameters
  nh_.param<bool>("J23_coupled", params_.j23_coupled, false);
  nh_.param<std::string>("io_name", params_.io_name, "do_PIO_8");
  nh_.param<bool>("wolf_mode", params_.wolf_mode, false);

  // Create client services
  sim_client_ = nh_.serviceClient<industrial_robot_simulator_service::SimulateTrajectory>(SIMULATION_SERVICE_NAME);
  real_client_ = nh_.serviceClient<abb_file_suite::ExecuteProgram>(EXECUTION_SERVICE_NAME);
  process_exe_action_server_.start();
}

void godel_process_execution::AbbBlendProcessService::executionCallback(
    const godel_msgs::ProcessExecutionGoalConstPtr &goal)
{
  godel_msgs::ProcessExecutionResult res;
  if (goal->simulate)
  {
    res.success = simulateProcess(goal);
  }
  else
  {
    res.success = executeProcess(goal);
  }
  process_exe_action_server_.setSucceeded(res);
}

bool godel_process_execution::AbbBlendProcessService::executeProcess(
    const godel_msgs::ProcessExecutionGoalConstPtr &goal)
{

  const auto meta_path_size = std::accumulate(goal->meta_info.segment_size.begin(),
                                              goal->meta_info.segment_size.end(), 0u);
  if (meta_path_size != goal->trajectory_process.points.size())
  {
    ROS_ERROR_STREAM("Meta Info Msg Accounts for " << meta_path_size << " points, which does not equal the process "
                     "path size of " << goal->trajectory_process.points.size());
    return false;
  }

  // A utility for safety-checking speed values that come in via execution requests
  const auto clamp_speed = [] (const double speed) {
    const static double min_speed = 1.0; // mm per second
    const static double max_speed = 1000.0; // mm per second
    return std::min(std::max(min_speed, speed), max_speed);
  };

  // Load parameters required for post-processing
  rapid_emitter::ProcessParams params;
  params.wolf_mode = params_.wolf_mode;
  params.slide_force = goal->blend_params.tool_force;
  params.spindle_speed = goal->blend_params.spindle_speed;

  // Note that ROS represents speeds in terms of meters per second
  // ABB represents speeds in mm per second, hence the multiplication by 1000 in these expressions
  params.process_speed = clamp_speed(goal->blend_params.blending_spd * 1000.0);
  params.traversal_speed = clamp_speed(goal->blend_params.traverse_spd * 1000.0);
  params.approach_speed = clamp_speed(goal->blend_params.approach_spd * 1000.0);

  params.output_name = params_.io_name;


  auto approach = toRapidTrajectory(goal->trajectory_approach, params_.j23_coupled);
  auto depart = toRapidTrajectory(goal->trajectory_depart, params_.j23_coupled);

  // Now we must unpack the segments of the process path
  std::vector<rapid_emitter::TrajectorySegment> segments;
  segments.reserve(goal->meta_info.segment_size.size());

  std::size_t running_count = 0;
  for (std::size_t i = 0; i < goal->meta_info.segment_size.size(); ++i)
  {
    const auto segment_size = goal->meta_info.segment_size[i];
    const auto segment_type = goal->meta_info.segment_types[i];

    rapid_emitter::TrajectorySegment segment;
    if (segment_type == godel_msgs::PathMetaInfo::SEGMENT_APPROACH_TYPE)
      segment.type = segment.APPROACH;
    else if (segment_type == godel_msgs::PathMetaInfo::SEGMENT_PROCESS_TYPE)
      segment.type = segment.PROCESS;
    else
      segment.type = segment.TRAVERSE;

    const auto path_start = goal->trajectory_process.points.begin() + running_count;
    const auto path_end = path_start + segment_size;
    running_count += segment_size;
    segment.points = toRapidTrajectory(path_start, path_end, params_.j23_coupled);
    segments.push_back(segment);
  }

  if (!writeRapidFile("/tmp/blend.mod", approach, depart, segments, params))
  {
    ROS_ERROR("Unable to generate RAPID motion file; Cannot execute process.");
    return false;
  }

  // Call the ABB driver
  abb_file_suite::ExecuteProgram srv;
  srv.request.file_path = "/tmp/blend.mod";

  if (!real_client_.call(srv))
  {
    ROS_ERROR("Unable to upload blending process RAPID module to controller via FTP.");
    return false;
  }

  if (goal->wait_for_execution)
  {
    const auto& final_pose = goal->trajectory_depart.points.back().positions;
    const auto& total_time = goal->trajectory_approach.points.back().time_from_start +
                             goal->trajectory_process.points.back().time_from_start +
                             goal->trajectory_depart.points.back().time_from_start;
    const auto& timeout = total_time + ros::Duration(DEFAULT_TRAJECTORY_BUFFER_TIME);
    // If we must wait for execution, then block and listen until robot returns to initial point or times out
    return waitForExecution(final_pose, total_time, timeout);
  }
  else
  {
    return true;
  }
}

bool godel_process_execution::AbbBlendProcessService::simulateProcess(
    const godel_msgs::ProcessExecutionGoalConstPtr &goal)
{
  // The simulation server doesn't support any I/O visualizations, so we aggregate the
  // trajectory components and send them all at once
  trajectory_msgs::JointTrajectory aggregate_traj;
  aggregate_traj = goal->trajectory_approach;
  appendTrajectory(aggregate_traj, goal->trajectory_process);
  appendTrajectory(aggregate_traj, goal->trajectory_depart);

  // Pass the trajectory to the simulation service
  industrial_robot_simulator_service::SimulateTrajectory srv;
  srv.request.trajectory = aggregate_traj;

  // Call simulation service
  if (!sim_client_.call(srv))
  {
    ROS_ERROR("Simulation client unavailable or unable to simulate trajectory.");
    return false;
  }
  else
  {
    return true;
  }
}

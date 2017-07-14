#include "path_transitions.h"

/**
 * @brief Computes the angle theta required to move from \e a to \e b.
 * http://math.stackexchange.com/questions/90081/quaternion-distance
 * @param a Initial rotation
 * @param b Final rotation
 * @return Distance in radians (unsigned)
 */
static double quaternionDistance(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
{
  return std::acos(2.0 * std::pow(a.dot(b), 2.0) - 1.0);
}

/**
 * @brief Because we currently constrain the robot to transition linearly between points, we frequently fail on
 * scan paths whose direction flip as the robot kinematics do not allow such a transition. This tests the next
 * pose and sees if the rotation 180 deg about Z is "closer" in rotational space than the \e nominal_stop pose.
 * @param start
 * @param nominal_stop
 * @return \e nominal_stop if its closer, else 180 about Z from \e nominal_stop
 */
static Eigen::Affine3d closestRotationalPose(const Eigen::Affine3d& start, const Eigen::Affine3d& nominal_stop)
{
  Eigen::Affine3d alt_candidate = nominal_stop * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1));

  const auto distance1 = quaternionDistance(Eigen::Quaterniond(start.rotation()),
                                            Eigen::Quaterniond(nominal_stop.rotation()));
  const auto distance2 = quaternionDistance(Eigen::Quaterniond(start.rotation()),
                                            Eigen::Quaterniond(alt_candidate.rotation()));
  if (distance1 < distance2)
  {
    return nominal_stop;
  }
  else
  {
    return alt_candidate;
  }
}

static EigenSTL::vector_Affine3d interpolateCartesian(const Eigen::Affine3d& start, const Eigen::Affine3d& stop,
                                                      const double ds, const double dt)
{
  // Required position change
  Eigen::Vector3d delta_translation = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  Eigen::Affine3d stop_prime = start.inverse()*stop; //This the stop pose represented in the start pose coordinate system
  Eigen::AngleAxisd delta_rotation(stop_prime.rotation());

  // Calculate number of steps
  unsigned steps_translation = static_cast<unsigned>(delta_translation.norm() / ds) + 1;
  unsigned steps_rotation = static_cast<unsigned>(delta_rotation.angle() / dt) + 1;
  unsigned steps = std::max(steps_translation, steps_rotation);

  // Step size
  Eigen::Vector3d step = delta_translation / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q (start.rotation());
  Eigen::Quaterniond stop_q (stop.rotation());
  double slerp_ratio = 1.0 / steps;

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > result;
  Eigen::Vector3d trans;
  Eigen::Quaterniond q;
  Eigen::Affine3d pose;
  result.reserve(steps+1);
  for (unsigned i = 0; i <= steps; ++i)
  {
    trans = start_pos + step * i;
    q = start_q.slerp(slerp_ratio * i, stop_q);
    pose = (Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

static EigenSTL::vector_Affine3d retractPath(const Eigen::Affine3d& start, double retract_dist, double traverse_height,
                                             const double linear_disc, const double angular_disc)
{

  Eigen::Affine3d a = start * Eigen::Translation3d(0, 0, retract_dist);

  if (a.translation().z() > traverse_height)
  {
    traverse_height = a.translation().z() + linear_disc;
  }
  Eigen::Affine3d b = a;
  b.translation().z() = traverse_height;

  auto segment_a = interpolateCartesian(start, a, linear_disc, angular_disc);
  auto segment_b = interpolateCartesian(a, b, linear_disc, angular_disc);

  EigenSTL::vector_Affine3d result;
  result.insert(result.end(), segment_a.begin(), segment_a.end());
  result.insert(result.end(), segment_b.begin() + 1, segment_b.end());

  return result;
}

std::vector<godel_process_planning::ConnectingPath>
godel_process_planning::generateTransitions(const std::vector<geometry_msgs::PoseArray> &segments,
                                            const TransitionParameters& params)
{
  auto traverse_height = params.traverse_height;
  if (traverse_height < 0.05)
  {
    ROS_WARN("Forcing traverse height to at least 0.05m to protect against broken configurations "
             "user requested height = %f.", traverse_height);
    traverse_height = 0.1;
  }
  std::vector<ConnectingPath> result;

  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    // First we extract the initial and final position of the path segment
    const auto& start_pose = segments[i].poses.front(); // First point in this segment
    const auto& end_pose = segments[i].poses.back();    // Last point in this segment

    Eigen::Affine3d e_start, e_end;
    tf::poseMsgToEigen(start_pose, e_start);
    tf::poseMsgToEigen(end_pose, e_end);

    // Now we want to generate our intermediate waypoints
    auto approach = retractPath(e_start, params.retract_dist,traverse_height, params.linear_disc,
                                params.angular_disc);
    auto depart = retractPath(e_end, params.retract_dist, traverse_height, params.linear_disc,
                              params.angular_disc);
    std::reverse(approach.begin(), approach.end()); // we flip the 'to' path to keep the time ordering of the path

    ConnectingPath c;
    c.depart = std::move(depart);
    c.approach = std::move(approach);
    result.push_back(c);
  }
  return result;
}

using DescartesConversionFunc =
  boost::function<descartes_core::TrajectoryPtPtr (const Eigen::Affine3d &, const double)>;

godel_process_planning::DescartesTraj
godel_process_planning::toDescartesTraj(const std::vector<geometry_msgs::PoseArray> &segments,
                                        const ToolSpeeds& speeds, const TransitionParameters& transition_params,
                                        DescartesConversionFunc conversion_fn, PathMetaInfo* meta)
{
  auto transitions = generateTransitions(segments, transition_params);

  DescartesTraj traj;
  Eigen::Affine3d last_pose = createNominalTransform(segments.front().poses.front());

  // Keep track of the meta-information about the size and type of each segment of the path
  PathMetaInfo temp_meta;
  temp_meta.speeds = speeds;

  // Convert pose arrays to Eigen types
  auto eigen_segments = toEigenArrays(segments);

  // Inline function for adding a sequence of motions
  auto add_segment = [&traj, &last_pose, &temp_meta, conversion_fn, transition_params]
                     (const EigenSTL::vector_Affine3d& poses, const double speed, bool free_last,
                      PathMetaInfo::Type type)
  {
    PathMetaInfo::Segment meta_segment;
    const auto start_size = traj.size();

    // Create Descartes trajectory for the segment path
    for (std::size_t j = 0; j < poses.size(); ++j)
    {
      Eigen::Affine3d this_pose = createNominalTransform(poses[j], transition_params.z_adjust);
      // O(1) jerky - may need to revisit this time parameterization later. This at least allows
      // Descartes to perform some optimizations in its graph serach.
      double dt = (this_pose.translation() - last_pose.translation()).norm() / speed;

      if (dt < 1e-4)
      {
        continue;
      }

      if (j == poses.size() - 1 && free_last)
      {
        dt = 0.0;
      }
      traj.push_back( conversion_fn(this_pose, dt) );
      last_pose = this_pose;
    }

    const auto end_size = traj.size();
    meta_segment.size = end_size - start_size;
    meta_segment.type = type;
    temp_meta.segments.push_back(meta_segment);
  };

  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    add_segment(transitions[i].approach, speeds.approach_speed, false, PathMetaInfo::Type::APPROACH);

    add_segment(eigen_segments[i], speeds.process_speed, false, PathMetaInfo::Type::PROCESS);

    add_segment(transitions[i].depart, speeds.approach_speed, false, PathMetaInfo::Type::APPROACH);

    if (i != segments.size() - 1)
    {
      // To keep the robot at a safe height while we have no model of the parts we're working on, this code enforces a
      // linear travel between poses. The call to closestRotationalPose allows the linear interpolation to happen to the
      // pose that is 180 degrees off (about Z) from the nominal one. The discretization in Descartes takes care of the rest.
      auto connection = interpolateCartesian(transitions[i].depart.back(),
                                             closestRotationalPose(transitions[i].depart.back(), transitions[i+1].approach.front()),
                                             transition_params.linear_disc, transition_params.angular_disc);
      add_segment(connection, speeds.traverse_speed, false, PathMetaInfo::Type::TRAVERSE);
    }
  } // end segments

  if (meta)
    *meta = temp_meta;

  return traj;
}

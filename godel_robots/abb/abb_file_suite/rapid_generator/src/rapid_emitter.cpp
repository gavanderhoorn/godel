#include "rapid_generator/rapid_emitter.h"
#include <iostream>

bool rapid_emitter::emitGrindMotion(std::ostream& os, const ProcessParams& params, size_t n,
                                    bool start, bool end)
{
  // The 'fine' zoning means the robot will stop at this point

  // Note that 'wolf_mode' indicates the presence of WolfWare software for the ABB which has
  // a sepcial instruction called the 'GrindL'. This encapsulates both a robot motion and the
  // state of a tool as it moves through the path. The 'Start' and 'End' varieties handle I/O.
  if (params.wolf_mode)
  {
    if (start)
    {
      os << "GrindLStart CalcRobT(jTarget_" << n << ",tGrind), v100, gr1, fine, tGrind;\n";
    }
    else if (end)
    {
      os << "GrindLEnd CalcRobT(jTarget_" << n << ",tGrind), v100, fine, tGrind;\n";
    }
    else
    {
      os << "GrindL CalcRobT(jTarget_" << n << ",tGrind), v100, z40, tGrind;\n";
    }
  }
  else
  {
    os << "MoveL CalcRobT(jTarget_" << n << ",tGrind), vProcessSpeed, z40, tGrind;\n";
  }
  return os.good();
}

bool rapid_emitter::emitFreeMotion(std::ostream& os, const ProcessParams& params, size_t n,
                                   double duration, bool stop_at)
{
  // We want the robot to move smoothly and stop at the last point; these tolerances ensure that
  // this happens
  const char* zone = stop_at ? "fine" : "z20";

  if (duration <= 0.0)
  {
    os << "MoveAbsJ jTarget_" << n << ", vMotionSpeed," << zone << ", tGrind;\n";
  }
  else
  {
    os << "MoveAbsJ jTarget_" << n << ", vMotionSpeed, \\T:=" << duration << ", "
       << zone << ", tGrind;\n";
  }
  return os.good();
}
bool rapid_emitter::emitJointPosition(std::ostream& os, const TrajectoryPt& pt, size_t n)
{
  os << "TASK PERS jointtarget jTarget_" << n << ":=[[";
  for (size_t i = 0; i < pt.positions_.size(); i++)
  {
    os << pt.positions_[i];
    if (i < pt.positions_.size() - 1)
    {
      os << ",";
    }
  }
  // We assume a six axis robot here.
  os << "],[9E9,9E9,9E9,9E9,9E9,9E9]];\n";
  return true;
}

bool rapid_emitter::emitSetOutput(std::ostream& os, const ProcessParams& params, size_t value)
{
  // Wolf instructions handle I/O internally, so this method should do nothing
  if (params.wolf_mode == false)
  {
    // This wait time is inserted here to ensure that the zone is achieved BEFORE the I/O happens
    os << "WaitTime\\InPos, 0.01;\n";
    os << "SETDO " << params.output_name << ", " << value << ";\n";
  }
  else
  {
    if (value)
    {
      os << "GrindStart gr1;" << "\n";
      os << "WaitTime\\InPos, RPM_REACHED;" << "\n";
    }
  }
  return os.good();
}

static void emitSpeedData(std::ostream& os, const std::string& name, const double linear_speed, const double angular_speed,
                          const double external_linear_speed = 500.0, const double external_angular_speed = 90.0)
{
  os << "CONST speeddata " << name << ":=[" << linear_speed << "," << angular_speed << "," << external_linear_speed
     << "," << external_angular_speed << "];\n";
}

bool rapid_emitter::emitProcessDeclarations(std::ostream& os, const ProcessParams& params,
                                            size_t value)
{
  if (params.wolf_mode)
  {
    os << "TASK PERS grinddata gr1:=[" << params.process_speed << ","
                                       << params.spindle_speed << ","
                                       << params.slide_force << ",FALSE,FALSE,FALSE,0,0];\n";
  }
  else
  {
    emitSpeedData(os, "vApproachSpeed", params.approach_speed, 45.0);
    emitSpeedData(os, "vProcessSpeed", params.process_speed, 45.0);
    emitSpeedData(os, "vTraverseSpeed", params.traversal_speed, 45.0);
  }

  emitSpeedData(os, "vMotionSpeed", 200, 30);

  return os.good();
}

bool rapid_emitter::emitJointTrajectoryFile(std::ostream& os,
                                            const std::vector<TrajectoryPt>& points,
                                            const ProcessParams& params)
{
  // Write header
  os << "MODULE mGodel_Blend\n\n";
  // Emit all of the joint points
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    emitJointPosition(os, points[i], i);
  }
  // Emit Process Declarations
  emitProcessDeclarations(os, params, 1);
  // Write beginning of procedure
  os << "\nPROC Godel_Blend()\n";
  // For 0 to lengthFreeMotion, emit free moves
  if (points.empty())
  {
    return false;
  }

  // Write first point as normal move abs j so that the robot gets to where it needs to be
  emitFreeMotion(os, params, 0, 0.0, true);

  // The second motion
  for (std::size_t i = 1; i < points.size() - 1; ++i)
  {
    emitFreeMotion(os, params, i, points[i].duration_, false);
  }

  // Stop at the last point
  emitFreeMotion(os, params, points.size() - 1, points.back().duration_, true);

  os << "EndProc\n";
  // write any footers including main procedure calling the above
  os << "ENDMODULE\n";

  return os.good();
}


static void emitMoveMotion(std::ostream& os, const std::size_t index, const std::string& speeddata,
                           const rapid_emitter::ProcessParams& params)
{
  os << "MoveL CalcRobT(jTarget_" << index << ",tGrind), " << speeddata << ", z40, tGrind;\n";
}

static void emitProcessMotion(std::ostream& os, const std::size_t index, bool start, bool end, bool is_last_segment,
                              const rapid_emitter::ProcessParams& params)
{
  if (params.wolf_mode)
  {
    if (start)
    {
      os << "GrindLStart CalcRobT(jTarget_" << index << ",tGrind), vProcessSpeed, gr1, fiindexe, tGrind;\n";
    }
    else if (end)
    {
      os << "GrindLEnd CalcRobT(jTarget_" << index << ",tGrind), vProcessSpeed, fine, tGrind";
      if (is_last_segment)
      {
        os << "\\KeepOn:=True";
      }
      os << ";\n";
    }
    else
    {
      os << "GrindL CalcRobT(jTarget_" << index << ",tGrind), vProcessSpeed, z40, tGrind;\n";
    }
  }
  else
  {
    emitMoveMotion(os, index, "vProcessSpeed", params);
  }
}

static void emitApproachPath(std::ostream& os, const std::size_t segment_size,
                             const rapid_emitter::ProcessParams& params, std::size_t& running_count)
{
  for (std::size_t i = 0; i < segment_size; ++i)
  {
    emitMoveMotion(os, running_count++, "vApproachSpeed", params);
  }
}

static void emitTraversePath(std::ostream& os, const std::size_t segment_size,
                             const rapid_emitter::ProcessParams& params, std::size_t& running_count)
{
  for (std::size_t i = 0; i < segment_size; ++i)
  {
    emitMoveMotion(os, running_count++, "vTraverseSpeed", params);
  }
}

static void emitProcessPath(std::ostream& os, const std::size_t segment_size,
                            const rapid_emitter::ProcessParams& params, const bool is_last_segment,
                            std::size_t& running_count)
{
  for (std::size_t i = 0; i < segment_size; ++i)
  {
    bool start = i == 0;
    bool end = i == (segment_size - 1);
    emitProcessMotion(os, running_count++, start, end, is_last_segment, params);
  }
}

static std::pair<bool, size_t> findLastProcessIndex(const std::vector<rapid_emitter::TrajectorySegment>& segments)
{
  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    const auto current_index = segments.size() - 1 - i;
    if (segments[current_index].type == rapid_emitter::TrajectorySegment::PROCESS)
    {
      return {true, current_index};
    }
  }
  return {false, 0};
}

bool rapid_emitter::emitRapidFile(std::ostream &os, const std::vector<rapid_emitter::TrajectoryPt> &approach,
                                  const std::vector<rapid_emitter::TrajectoryPt> &departure,
                                  const std::vector<rapid_emitter::TrajectorySegment> &segments,
                                  const rapid_emitter::ProcessParams &params)
{

  // Write header
  os << "MODULE mGodel_Blend\n\n";

  // Emit all joint positions
  std::size_t count = 0;
  for (const auto& pt : approach)
  {
    emitJointPosition(os, pt, count++);
  }

  for (const auto& segment : segments)
  {
    for (const auto& pt : segment.points)
    {
      emitJointPosition(os, pt, count++);
    }
  }

  for (const auto& pt : departure)
  {
    emitJointPosition(os, pt, count++);
  }

  // Emit Process Declarations
  emitProcessDeclarations(os, params, 1);

  // Write beginning of procedure
  os << "\nPROC Godel_Blend()\n";

  std::size_t running_count = 0;
  for (std::size_t i = 0; i < approach.size(); ++i)
  {
    const bool stop_at = (i == 0 || i == approach.size() - 1) ? true : false;
    const double duration = (i == 0) ? 0.0 : approach[i].duration_;

    emitFreeMotion(os, params, running_count++, duration, stop_at);
  }

  // Turn on the tool
  emitSetOutput(os, params, 1);

  // Determine the index of the last 'process' segment, if there is one
  const auto last_process_index = findLastProcessIndex(segments);

  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    const auto& segment = segments[i];

    const auto last_process_segment = last_process_index.first ? (i == last_process_index.second) : false;

    if (segment.type == TrajectorySegment::APPROACH)
      emitApproachPath(os, segment.points.size(), params, running_count);
    else if (segment.type == TrajectorySegment::PROCESS)
      emitProcessPath(os, segment.points.size(), params, last_process_segment, running_count);
    else if (segment.type == TrajectorySegment::TRAVERSE)
      emitTraversePath(os, segment.points.size(), params, running_count);
    else
      throw std::runtime_error("Unrecognized process type");
  }

  // Turn off the tool
  emitSetOutput(os, params, 0);

  for (std::size_t i = 0; i < departure.size(); ++i)
  {
    const bool stop_at = (i == 0 || i == departure.size() - 1) ? true : false;
    const double duration = (i == 0) ? 0.0 : departure[i].duration_;

    emitFreeMotion(os, params, running_count++, duration, stop_at);
  }

  os << "EndProc\n";

  // write any footers including main procedure calling the above
  os << "ENDMODULE\n";
  return true;
}

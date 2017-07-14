#ifndef ABB_BLEND_PROCESS_SERVICE_H
#define ABB_BLEND_PROCESS_SERVICE_H

#include <ros/ros.h>
#include <godel_msgs/ProcessExecutionAction.h>
#include <actionlib/server/simple_action_server.h>

namespace godel_process_execution
{

/**
 * @brief Parameters container for those things that are largely meant to be
 * constant and not imagined to change run-to-run.
 */
struct AbbBlendProcessParameters
{
  std::string io_name;
  bool j23_coupled;
  bool wolf_mode;
};

class AbbBlendProcessService
{
public:
  AbbBlendProcessService(ros::NodeHandle& nh);

  /**
   * Currently forwards the godel_msgs::TrajectoryExecution on to the corresponding
   * MoveIt node. The idea though is that abstracting 'execution' will give us more flexibility
   * later to implement our own process parameters related to execution.
   */
  void executionCallback(const godel_msgs::ProcessExecutionGoalConstPtr &goal);
  bool executeProcess(const godel_msgs::ProcessExecutionGoalConstPtr &goal);
  bool simulateProcess(const godel_msgs::ProcessExecutionGoalConstPtr &goal);

private:
  ros::NodeHandle nh_;
  ros::ServiceClient real_client_;
  ros::ServiceClient sim_client_;
  actionlib::SimpleActionServer<godel_msgs::ProcessExecutionAction> process_exe_action_server_;
  AbbBlendProcessParameters params_;
};
}

#endif // ABB_BLEND_PROCESS_SERVICE_H

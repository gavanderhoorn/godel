#ifndef GODEL_BLEND_PLAN_CONFIG_WINDOW_H
#define GODEL_BLEND_PLAN_CONFIG_WINDOW_H

#include <godel_plugins/widgets/parameter_window_base.h>
#include <godel_msgs/BlendingPlanParameters.h>
#include <ui_blend_plan_config_widget.h>

namespace godel_plugins
{

class BlendPlanConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  BlendPlanConfigWidget(const godel_msgs::BlendingPlanParameters& params);

  godel_msgs::BlendingPlanParameters& params() { return params_; }

protected:
  virtual void update_gui_fields();
  virtual void update_internal_values();

  godel_msgs::BlendingPlanParameters params_;
  Ui::BlendPlanConfigWidget ui_;
};

}

#endif // GODEL_BLEND_PLAN_CONFIG_WINDOW_H

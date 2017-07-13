#include <godel_plugins/widgets/blend_plan_config_widget.h>

godel_plugins::BlendPlanConfigWidget::BlendPlanConfigWidget(const godel_msgs::BlendingPlanParameters &params)
  : params_(params)
{
  ui_.setupUi(this);

  connect(ui_.PushButtonAccept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_.PushButtonCancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_.PushButtonSave, SIGNAL(clicked()), this, SLOT(save_changes_handler()));
}

void godel_plugins::BlendPlanConfigWidget::update_gui_fields()
{
  ui_.lineEditToolRadius->setText(QString::number(params_.tool_radius));
  // Margin is currently un-used
  ui_.lineEditOverlap->setText(QString::number(params_.overlap));

  ui_.lineEditSpindleSpeed->setText(QString::number(params_.spindle_speed));
  ui_.lineEditToolForce->setText(QString::number(params_.tool_force));

  ui_.lineEditProcessSpeed->setText(QString::number(params_.blending_spd));
  ui_.lineEditTraverseSpeed->setText(QString::number(params_.traverse_spd));
  ui_.lineEditApproachSpeed->setText(QString::number(params_.approach_spd));

  ui_.lineEditDiscretization->setText(QString::number(params_.discretization));
  ui_.lineEditTraverseHeight->setText(QString::number(params_.safe_traverse_height));
  ui_.lineEditZAdjust->setText(QString::number(params_.z_adjust));
}

void godel_plugins::BlendPlanConfigWidget::update_internal_values()
{
  // Block of un-used parameters
  params_.margin = 0.0;
  params_.retract_spd = 0.0;
  params_.min_boundary_length = 0.0;

  params_.tool_radius = ui_.lineEditToolRadius->text().toDouble();
  params_.overlap = ui_.lineEditOverlap->text().toDouble();

  params_.spindle_speed = ui_.lineEditSpindleSpeed->text().toDouble();
  params_.tool_force = ui_.lineEditToolForce->text().toDouble();

  params_.blending_spd = ui_.lineEditProcessSpeed->text().toDouble();
  params_.traverse_spd = ui_.lineEditTraverseSpeed->text().toDouble();
  params_.approach_spd = ui_.lineEditApproachSpeed->text().toDouble();

  params_.discretization = ui_.lineEditDiscretization->text().toDouble();
  params_.safe_traverse_height = ui_.lineEditTraverseHeight->text().toDouble();
  params_.z_adjust = ui_.lineEditZAdjust->text().toDouble();
}

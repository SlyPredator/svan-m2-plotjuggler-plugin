#pragma once

#include <PlotJuggler/toolbox_base.h>

namespace plotjuggler_m2
{

class M2RobotViewWidget;

class M2RobotViewToolbox : public PJ::ToolboxPlugin
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.Toolbox")
  Q_INTERFACES(PJ::ToolboxPlugin)

public:
  M2RobotViewToolbox();
  ~M2RobotViewToolbox() override;

  const char* name() const override { return "Svan M2 Robot View"; }
  void init(PJ::PlotDataMapRef& src_data, PJ::TransformsMap& transform_map) override;
  std::pair<QWidget*, WidgetType> providedWidget() const override;

public slots:
  bool onShowWidget() override;

private:
  M2RobotViewWidget* widget_ = nullptr;
};

} // namespace plotjuggler_m2

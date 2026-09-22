#pragma once

#include <PlotJuggler/toolbox_base.h>
#include <QDialog>

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
  void toggleView();
  void togglePopOut();

private:
  M2RobotViewWidget* widget_ = nullptr;
  QObject* filter_ = nullptr;
  QWidget* saved_parent_ = nullptr;
  int saved_stacked_index_ = -1;
  bool is_popped_out_ = false;
  QDialog* pip_dialog_ = nullptr;
};

} // namespace plotjuggler_m2

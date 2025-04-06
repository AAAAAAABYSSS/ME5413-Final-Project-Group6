#pragma once

#include <rviz/panel.h>
#include <QProcess>
#include <QWidget>
#include <QTimer>

namespace interactive_tools
{

class PlotJugglerPanel : public rviz::Panel
{
  Q_OBJECT
public:
  PlotJugglerPanel(QWidget* parent = nullptr);
  ~PlotJugglerPanel() override;

protected:
  QProcess* process_;
  QTimer* embed_timer_;
  QWidget* container_;
  void tryEmbedPlotJuggler();
};

}  // namespace interactive_tools

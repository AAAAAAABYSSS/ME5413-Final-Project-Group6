#include "interactive_tools/plotjuggler_panel.hpp"
#include <QHBoxLayout>
#include <QX11Info>
#include <QWindow>
#include <QDebug>

namespace interactive_tools
{

PlotJugglerPanel::PlotJugglerPanel(QWidget* parent) : rviz::Panel(parent)
{
  QHBoxLayout* layout = new QHBoxLayout;
  container_ = new QWidget(this);
  layout->addWidget(container_);
  setLayout(layout);

  process_ = new QProcess(this);
  process_->start("plotjuggler", QStringList() << "--streaming");

  embed_timer_ = new QTimer(this);
  embed_timer_->setInterval(1000);
  connect(embed_timer_, &QTimer::timeout, this, &PlotJugglerPanel::tryEmbedPlotJuggler);
  embed_timer_->start();
}

PlotJugglerPanel::~PlotJugglerPanel()
{
  if (process_->state() == QProcess::Running)
    process_->terminate();
}

void PlotJugglerPanel::tryEmbedPlotJuggler()
{
  system("xdotool search --name PlotJuggler > /tmp/pj_id.txt");
  QFile file("/tmp/pj_id.txt");
  if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QString wid_str = file.readLine().trimmed();
    bool ok = false;
    WId wid = wid_str.toULong(&ok, 10);
    if (ok && wid) {
      QWindow* external_win = QWindow::fromWinId(wid);
      QWidget* external_widget = QWidget::createWindowContainer(external_win, container_);
      QHBoxLayout* embed_layout = new QHBoxLayout(container_);
      embed_layout->addWidget(external_widget);
      container_->setLayout(embed_layout);
      embed_timer_->stop();
    }
  }
}

}  // namespace interactive_tools

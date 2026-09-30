// SPDX-License-Identifier: BSD-3-Clause
#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

// Pace the rendering thread, not physics or the Qt event loop. WSLg may not
// throttle swaps to the monitor refresh rate, so vsync alone is insufficient.
class RenderRateLimit : public gz::gui::Plugin
{
  Q_OBJECT
  using Clock = std::chrono::steady_clock;
  Clock::duration period_{std::chrono::milliseconds(17)};
  Clock::time_point previous_{}, report_{};
  unsigned frames_{0};

  void LoadConfig(const tinyxml2::XMLElement *config) override
  {
    double fps = 60.0;
    if (config && config->FirstChildElement("fps"))
      config->FirstChildElement("fps")->QueryDoubleText(&fps);
    fps = std::clamp(fps, 1.0, 60.0);
    period_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / fps));
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
    std::clog << "GUI rendering limited to " << fps << " fps" << std::endl;
  }

  bool eventFilter(QObject *object, QEvent *event) override
  {
    if (event->type() == gz::gui::events::PreRender::kType)
    {
      if (previous_ != Clock::time_point{}) std::this_thread::sleep_until(previous_ + period_);
      previous_ = Clock::now();
      if (report_ == Clock::time_point{}) report_ = previous_;
      ++frames_;
      const double elapsed = std::chrono::duration<double>(previous_ - report_).count();
      if (elapsed >= 10.0)
      {
        std::clog << "GUI render rate: " << frames_ / elapsed << " fps" << std::endl;
        frames_ = 0;
        report_ = previous_;
      }
    }
    return QObject::eventFilter(object, event);
  }
};
GZ_ADD_PLUGIN(RenderRateLimit, gz::gui::Plugin)
#include "render_rate_limit.moc"

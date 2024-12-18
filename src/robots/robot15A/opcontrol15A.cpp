#include "robot15A.hpp"

namespace atum {
void Robot15A::opcontrol() {
  drive->setPose({2_tile, 0_tile, 0_deg});
  while(true) {
    const Pose pos{drive->getPose()};
    GUI::Map::addPosition(pos, GUI::SeriesColor::Green);
    GUI::Graph::setSeriesRange(6000, GUI::SeriesColor::Red);
        GUI::Graph::addPoint(getValueAs<feet_per_second_t>(pos.v) * 1000,
                             GUI::SeriesColor::Red);
    GUI::Graph::setSeriesRange(3600, GUI::SeriesColor::Red);
    GUI::Graph::addPoint(getValueAs<degrees_per_second_t>(pos.w) * 10, GUI::SeriesColor::Blue);

    const double forward{remote.getLStick().y};
    const double turn{remote.getRStick().x};
    drive->arcade(forward, turn);

    switch(remote.getRTrigger()) {
      case -1: intake->outtake(); break;
      case 1: intake->intake(); break;
      default: intake->stop(); break;
    }

    switch(remote.getLTrigger()) {
      case -1: ladybrownArm.moveVoltage(-6); break;
      case 1: ladybrownArm.moveVoltage(6); break;
      default: ladybrownArm.brake(); break;
    }

    if(remote.getPress(Remote::Button::A)) {
      goalClamp.toggle();
    }

    if(remote.getPress(Remote::Button::X)) {
      ladybrownWrist.toggle();
    }

    if(remote.getPress(Remote::Button::Up) &&
       remote.getPress(Remote::Button::Down)) {
      GUI::Manager::easteregg();
    }

    remote.print(
        0,
        "Brain: " +
            std::to_string(static_cast<int>(pros::battery::get_capacity())) +
            "%");
    wait(10_ms);
  }
}
} // namespace atum
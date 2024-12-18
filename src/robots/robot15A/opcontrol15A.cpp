#include "robot15A.hpp"

namespace atum {
void Robot15A::opcontrol() {
  odometry->setPosition({2_tile, 0_tile, 0_deg});
  while(true) {
    const Pose pos{odometry->getPosition()};
    GUI::Map::addPosition(pos, GUI::SeriesColor::Green);

    const double forward{remote.getLStick().y};
    const double turn{remote.getRStick().x};
    leftMotors.moveVoltage(forward + turn);
    rightMotors.moveVoltage(forward - turn);

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

    if(remote.getPress(Remote::Button::Up) && remote.getPress(Remote::Button::Down)) {
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
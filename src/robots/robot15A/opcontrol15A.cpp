#include "robot15A.hpp"

namespace atum {
void Robot15A::opcontrol() {
  Timer fifteenAway{1_min};
  Schedule fifteenAwayScheduled{{"Rumble at 15s Away",
                                 fifteenAway.checkGoneOff(),
                                 [=]() { remote.rumble("---"); }},
                                Logger::Level::Debug};
  drive->setPose({2_tile, 0_tile, 0_deg});
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    intake->setSortOutColor(ColorSensor::Color::Red);
  } else {
    intake->setSortOutColor(ColorSensor::Color::Blue);
  }
  bool manualControls{false};
  bool ladybrownMode{false};
  while(true) {
    const Pose pos{drive->getPose()};
    GUI::Map::addPosition(pos, GUI::SeriesColor::Green);

    const double forward{remote.getLStick().y};
    const double turn{remote.getRStick().x};
    // drive->arcade(forward, turn);

    remote.print(
        0,
        "BRAIN: " +
            std::to_string(static_cast<int>(pros::battery::get_capacity())) +
            "%");
    remote.print(1, "SORT: " + toString(intake->getSortOutColor()));

    if(manualControls) {
      remote.print(2, "MODE: Manual");

      switch(remote.getLTrigger()) {
        case -1: ladybrown->retract(); break;
        case 1: ladybrown->extend(); break;
        default: ladybrown->stop(); break;
      }

      switch(remote.getRTrigger()) {
        case -1: intake->outtake(); break;
        case 1: intake->intake(); break;
        default: intake->stop(); break;
      }
    } else if(ladybrownMode) {
      remote.print(2, "MODE: Ladybrown");

      switch(remote.getLTrigger()) {
        case -1:
          if(ladybrown->noRingDetection() || !ladybrown->hasRing()) {
            ladybrown->rest();
          }
          break;
        case 1:
          if(ladybrown->readyToScore()) {
            ladybrown->score();
          }
          break;
      }

      switch(remote.getRTrigger()) {
        case -1: intake->outtake(); break;
        case 1: intake->load(); break;
        default: intake->stop(); break;
      }
    } else {
      remote.print(2, "MODE: Intake");

      // Only go down if you know it's okay to go down.
      if(!ladybrown->noRingDetection() && !ladybrown->hasRing()) {
        ladybrown->rest();
      }

      switch(remote.getRTrigger()) {
        case -1: intake->outtake(); break;
        case 1:
          if(remote.getHold(Remote::Button::L1) || goalClamp.isExtended()) {
            intake->intake();
          } else {
            intake->index();
          }
          break;
        default: intake->stop(); break;
      }
    }

    if(remote.getPress(Remote::Button::A)) {
      goalClamp.toggle();
    }

    if(remote.getPress(Remote::Button::X)) {
      manualControls = !manualControls;
    }

    if(remote.getPress(Remote::Button::Y)) {
      ladybrownMode = !ladybrownMode;
    }

    if(remote.getPress(Remote::Button::Right)) {
      const ColorSensor::Color currentColor{intake->getSortOutColor()};
      const ColorSensor::Color nextColor{(static_cast<int>(currentColor) + 1) %
                                         3};
      intake->setSortOutColor(nextColor);
    }

    if(remote.getPress(Remote::Button::Up) &&
       remote.getPress(Remote::Button::Down)) {
      GUI::Manager::easteregg();
    }

    wait();
  } // namespace atum
} // namespace atum
} // namespace atum
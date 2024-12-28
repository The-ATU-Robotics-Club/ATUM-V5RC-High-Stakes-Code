#include "robot15A.hpp"

namespace atum {
void Robot15A::opcontrol() {
  Timer halfway{37.5_s};
  Schedule halfwayRumbleScheduled{{"Rumble at Halfway Time",
                                   halfway.checkGoneOff(),
                                   [=]() { remote.rumble("-"); }},
                                  Logger::Level::Debug};
  Timer fifteenAway{1_min};
  Schedule fifteenAwayScheduled{{"Rumble at 15s Away",
                                 fifteenAway.checkGoneOff(),
                                 [=]() { remote.rumble("---"); }},
                                Logger::Level::Debug};
  drive->setPose({2_tile, 0_tile, 0_deg});
  while(true) {
    const Pose pos{drive->getPose()};
    GUI::Map::addPosition(pos, GUI::SeriesColor::Green);

    const double forward{remote.getLStick().y};
    const double turn{remote.getRStick().x};
    // drive->arcade(forward, turn);

    switch(remote.getRTrigger()) {
      case -1: intake->outtake(); break;
      case 1: intake->load(); break;
      default: intake->stop(); break;
    }
    if(intake->getState() != IntakeState::Loading) {
      switch(remote.getLTrigger()) {
        case -1: ladybrown->score(); break;
        case 1:
          if(!ladybrown->hasRing()) {
            ladybrown->rest();
          }
          break;
      }
    }

    // switch(remote.getLTrigger()) {
    //   case -1: ladybrown->retract(); break;
    //   case 1: ladybrown->extend(); break;
    //   default: ladybrown->stop(); break;
    // }
    // if(remote.getPress(Remote::Button::L2)) {
    //   ladybrown->load();
    // } else if(remote.getPress(Remote::Button::L1)) {
    //   ladybrown->score();
    // }

    if(remote.getPress(Remote::Button::A)) {
      goalClamp.toggle();
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
    wait();
  }
}
} // namespace atum
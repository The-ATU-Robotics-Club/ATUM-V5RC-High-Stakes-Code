#include "atum/devices/colorSensor.hpp"
#include "robotClone.hpp"

namespace atum {
// Max drive velocity: 76.5 in. / s.
// Max drive acceleration: 153 in. / s^2.
ROUTINE_DEFINITIONS_FOR(RobotClone) {
  /*
    ___ _   _ _ _
   / __| |_(_) | |___
   \__ \ / / | | (_-<
   |___/_\_\_|_|_/__/
  */
  START_ROUTINE("Skills")

  if(id == ID15) {
    /*
      _ ___ _ _   ___ _   _ _ _
     / | __( | ) / __| |_(_) | |___
     | |__ \V V  \__ \ / / | | (_-<
     |_|___/     |___/_\_\_|_|_/__/

    */
    setupRoutine({-2.5_tile + 7.5_in, 1_tile, 90_deg});
    pathFollower->follow({{{-0.5_tile, 2_tile, 0_deg},
                           false,
                           Path::Parameters{2_tile, 30_in_per_s}}});
    // intake->setSortOutColor(ColorSensor::Color::None);
    // intake->index();
    // goalRush->extendArm();
    // pathFollower->follow({{{-0.45_tile, 1.625_tile, 10_deg},
    //                        false,
    //                        Path::Parameters{{0.1_tile, 3_tile}}}});
    // goalRush->grab();
    // intake->stop();
    // setSortToOpposite();
    // moveTo->reverse({-1_tile, 1_tile});
    // if(GUI::Routines::selectedColor() == MatchColor::Red) {
    //   turn->toward(20_deg);
    // } else {
    //   turn->toward(90_deg);
    // }
    // goalRush->release();
    // wait(200_ms);
    // clampWhenReady();
    // moveTo->reverse({-0.5_tile, 1.5_tile});
    // intake->intake();
  } else if(id == ID24) {
    /*
          ___ _ _  _ _   ___ _   _ _ _
         |_  ) | |( | ) / __| |_(_) | |___
          / /|_  _|V V  \__ \ / / | | (_-<
         /___| |_|      |___/_\_\_|_|_/__/

    */
    setupRoutine({});
  }

  END_ROUTINE

  /*
    ___ _ _  _ _     _       _
   |_  ) | |( | )   /_\ _  _| |_ ___
    / /|_  _|V V   / _ \ || |  _/ _ \
   /___| |_|      /_/ \_\_,_|\__\___/

  */
  START_ROUTINE("24\" Auto")
  setupRoutine({});
  END_ROUTINE

  /*
      _ ___ _ _     _       _
     / | __( | )   /_\ _  _| |_ ___
     | |__ \V V   / _ \ || |  _/ _ \
     |_|___/     /_/ \_\_,_|\__\___/

  */
  START_ROUTINE("15\" Auto")
  setupRoutine({});
  END_ROUTINE

  START_ROUTINE("Do Nothing")
  setupRoutine({});
  END_ROUTINE
}

void RobotClone::setupRoutine(Pose startingPose) {
  matchTimer.setTime();

  const bool flipped{GUI::Routines::selectedColor() == MatchColor::Blue};
  if(flipped) {
    startingPose.flip();
  }
  Movement::setFlipped(flipped);

  drive->setPose(startingPose);
  gps->setPose(startingPose);

  setSortToOpposite();

  goalClamp->unclamp();

  if(id == ID15) {
    intake->outtake();
    wait(0.1_s);
    intake->stop();
  }

  drive->setBrakeMode(pros::MotorBrake::brake);
}

void RobotClone::clampWhenReady(const second_t timeout) {
  scheduler.schedule({"Clamp When Ready",
                      [=]() { return goalClamp->hasGoal(); },
                      [=]() {
                        turn->interrupt();
                        moveTo->interrupt();
                        pathFollower->interrupt();
                        goalClamp->clamp();
                      },
                      timeout,
                      Scheduler::doNothing});
}

void RobotClone::setSortToOpposite() {
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    intake->setSortOutColor(ColorSensor::Color::Blue);
  } else {
    intake->setSortOutColor(ColorSensor::Color::Red);
  }
}
} // namespace atum

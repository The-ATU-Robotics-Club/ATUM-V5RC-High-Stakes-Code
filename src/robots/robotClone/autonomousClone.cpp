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
  } else if(id == ID24) {
    /*
          ___ _ _  _ _   ___ _   _ _ _
         |_  ) | |( | ) / __| |_(_) | |___
          / /|_  _|V V  \__ \ / / | | (_-<
         /___| |_|      |___/_\_\_|_|_/__/

    */
  }

  END_ROUTINE

  START_ROUTINE("Negative Side")
  setupRoutine({-2.5_tile + 7.5_in + (id == ID15 ? 0_in : 0_in), 1_tile, 90_deg});
  intake->setSortOutColor(ColorSensor::Color::None);
  goalRush->extendArm();
  goalRush->release();
  intake->outtake();
  const meter_t rushOffRamp{id == ID15 ? 1_tile : 1_tile - 0_in};
  pathFollower->follow(
      {{AcceptableDistance{3_s},
        {-0.45_tile, 1.55_tile, 30_deg},
        false,
        Path::Parameters{
            rushOffRamp, 0_in_per_s, 0_in_per_s_sq, 76.5_in_per_s_sq}}});
  goalRush->grab();
  wait(200_ms);
  moveTo->reverse({-1.25_tile, 0.75_tile});
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    turn->toward(0_deg);
  } else {
    turn->toward(110_deg);
  }
  goalRush->release();
  wait(200_ms);
  setSortToOpposite();
  intake->stop();
  clampWhenReady();
  moveTo->reverse({-0.45_tile, 1.55_tile},
                  LateralProfile::Parameters{30_in_per_s});
  intake->intake();
  goalClamp->clamp();  END_ROUTINE

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

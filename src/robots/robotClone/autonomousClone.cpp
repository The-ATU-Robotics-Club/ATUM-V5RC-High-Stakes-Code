#include "robotClone.hpp"
#include "robots/sharedSystems/intake.hpp"
#include "robots/sharedSystems/ladybrown.hpp"


namespace atum {
// Max drive velocity: 76.5 in. / s.
// Max drive acceleration: 153 in. / s^2.

// General Constants
static const second_t goalRushDelay{100_ms};
static const second_t goalClampDelay{100_ms};

/*
  _ ___ _ _   ___ _   _ _ _
 / | __( | ) / __| |_(_) | |___
 | |__ \V V  \__ \ / / | | (_-<
 |_|___/     |___/_\_\_|_|_/__/

*/
void RobotClone::skills15() {}

/*
      ___ _ _  _ _   ___ _   _ _ _
     |_  ) | |( | ) / __| |_(_) | |___
      / /|_  _|V V  \__ \ / / | | (_-<
     /___| |_|      |___/_\_\_|_|_/__/

*/
void RobotClone::skills24() {}

ROUTINE_DEFINITIONS_FOR(RobotClone) {
  START_ROUTINE("Skills")
  if(id == ID15) {
    skills15();
  } else if(id == ID24) {
    skills24();
  }
  END_ROUTINE

  START_ROUTINE("Negative Side: N/M Rush")
  setupRoutine({});
  moveToFar->forward(2_s, {0.0_tile, 4_tile}, 12.0, false);
  wait(1.5_s);
  moveToFar->reverse(2_s, {0.0_tile, 0.0_tile}, 12.0, false);
  END_ROUTINE

  START_ROUTINE("Do Nothing")
  setupRoutine({});
  intake->outtake();
  wait(3_s);
  intake->stop();
  END_ROUTINE

  

  START_ROUTINE("Positive Side: P Rush")
  setupRoutine({});
  moveToClose->reverse(2_s, {1_tile, 1.5_tile});
  clampWhenReady;
  moveToClose->reverse(2_s, {.5_tile, 1.75_tile});
  goalClamp->clamp();
  intake->intake();
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.75_tile, 2.25_tile});
  wait(100_ms);
  goalClamp->unclamp();
  moveToFar->forward(2_s, {1_tile, 0_tile});  
  END_ROUTINE


  START_ROUTINE("Negative Side: P Rush")
  setupRoutine({});
  moveToClose->reverse(2_s, {2_tile, .75_tile});
  clampWhenReady;
  goalClamp->clamp();
  intake->intake();
  wait(100_ms);
  intake->load();
  moveToClose->forward(2_s, {2.5_tile, 0_tile, -90_deg});
  ladybrown->extend();
  ladybrown->retract();
  wait(100_ms);
  intake->intake();
  moveToFar->forward(2_s, {1_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 2.5_tile});
  END_ROUTINE


  START_ROUTINE("Positive Side: M Rush")
  setupRoutine({});
  intake->index();
  moveToClose->forward(2_s, {2.5_tile, -.5_tile});
  wait(100_ms);
  clampWhenReady;
  moveToClose->reverse(2_s, {2_tile, 0_tile});
  wait(100_ms);
  goalClamp->clamp();
  intake->intake();
  moveToClose->forward(2_s, {2_tile, 1_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.5_tile, 2.5_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.75_tile, 2.25_tile});
  wait(100_ms);
  goalClamp->unclamp();
  moveToFar->forward(2_s, {1_tile, 0_tile});  
  END_ROUTINE


  START_ROUTINE("Negative Side: M Rush")
  setupRoutine({});
  goalRushWhenReady;
  clampWhenReady;
  goalClamp->clamp();
  intake->intake();
  wait(100_ms);
  intake->stop();
  moveToClose->forward(2_s, {1_tile, -1_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.5_tile, -2.5_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  intake->stop();
  intake->load();
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 0_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2_tile, 0_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 0_tile});
  ladybrown->extend();
  wait(100_ms);
  ladybrown->retract();
  moveToFar->forward(2_s, {2.5_tile, 2.5_tile});
  END_ROUTINE
  
  START_ROUTINE("Positive Side: N Rush")
  setupRoutine({});
  intake->index();
  clampWhenReady;
  moveToClose->forward(2_s, {2.5_tile, -.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2_tile, 0_tile});
  goalClamp->clamp();
  intake->intake();
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 1_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {1_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, 2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.5_tile, 2.5_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, 2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, 2.25_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.75_tile, 2.25_tile});
  wait(100_ms);
  goalClamp->unclamp();
  moveToFar->forward(2_s, {1_tile, 0_tile});

  
  END_ROUTINE

  START_ROUTINE("Negative Side: N Rush")
  setupRoutine({});
  clampWhenReady;
  goalClamp->clamp();
  intake->intake();
  moveToClose->forward(2_s, {1_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2_tile, -2_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.5_tile, -2.5_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  intake->stop();
  intake->load();
  wait(100_ms);
  moveToClose->forward(2_s, {2.75_tile, -2.75_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2.25_tile, -2.25_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 0_tile});
  wait(100_ms);
  moveToClose->reverse(2_s, {2_tile, 0_tile});
  wait(100_ms);
  moveToFar->forward(2_s, {2.5_tile, 0_tile});
  ladybrown->extend();
  wait(100_ms);
  ladybrown->retract();
  moveToFar->forward(2_s, {2.5_tile, 2.5_tile});
  END_ROUTINE


  START_ROUTINE("Positive Side: P/N Rush")
  setupRoutine({});
  moveToFar->reverse(5_s, {-1_tile, -1.5_tile});
  clampWhenReady();
  moveToFar->reverse(5_s, {-0.25_tile, -1.75_tile});
  goalClamp->clamp();
  goalClampDelay();
  intake->intake();
  wait(100_ms);
  goalClamp->toggleClamp();
  moveToFar->forward(5_s, {-2.5_tile, -1_tile});
  intake->index();
  moveToFar->forward(5_s, {-2.5_tile, 0.5_tile});
  clampWhenReady();
  moveToClose->reverse(5_s, {-1.8_tile, -0.2_tile});
  goalClamp->clamp();
  intake->intake();
  moveToClose->forward(5_s, {-2_tile, 1_tile});
  wait(1_s);
  moveToFar->forward(5_s, {-0.75_tile, -2.25_tile});
  moveToClose->forward(5_s, {-2.1_tile, -2.0_tile});
  moveToClose->forward(5_s, {-3_tile, -3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile});
  moveToClose->forward(5_s, {-3_tile, -3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile});
  moveToClose->forward(5_s, {-3_tile, -3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile});
  moveToClose->reverse(5_s, {-3_tile, -3_tile});
  goalClamp->toggleClamp();
  moveToFar->forward(5_s, {-1_tile, 3_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: P/N Rush")
  setupRoutine({});
  moveToClose->reverse(5_s, {-1_tile, 1.5_tile});
  goalRushL->extend();
  clampWhenReady();
  moveToClose->reverse(5_s, {-0.5_tile, 1.25_tile});
  goalClamp->clamp();
  goalClampDelay();
  intake->intake();
  wait(100_ms);
  moveToClose->forward(5_s, {-1.2_tile, 2.2_tile});
  moveToClose->forward(5_s, {-2_tile, 2_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  ladybrown->load();
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.75_tile, 2.75_tile});
  moveToFar->forward(5_s, {-2.8_tile, 0.2_tile});
  turn->toward(5_s, {-3_tile, 0_tile});
  ladybrown->extend();
  wait(500_ms);
  ladybrown->rest();
  wait(100_ms);
  moveToClose->forward(5_s, {-2.75_tile, -2.75_tile});

  
  END_ROUTINE


  START_ROUTINE("Positive Side: N/M Rush")
  setupRoutine({});
  moveToFar->reverse(5_s, {-1_tile, -1_tile});
  clampWhenReady();
  moveToClose->reverse(5_s, {-0.5_tile, -0.50_tile});
  goalClamp->clamp();
  goalClampDelay();
  intake->intake();
  wait(100_ms);
  moveToFar->forward(5_s, {-2.5_tile, -1_tile});
  intake->index();
  moveToFar->forward(5_s, {-2.5_tile, 0.5_tile});
  clampWhenReady();
  moveToClose->reverse(5_s, {-1.8_tile, -0.2_tile});
  goalClamp->clamp();
  intake->intake();
  moveToClose->forward(5_s, {-2_tile, 1_tile});
  wait(1_s);
  moveToFar->forward(5_s, {-0.75_tile, -2.25_tile});
  moveToClose->forward(5_s, {-2.2_tile, -2.0_tile});
  wait(100_ms);
  moveToClose->forward(5_s, {-3_tile, -3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile});
  moveToClose->forward(5_s, {-3_tile, -3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile});
  moveToClose->forward(5_s, {-3_tile, -3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, -2.5_tile});
  moveToClose->reverse(5_s, {-3_tile, -3_tile});
  goalClamp->toggleClamp();
  moveToFar->forward(5_s, {-1_tile, 3_tile});
  END_ROUTINE

  START_ROUTINE("Negative Side: N/M Rush")
  setupRoutine({});
  moveToClose->reverse(5_s, {-1_tile, 1.5_tile});
  goalRushL->extend();
  clampWhenReady();
  moveToClose->reverse(5_s, {-0.5_tile, 1.25_tile});
  goalClamp->clamp();
  goalClampDelay();
  intake->intake();
  wait(100_ms);
  moveToClose->forward(5_s, {-1.2_tile, 2.2_tile});
  moveToClose->forward(5_s, {-2_tile, 2_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  ladybrown->load();
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.5_tile, 2.5_tile});
  moveToClose->forward(5_s, {-3_tile, 3_tile});
  wait(500_ms);
  moveToClose->reverse(5_s, {-2.75_tile, 2.75_tile});
  moveToFar->forward(5_s, {-2.8_tile, 0.2_tile});
  turn->toward(5_s, {-3_tile, 0_tile});
  ladybrown->extend();
  wait(500_ms);
  ladybrown->rest();
  wait(100_ms);
  moveToClose->forward(5_s, {-2.75_tile, -2.75_tile});
  
  END_ROUTINE

  //START_ROUTINE("Negative Side: N/M Rush")
  //setupRoutine({});
  //moveToFar->forward(2_s, {0.0_tile, 4_tile}, 12.0, false);

}


    void RobotClone::safePositive(){
  
    }

    void RobotClone::safeNegative(){
  
    }

    void RobotClone::rushPositive(){
  
    }

    void RobotClone::rushNegative(){
  
    }

    void RobotClone::rushMidFromNegative(){
  
    }

    void RobotClone::rushMidFromPositive(){
  
    }

    void RobotClone::endOfPositiveRushRoutines(){
  
    }

    void RobotClone::endOfNegativeRushRoutines(){
  
    }

    void RobotClone::endOfPositiveSafeRoutines(){
  
    }

    void RobotClone::endOfNegativeSafeRoutines(){
  
    }

    void RobotClone::endOfPositiveDoubleRushRoutines(){
  
    }

    void RobotClone::endOfNegativeDoubleRushRoutines(){
  
    }
}
void RobotClone::setupRoutine(Pose startingPose){
  matchTimer.setTime();

  const bool flipped{GUI::Routines::selectedColor() == MatchColor::Blue};
  if(flipped) {
    startingPose.flip();
  }
  Movement::setFlipped(flipped);

  drive->setPose(startingPose);

  setSortToOpposite();

  goalClamp->unclamp();
  goalRush1->retract();
  goalRush2->retract();

  drive->setBrakeMode(pros::MotorBrake::brake);
}

void RobotClone::clampWhenReady(const second_t timeout) {
  scheduler.schedule({"Clamp When Ready",
                      [=]() { return goalClamp->hasGoal(); },
                      [=]() {
                        if(goalClamp->isClamped()) {
                          return;
                        }
                        goalClamp->clamp();
                        wait(goalClampDelay);
                        turn->interrupt();
                        moveToClose->interrupt();
                        pathFollower->interrupt();
                      },
                      timeout,
                      Scheduler::doNothing});
}

void RobotClone::goalRushWhenReady(const second_t timeout) {
  scheduler.schedule({"Goal Rush Grab When Ready",
                      [=]() { return goalRush1->hasGoal(); },
                      [=]() {
                        if(goalRush1->isUp()) {
                          return;
                        }
                        goalRush1->retract();
                        wait(goalRushDelay);
                        turn->interrupt();
                        moveToClose->interrupt();
                        pathFollower->interrupt();
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

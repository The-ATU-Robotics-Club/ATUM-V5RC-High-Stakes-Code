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

/*
  START_ROUTINE("Positive Safe")

  if(id == ID24){
  setupRoutine({ rushPositive });
  endOfPositiveRushRoutines();
  }
  else{
  setupRoutine({ safeNegative });
  endOfNegativeSafeRoutines();
  }
  END_ROUTINE


  START_ROUTINE("Mid Safe")
  if(id == ID24){
  setupRoutine({ rushMidFromNegative });
  endOfMidRushRoutines();
  }
  else{
  setupRoutine({ safePositive });
  endOfPositiveSafeRoutines();
  }
  END_ROUTINE


  START_ROUTINE("Negative Safe")
  if(id == ID24){
  setupRoutine({ rushNegative });
  endOfNegativeRushRoutines();
  }
  else{
  setupRoutine({ safePositive });
  endOfPositiveSafeRoutines();
  }
  END_ROUTINE


  START_ROUTINE("Positive Negative")
  if(id == ID24){
  setupRoutine({ rushNegative });     // 24 rushes negative because negative goal more important to hold 
  endOfNegativeRushRoutines();
  }
  else{
  setupRoutine({ rushPositive });
  endOfPositiveDoubleRushRoutines();
  }
  END_ROUTINE


  START_ROUTINE("Negative Mid")
  if(id == ID24){
  setupRoutine({ rushNegative });
  endOfNegativeDoubleRushRoutines();
  }
  else{
  setupRoutine({ rushMidFromPositive });
  endOfPositiveDoubleRushRoutines();
  }
  END_ROUTINE


  START_ROUTINE("Positive Mid")
  if(id == ID24){
  setupRoutine({ rushMidFromNegative });
  endOfNegativeDoubleRushRoutines();
  }
  else{
  setupRoutine({ rushPositive });
  endOfPositiveDoubleRushRoutines();
  }
  END_ROUTINE

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
    */
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

#include "atum/time/time.hpp"
#include "robotClone.hpp"

namespace atum {
// Max drive velocity: 76.5 in. / s.
// Max drive acceleration: 76.5 in. / s^2.
ROUTINE_DEFINITIONS_FOR(RobotClone) {
  /*
    ___ _   _ _ _
   / __| |_(_) | |___
   \__ \ / / | | (_-<
   |___/_\_\_|_|_/__/
  */

  START_ROUTINE("Skills")

  if(id == ID15) {
    setupRoutine({-5_ft, 5_ft, -45_deg});
  // (-1.520147 m, 1.485433 m, -0.725566 rad).
  
    // outake to allow the intake to drop of the rubber band
 
    intake->intake();

    pathFollower->follow({{{-5.6_ft, 5.6_ft, -45_deg}, false, Path::Parameters{0.1}}});

    intake->stop();

    // ring 1


    pathFollower->follow({{{-5_ft, 5_ft, 135_deg}, true, Path::Parameters{0.1, 38_in_per_s}}});

    turn->awayFrom(90_deg);

    pathFollower->follow({{{-2_ft, 4_ft, -45_deg}, true, Path::Parameters{1, 38_in_per_s}}});

    goalClamp->clamp();

    wait(100_ms);

    turn->toward(-90_deg);

    // ring 2
    intake->intake();

    pathFollower->follow({{{-4_ft, 4_ft, -90_deg}, false, Path::Parameters{1, 38_in_per_s}}});

    turn->toward(120_deg);

    pathFollower->follow({{{-2_ft, 2_ft, 120_deg}, false, Path::Parameters{.1, 38_in_per_s}}});

    wait(100_ms);

    turn->toward(110_deg);

    // ring 3


    pathFollower->follow({{{0.2_ft, -0.2_ft, 110_deg}, false, Path::Parameters{.1, 30_in_per_s}}});

    wait(2000_ms);

    pathFollower->follow({{{-2_ft, 2_ft, -45_deg}, true, Path::Parameters{.1, 50_in_per_s}}});

    turn->toward(30_deg);

     pathFollower->follow({{{0_ft, 4_ft, 45_deg}, false, Path::Parameters{.1, 50_in_per_s}}});

    pathFollower->follow({{{-5.8_ft, 5.8_ft, 135_deg}, true, Path::Parameters{3, 76.5_in_per_s}}});

    goalClamp->unclamp();

    ladybrown->prepare();

pathFollower->follow(
        {{{1.5_ft, -1.5_ft, 135_deg}, true, Path::Parameters{.1, 76.5_in_per_s}}});

        ladybrown->fullyExtend();

    pathFollower->follow({{{-1_ft, 1_ft, 135_deg}, true, Path::Parameters{.1, 30_in_per_s}}});


/*
    pathFollower->follow({{{0_ft, 5.2_ft, 90_deg}, false, Path::Parameters{.1, 76.5_in_per_s}}});

    turn->awayFrom(135_deg);

    pathFollower->follow({{{2_ft, 2_ft, -135_deg}, true, Path::Parameters{.1, 76.5_in_per_s}}});

    goalClamp->clamp();
    
    wait(10_ms);

    intake->intake();

    pathFollower->follow({{{2_ft, 4_ft, 0_deg}, false, Path::Parameters{.1, 76.5_in_per_s}}});
        pathFollower->follow({{{4_ft, 2_ft, 135_deg}, false, Path::Parameters{.1, 76.5_in_per_s}}});
            pathFollower->follow({{{4_ft, 4_ft, 0_deg}, false, Path::Parameters{.1, 76.5_in_per_s}}});
                pathFollower->follow({{{5.8_ft, 5.8_ft, 45_deg}, false, Path::Parameters{.1, 76.5_in_per_s}}});
                    pathFollower->follow({{{5_ft, 5_ft, 45_deg}, true, Path::Parameters{.1, 76.5_in_per_s}}});





    

    turn->toward(-45_deg);

    pathFollower->follow({{{-6_ft, 6_ft, -45_deg}, true, Path::Parameters{}}});

    goalClamp->unclamp();

    wait(100_ms);

    turn->toward(90_deg);

    pathFollower->follow({{{0_ft, 5_ft, 90_deg}, false, Path::Parameters{}}});

    // ring 6
    intake->intake();

    intake->stop();

    turn->toward(180_deg);

    pathFollower->follow({{{0_ft, 4_ft, 180_deg}, false, Path::Parameters{}}});

    // ring 7
    intake->intake();

    intake->stop();

    turn->toward(0_deg);

    pathFollower->follow({{{0_ft, 6_ft, 0_deg}, false, Path::Parameters{}}});

    ladybrown->load();

    ladybrown->score();

    ladybrown->finishScore();

    turn->toward(90_deg);

    pathFollower->follow({{{2_ft, 5_ft, 90_deg}, false, Path::Parameters{}}});

    // ring 8

    intake->intake();

    intake->stop();

    pathFollower->follow({{{0_ft, 5_ft, -90_deg}, true, Path::Parameters{}}});

    turn->toward(0_deg);

    ladybrown->load();

    ladybrown->score();

    ladybrown->finishScore();

    turn->toward(135_deg);

    pathFollower->follow({{{2_ft, 2_ft, 135_deg}, false, Path::Parameters{}}});

    goalClamp->clamp();

    wait(100_ms);

    turn->toward(0_deg);

    // ring 9

    intake->intake();

    pathFollower->follow({{{2_ft, 4_ft, 0_deg}, false, Path::Parameters{}}});

    turn->toward(90_deg);

    // ring 10

    pathFollower->follow({{{4_ft, 4_ft, 90_deg}, false, Path::Parameters{}}});

    intake->stop();

    turn->toward(45_deg);

    // ring 11

    intake->intake();

    pathFollower->follow(
        {{{5.8_ft, 5.8_ft, 45_deg}, false, Path::Parameters{}}});

    pathFollower->follow({{{5_ft, 5_ft, -135_deg}, true, Path::Parameters{}}});

    // ring 12

    pathFollower->follow(
        {{{5.8_ft, 5.8_ft, 45_deg}, false, Path::Parameters{}}});

    intake->stop();

    pathFollower->follow({{{2_ft, 2_ft, -135_deg}, true, Path::Parameters{}}});

    turn->toward(0_deg);

    // ring 13

    intake->intake();

    pathFollower->follow({{{2_ft, 5.5_ft, 0_deg}, false, Path::Parameters{}}});

    turn->toward(90_deg);

    // ring 14

    pathFollower->follow({{{5_ft, 5_ft, 90_deg}, false, Path::Parameters{}}});

    intake->stop();

    turn->awayFrom(45_deg);

    pathFollower->follow(
        {{{5.8_ft, 5.8_ft, 45_deg}, true, Path::Parameters{}}});

    goalClamp->unclamp();

    wait(100_ms);

    // ring 15

    intake->intake();

    pathFollower->follow({{{4_ft, 2_ft, -135_deg}, false, Path::Parameters{}}});

    intake->stop();

    turn->awayFrom(180_deg);

    pathFollower->follow({{{4_ft, 0_ft, 189_deg}, true, Path::Parameters{}}});

    goalClamp->clamp();

    wait(100_ms);

    // ring 16

    intake->intake();

    turn->toward(90_deg);

    pathFollower->follow({{{5_ft, 0_ft, 90_deg}, false, Path::Parameters{}}});

    intake->stop();

    pathFollower->follow({{{4_ft, 0_ft, -90_deg}, true, Path::Parameters{}}});

    turn->toward(180_deg);

    // ring 17

    intake->intake();

    pathFollower->follow({{{4_ft, 2_ft, 180_deg}, false, Path::Parameters{}}});

    // ring 18

    pathFollower->follow({{{4_ft, 3_ft, 180_deg}, false, Path::Parameters{}}});

    turn->toward(45_deg);

    pathFollower->follow({{{5.8_ft, 0_ft, 0_deg}, false, Path::Parameters{}}});

    // ring 19

    // ring 20

    pathFollower->follow({{{5.5_ft, 5_ft, 0_deg}, false, Path::Parameters{}}});

    intake->stop();

    goalClamp->unclamp();

    wait(100_ms);

    turn->awayFrom(-135_deg);

    // change to half speed

    ladybrown->prepare();

    pathFollower->follow({{{1_ft, 1_ft, -135_deg}, true, Path::Parameters{}}});

    ladybrown->fullyExtend();

    pathFollower->follow(
        {{{1.5_ft, 1.5_ft, -135_deg}, true, Path::Parameters{}}});
*/
  } else if(id == ID24) {
    setupRoutine({-5_ft, 0_ft, 90_deg});

    intake->index();
    pathFollower->follow(
        {{{-4_ft, 0_ft, 90_deg}, false, Path::Parameters{0.1}}});

    pathFollower->follow(
        {{{-5_ft, 0_ft, 90_deg}, true, Path::Parameters{0.1}}});
    // LADY BROWN
    intake->intake();

    wait(500_ms);

    turn->toward({-4_ft, -4_ft});
    pathFollower->follow(
        {{{-3.75_ft, -4_ft, 165_deg}, false, Path::Parameters{0.1}}});
    intake->index();
    turn->toward({-6_ft, -4_ft});

    pathFollower->follow(
        {{{-5_ft, -4_ft, 180_deg}, false, Path::Parameters{0.1}}});

    pathFollower->follow(
        {{{-2_ft, -4_ft, 90_deg}, true, Path::Parameters{0.01}}});
    /**pathFollower->follow({{{-4_ft, -4_ft, 155_deg}, false,
     Path::Parameters{}}}); turn->awayFrom(90_deg);
     pathFollower->follow({{{-2_ft, -4_ft, 90_deg}, true, Path::Parameters{}}});
     // Clamp mogo
     pathFollower->follow({{{-6_ft, -6_ft, -135_deg}, false,
     Path::Parameters{}}}); pathFollower->follow(
         {{{-0.5_ft, -0.5_ft, 90_deg}, false, Path::Parameters{}}});
     pathFollower->follow({{{0.5_ft, 0.5_ft, 90_deg}, false,
     Path::Parameters{}}}); pathFollower->follow({{{-6_ft, -6_ft, -135_deg},
     true, Path::Parameters{}}});
     // Unclamp mogo
     pathFollower->follow({{{0_ft, -5_ft, 90_deg}, false, Path::Parameters{}}});
     pathFollower->follow({{{0_ft, -4_ft, 45_deg}, false, Path::Parameters{}}});
     pathFollower->follow({{{0_ft, -5.5_ft, 180_deg}, false,
     Path::Parameters{}}});
     // LADY BROWN 2 RINGS
     pathFollower->follow({{{.5_ft, -5_ft, 90_deg}, false,
     Path::Parameters{}}}); pathFollower->follow({{{0_ft, -5.5_ft, 180_deg},
     false, Path::Parameters{}}});
     // LADY BROWN 1 MORE RING
     pathFollower->follow({{{2_ft, -2_ft, 45_deg}, true, Path::Parameters{}}});
     // CLAMP MOGO
     pathFollower->follow({{{2_ft, -4_ft, 180_deg}, false,
     Path::Parameters{}}});
     // INTAKE ONLY RED RING
     pathFollower->follow({{{4_ft, -4_ft, 90_deg}, false, Path::Parameters{}}});
     // INTAKE ONLY RED RING
     pathFollower->follow({{{6_ft, -6_ft, 135_deg}, false,
     Path::Parameters{}}});
     // INTAKE ONLY RED RING
     pathFollower->follow({{{2_ft, -4_ft, -45_deg}, false,
     Path::Parameters{}}}); pathFollower->follow({{{4.5_ft, -4_ft, 90_deg},
     false, Path::Parameters{}}}); pathFollower->follow({{{6_ft, -6_ft,
     135_deg}, false, Path::Parameters{}}}); turn->toward(-45_deg);
    // RELEASE GOAL CLAMP

    // RAISE HANG MECH
    pathFollower->follow(
        {{{.5_ft, -.5_ft, -45_deg}, true, Path::Parameters{}}});
    // HANG */
  }

  END_ROUTINE

  /*
    ___ _ _  _ _     _       _
   |_  ) | |( | )   /_\ _  _| |_ ___
    / /|_  _|V V   / _ \ || |  _/ _ \
   /___| |_|      /_/ \_\_,_|\__\___/

  */
  START_ROUTINE("24\" Auto")
  setupRoutine({-2_tile + 3.75_in, -0.5_tile, 90_deg});

  // Goal rush middle goal.
  intake->index();
  goalRush->extendArm();
  pathFollower->follow(
      {{{-1.375_tile, -0.55_tile, 90_deg, 76.5_in_per_s},
        false,
        Path::Parameters{0.5}},
       {{-0.5_tile, -0.5_tile, 65_deg},
        false,
        Path::Parameters{0.9, 76.5_in_per_s, 38.25_in_per_s_sq}}});
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    turn->toward(20_deg);
  }
  goalRush->grab();
  wait(0.25_s); // Time for goal rush mech to clamp.
  turn->toward(45_deg);
  pathFollower->follow(
      {{{-1.1_tile, -1.1_tile, 45_deg},
        true,
        Path::Parameters{1, 76.5_in_per_s, 76.5_in_per_s_sq}}});
  if(GUI::Routines::selectedColor() == MatchColor::Blue) {
    turn->toward(85_deg);
  } else {
    turn->toward(-5_deg);
  }
  goalRush->release();
  wait(0.25_s); // Time for goal rush mech to un-clamp.

  // Goal rush side goal.
  turn->toward(135_deg);
  pathFollower->follow(
      {{{-0.5_tile, -1.5_tile, 135_deg},
        false,
        Path::Parameters{0.75, 76.5_in_per_s, 76.5_in_per_s_sq}}});
  if(GUI::Routines::selectedColor() == MatchColor::Blue) {
    turn->toward(160_deg);
  } else {
    turn->toward(110_deg);
  }
  goalRush->grab();
  wait(0.25_s); // Time for goal rush mech to clamp.
  turn->toward(135_deg);
  pathFollower->follow(
      {{{-1.1_tile, -1.1_tile, 135_deg},
        true,
        Path::Parameters{0.75, 76.5_in_per_s, 76.5_in_per_s_sq}}});
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    turn->toward(70_deg);
  }
  goalRush->release();
  wait(0.25_s); // Time for goal rush mech to un-clamp.

  // Clamp on to goal.
  turn->toward(-135_deg);
  goalRush->retractArm();
  pathFollower->follow(
      {{{-0.5_tile, -0.5_tile, -135_deg}, true, Path::Parameters{0.25}}});
  goalClamp->clamp();
  wait(0.1_s); // Time for goal clamp mech to clamp.
  pathFollower->follow(
      {{{-1_tile, -1_tile, -135_deg}, false, Path::Parameters{0.25}}});

  // Score first stack of rings.
  intake->intake();
  turn->toward({-1_tile, -2.5_tile});
  pathFollower->follow(
      {{{-1_tile,
         -2.5_tile,
         (GUI::Routines::selectedColor() == MatchColor::Blue ? -1 : 1) *
             angle(drive->getPose(), {-1_tile, -2.5_tile}, true)},
        false,
        Path::Parameters{2, 35_in_per_s}}});

  // Score second stack of rings.
  turn->toward({-2_tile, -2_tile});
  pathFollower->follow(
      {{{-2_tile,
         -2_tile,
         (GUI::Routines::selectedColor() == MatchColor::Blue ? -1 : 1) *
             angle(drive->getPose(), {-2_tile, -2_tile}, true)},
        false,
        Path::Parameters{0.1}}});

  // Score the rings in the corner.
  turn->toward(-135_deg);
  intake->resetCount();
  while(matchTimer.timeElapsed() <= 24_s) {
    turn->toward(-135_deg);
    pathFollower->follow({{{-2.575_tile, -2.575_tile, -135_deg},
                           false,
                           Path::Parameters{0.375, 20_in_per_s}}});
    pathFollower->follow(
        {{{-2_tile, -2_tile, -135_deg}, true, Path::Parameters{0.375}}});
    wait(0.15_s);
  }

  // Place goal in corner and go touch ladder.
  turn->toward(45_deg);
  pathFollower->follow(
      {{{-2.5_tile, -2.5_tile, 45_deg}, true, Path::Parameters{0.15}}});
  intake->stop();
  goalClamp->unclamp();
  pathFollower->follow(
      {{{-1.375_tile, 0_tile, 0_deg}, false, Path::Parameters{3}}});
  END_ROUTINE

  /*
      _ ___ _ _     _       _
     / | __( | )   /_\ _  _| |_ ___
     | |__ \V V   / _ \ || |  _/ _ \
     |_|___/     /_/ \_\_,_|\__\___/

  */
  START_ROUTINE("15\" Auto") setupRoutine({-2_tile - 4_in, 1.5_tile, 90_deg});

  // Goal rush side goal.
  intake->index();
  goalRush->extendArm();
  pathFollower->follow(
      {{{-0.5_tile, 1.5_tile, 65_deg}, false, Path::Parameters{1}}});
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    turn->toward(27.5_deg);
  }
  goalRush->grab();
  wait(0.25_s); // Time for goal rush mech to clamp.

  // Clamp on to goal.
  turn->toward(45_deg);
  pathFollower->follow(
      {{{-1.1_tile, 0.9_tile, 45_deg}, true, Path::Parameters{0.5}}});
  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    turn->toward(0_deg);
  } else {
    turn->toward(90_deg);
  }
  goalRush->release();
  wait(0.25_s); // Time for goal rush mech to un-clamp.
  turn->toward(-135_deg);
  goalRush->retractArm();
  pathFollower->follow(
      {{{-0.4_tile, 1.6_tile, -135_deg}, true, Path::Parameters{0.5}}});
  goalClamp->clamp();
  wait(0.1_s); // Time for goal clamp mech to clamp.
  pathFollower->follow(
      {{{-0.5_tile, 1.5_tile, -135_deg}, false, Path::Parameters{0.1}}});

  // Score first stack of rings.
  intake->intake();
  turn->toward({-1.5_tile, 2.5_tile});
  pathFollower->follow(
      {{{-1.5_tile,
         2.5_tile,
         (GUI::Routines::selectedColor() == MatchColor::Blue ? -1 : 1) *
             angle(drive->getPose(), {-1.5_tile, 2.5_tile}, true)},
        false,
        Path::Parameters{0.5}}});

  // Score second stack of rings.
  turn->toward({-2_tile, 2_tile});
  pathFollower->follow(
      {{{-2_tile,
         2_tile,
         (GUI::Routines::selectedColor() == MatchColor::Blue ? -1 : 1) *
             angle(drive->getPose(), {-2_tile, 2_tile}, true)},
        false,
        Path::Parameters{0.5}}});

  // Score the rings in the corner.
  turn->toward(-45_deg);
  intake->resetCount();
  while(matchTimer.timeElapsed() <= 20_s) {
    turn->toward(-45_deg);
    pathFollower->follow({{{-2.575_tile, 2.575_tile, -45_deg},
                           false,
                           Path::Parameters{0.375, 20_in_per_s}}});
    pathFollower->follow(
        {{{-2_tile, 2_tile, -45_deg}, true, Path::Parameters{0.375}}});
    wait(0.15_s);
  }

  // Get other ring.
  turn->toward(180_deg);
  pathFollower->follow({{{-2.4_tile, 0.5_tile, 180_deg},
                         false,
                         Path::Parameters{2, 30_in_per_s}}});
  turn->toward(180_deg);
  intake->index();
  goalClamp->unclamp();
  pathFollower->follow({{{-2.4_tile, -0.5_tile, 180_deg},
                         false,
                         Path::Parameters{0.5, 30_in_per_s}}});

  // Clamp on to other goal.
  turn->awayFrom({-1.5_tile, 0_tile});
  pathFollower->follow(
      {{{-2_tile,
         0_tile,
             (GUI::Routines::selectedColor() == MatchColor::Blue ? -1 : 1) *
                 angle(drive->getPose(), {-2_tile, 0_tile}, true)},
        true,
        Path::Parameters{0.1}}});
  goalClamp->clamp();
  wait(0.1_s); // Time for goal clamp mech to clamp.
  intake->intake();
  wait(1_s);
  END_ROUTINE
}

void RobotClone::setupRoutine(Pose startingPose) {
  matchTimer.setTime();

  const bool flipped{GUI::Routines::selectedColor() == MatchColor::Blue};
  if(flipped) {
    startingPose.flip();
  }
  pathFollower->setFlipped(flipped);
  turn->setFlipped(flipped);

  drive->setPose(startingPose);
  gps->setPose(startingPose);

  if(GUI::Routines::selectedColor() == MatchColor::Red) {
    intake->setSortOutColor(ColorSensor::Color::Blue);
  } else {
    intake->setSortOutColor(ColorSensor::Color::Red);
  }

  goalClamp->unclamp();

  if(id == ID15) {
    intake->outtake();
    wait(0.1_s);
    intake->stop();
  }
}

Schedule RobotClone::clampWhenReady(const second_t timeout) {
  return Schedule{Schedule::Item{"Clamp When Ready",
                                 [=]() {
                                   pathFollower->interrupt();
                                   turn->interrupt();
                                   return goalClamp->hasGoal();
                                 },
                                 [=]() { goalClamp->clamp(); },
                                 timeout}};
}
} // namespace atum

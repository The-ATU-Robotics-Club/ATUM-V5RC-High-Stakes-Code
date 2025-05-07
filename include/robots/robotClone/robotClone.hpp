/**
 * @file robotClone.hpp
 * @brief Includes the RobotClone class.
 * @date 2025-01-09
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "../sharedSystems/goalClamp.hpp"
#include "../sharedSystems/goalRush.hpp"
#include "../sharedSystems/intake.hpp"
#include "../sharedSystems/ladybrown.hpp"
#include "../sharedSystems/poseEstimator.hpp"


namespace atum {
/**
 * @brief This encapsulates all of the behaviors related to the clone bots,
 * differing only in setup where ports (and some parameters) may be changed.
 *
 */
class RobotClone : public Robot {
  ROBOT_BOILERPLATE(); // Included in all task derivatives for setup.

  public:
  /**
   * @brief Constants storing the brain IDs for the 15" and 24" bots.
   *
   */
  static constexpr int ID15{0x60539e00};
  static constexpr int ID24{0x64899e00};

  /**
   * @brief Setups the robot, changing the setup depending on what ID is
   * provided.
   *
   * @param iID
   */
  RobotClone(const int iID);

  /**
   * @brief The behavior of the clone bots when disabled.
   *
   */
  void disabled() override;

  /**
   * @brief The behavior of the clone bots when in operator control.
   *
   */
  void opcontrol() override;

  private:
  // Setup helpers.
  /**
   * @brief Sets up the drive.
   *
   */
  void driveSetup();

  /**
   * @brief Sets up the ladybrown.
   *
   */
  void ladybrownSetup();

  /**
   * @brief Sets up the intake.
   *
   */
  void intakeSetup();

  /**
   * @brief Sets up the goal clamp and goal rush.
   *
   */
  void goalSetup();

  /**
   * @brief Sets up the objects for autonomous routine usage.
   *
   */
  void autonSetup();

  // Opcontrol helpers.
  /**
   * @brief Deals with printing to the remote and setting the LEDs in accordance
   * with the clamp.
   *
   */
  void visualFeedback();

  /**
   * @brief Deals with controls in manual mode.
   *
   */
  void manualControls();

  /**
   * @brief Deals with controls in ladybrown mode.
   *
   */
  void ladybrownControls();

  /**
   * @brief Deals with controls in intake mode.
   *
   */
  void intakeControls();

  /**
   * @brief Deals with controls in hang mode.
   *
   */
  void hangControls();

  /**
   * @brief Deals with the various shift keys and buttons to change the state of
   * the robot (going to ladybrown/intake/manual mode, changing brake mode,
   * etcetera).
   *
   */
  void configurationControls();

  // Autonomous helpers.
  /**
   * @brief Contains the behavior for the 15" robot during programming skills.
   *
   */
  void skills15();

  /**
   * @brief Contains the behavior for the 24" robot during programming skills.
   *
   */
  void skills24();

  /**
   * @brief Finishes all positive routines and sends the robot to the given
   * position.
   *
   * @param endPosition
   */
  void endOfPositive(const Pose &endPosition);

  /**
   * @brief Collects the two stacks and the rings in the corner on the negative
   * side. Indexes the last ring of the negative side stack.
   *
   */
  void collectNegative();

  /**
   * @brief Places an indexed ring on our alliance stake coming from the
   * negative side.
   *
   */
  void allianceStake();

  /**
   * @brief Collects the rings near the middle and grabs the goal. Go to start
   * will have the robot move to the position next to the middle ring stack.
   *
   * The other booleans refer to what the robot will try to grab. 
   *
   * @param negative
   */
  void collectMiddle(const bool negative,
                     const bool goToStart,
                     const bool otherPreload,
                     const bool startPreload);

  /**
   * @brief Rushes a goal. The first boolean changes if the robot is rushing
   * left or right. The second boolean changes if the robot will clamp as soon
   * as it detects a goal.
   *
   * @param left
   * @param clampImmediately
   */
  void rush(const bool left, const bool clampImmediately);

  /**
   * @brief Collects the rings in a corner. Changes y-values if the negative
   * corner.
   *
   * @param negative
   * @param pushes
   * @param shouldIndexLastRing
   */
  void collectCorner(const bool negative,
                     const int pushes,
                     const bool shouldIndexLastRing);

  /**
   * @brief Clamps down whenever a goal is aligned. If the given timeout is
   * exceeded before a goal is found, does nothing (so you should place a manual
   * clamp at the point you expect the goal to be there).
   *
   * @param timeout
   */
  void clampWhenReady(const second_t timeout = 5_s);

  /**
   * @brief Gets the position a given offset in front of robot.
   *
   * @param offset
   * @return Pose
   */
  Pose getInFrontOf(const meter_t offset) const;

  /**
   * @brief Gets a pose a certain distance past a target, linear with the
   * robot.
   *
   * @param target
   * @param past
   * @return Pose
   */
  Pose getPast(const Pose &target, const meter_t past) const;

  /**
   * @brief Sets the sort out color of the intake to the opposite color
   * selected.
   *
   */
  void setSortToOpposite();

  /**
   * @brief Sets up the starting position of the robot based on if we're
   * starting on the negative side and if which way we're rushing.
   *
   * @param negative
   * @param rushingLeft
   */
  void rushSetup(const bool negative, const bool rushingLeft);

  /**
   * @brief Sets up the robot with the appropriate starting pose and flips poses
   * if necessary.
   *
   * @param startingPose
   */
  void setupRoutine(Pose startingPose);

  int id;
  Remote remote;
  std::unique_ptr<Drive> drive;
  std::unique_ptr<Intake> intake;
  std::unique_ptr<Ladybrown> ladybrown;
  std::unique_ptr<GoalClamp> goalClamp;
  std::unique_ptr<Piston> goalRushL;
  std::unique_ptr<Piston> goalRushR;
  std::unique_ptr<Piston> kaboomer;
  const inch_t extensionDistance{8.5_in};
  Scheduler scheduler;
  std::unique_ptr<MoveTo> moveToClose;
  std::unique_ptr<MoveTo> moveToFar;
  std::unique_ptr<MoveTo> moveToRush;
  std::unique_ptr<Turn> turn;
  std::unique_ptr<PathFollower> pathFollower;
  Timer matchTimer;
  bool useManualControls{false};
  bool useLadybrownControls{false};
  bool useHangControls{false};
  bool scored{false};
  Timer clampTimer{0.5_s};

  /**
   * @brief Initializes the ports for the 15 inch.
   *
   */
  void initialize15Ports();

  /**
   * @brief Initializes the ports for the 24 inch.
   *
   */
  void initialize24Ports();

  std::unordered_map<std::string, MotorPortsList> motorPorts;
  std::unordered_map<std::string, uint8_t> otherPorts;
};
} // namespace atum
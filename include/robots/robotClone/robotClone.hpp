/**
 * @file robotClone.hpp
 * @brief Includes the RobotClone class.
 * @date 2025-01-09
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "../sharedSystems/intake.hpp"
#include "../sharedSystems/ladybrown.hpp"
#include "atum/atum.hpp"
#include "atum/depend/units.h"

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
  static constexpr int ID15{0x64824900};
  static constexpr int ID24{0x00000000};

  /**
   * @brief Setups the robot, changing the setup depending on what ID is
   * provided.
   *
   * @param iID
   */
  RobotClone(const int iID);

  /**
   * @brief The behavior of the clone bots when disabled. Also starts the
   * background tasks for the intake and the ladybrown.
   *
   */
  void disabled() override;

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

  // Opcontrol helpers.
  /**
   * @brief Deals with printing to the remote.
   *
   */
  void remotePrinting();

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
   * @brief Deals with the various shift keys and buttons to change the state of
   * the robot (going to ladybrown/intake/manual mode, changing brake mode,
   * etcetera).
   *
   */
  void configurationControls();

  const int id;
  Remote remote;
  std::unique_ptr<Drive> drive;
  std::unique_ptr<Intake> intake;
  std::unique_ptr<Ladybrown> ladybrown;
  Piston goalClamp{'A'};
  std::unique_ptr<Odometry> odometry;
  bool useManualControls{false};
  bool useLadybrownControls{false};
};
} // namespace atum
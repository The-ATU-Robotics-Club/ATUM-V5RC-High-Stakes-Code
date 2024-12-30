#pragma once

#include "../sharedSystems/intake.hpp"
#include "../sharedSystems/ladybrown.hpp"
#include "atum/atum.hpp"

namespace atum {
class Robot15A : public Robot {
  ROBOT_BOILERPLATE();

  public:
  static constexpr int ID{0x64824900};

  Robot15A();

  void disabled() override;

  void opcontrol() override;

  private:
  void remotePrinting();
  void manualControls();
  void ladybrownControls();
  void intakeControls();
  void configurationControls();

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
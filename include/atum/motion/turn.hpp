#pragma once

#include "../systems/drive.hpp"
#include "profileFollower.hpp"

namespace atum {
class Turn {
  public:
  Turn(Drive *iDrive,
       std::unique_ptr<AngularProfileFollower> iFollower,
       const Logger::Level loggerLevel = Logger::Level::Info);

  void toward(const Pose &target,
              const AngularProfile::Parameters &specialParams = {});

  void toward(const degree_t target,
              const AngularProfile::Parameters &specialParams = {});

  void awayFrom(const Pose &target,
                const AngularProfile::Parameters &specialParams = {});

  void awayFrom(const degree_t target,
                const AngularProfile::Parameters &specialParams = {});

  void interrupt();

  private:
  Drive *drive;
  std::unique_ptr<AngularProfileFollower> follower;
  Logger logger;
  bool interrupted{false};
};
} // namespace atum
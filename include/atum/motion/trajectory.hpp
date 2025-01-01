#pragma once

#include "../pose/pose.hpp"

namespace atum {
class Trajectory {
  public:
  struct Parameters {
    Parameters &operator=(const Parameters &other);
    double curviness{0.0};
    meters_per_second_t maxV{0_mps};
    meters_per_second_squared_t maxA{0_mps_sq};
    meter_t track{0_m};
    meter_t spacing{1_in};
    double binarySearchScaling{0.75};
  };

  struct TimedPose {
    Pose pose;
    second_t t{0_s};
  };

  Trajectory(const std::pair<Pose, Pose> &waypoints,
             const std::optional<Parameters> &specialParams = {});

  static void setDefaultParameters(const Parameters &newParams);

  private:
  void generate();

  Pose getPoint(const double t) const;
  double getCurvature(const double t) const;
  Pose getDerivative(const double t) const;
  Pose get2ndDerivative(const double t) const;

  static Parameters defaultParams;
  std::vector<TimedPose> trajectory;
  Parameters params;
  TimedPose start;
  Pose startDirection;
  TimedPose end;
  Pose endDirection;
};
} // namespace atum
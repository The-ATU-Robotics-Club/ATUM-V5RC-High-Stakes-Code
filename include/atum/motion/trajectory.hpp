#pragma once

#include "../pose/pose.hpp"
#include "../utility/logger.hpp"

/**
 * @brief This template specialization is to allow the use of second_t as a key
 * in maps.
 *
 * @tparam
 */
template <>
struct std::hash<second_t> {
  std::size_t operator()(const second_t &k) const {
    return std::hash<double>()(atum::getValueAs<second_t>(k));
  }
};

namespace atum {
class Trajectory {
  public:
  struct Parameters {
    Parameters(double iCurviness = 0.0,
               meters_per_second_t iMaxV = 0_mps,
               meters_per_second_squared_t iMaxA = 0_mps_sq,
               meter_t iTrack = 0_m,
               meter_t iSpacing = 1_in,
               meter_t iMaxSpacingError = 0.1_in,
               double iBinarySearchScaling = 0.75);
    Parameters(const Parameters &other);
    Parameters &operator=(const Parameters &other);
    double curviness{0.0};
    meters_per_second_t maxV{0_mps};
    meters_per_second_squared_t maxA{0_mps_sq};
    meter_t track{0_m};
    meter_t halfTrack{0_m};
    meter_t spacing{1_in};
    meter_t maxSpacingError{0.1_in};
    double binarySearchScaling{0.75};
  };

  Trajectory(const std::pair<Pose, Pose> &waypoints,
             const std::optional<Parameters> &specialParams = {},
             const Logger::Level loggerLevel = Logger::Level::Debug);

  second_t getTotalTime();

  static void setDefaultParameters(const Parameters &newParams);

  private:
  void generate();
  void parameterize();
  double addNextPoint(double t0);
  Pose getPoint(const double t) const;
  degree_t getHeading(const Pose &deriv) const;
  double getCurvature(const double t, const UnwrappedPose &deriv) const;
  Pose getDerivative(const double t) const;
  Pose get2ndDerivative(const double t) const;
  void graphTrajectory();
  void prepareGraph();

  static Parameters defaultParams;
  Pose start;
  Pose startDirection;
  Pose end;
  Pose endDirection;
  Parameters params;
  Logger logger;
  std::unordered_map<second_t, Pose> trajectory{};
  second_t totalTime{0_s};
  std::vector<Pose> points;
  std::vector<double> curvatures;
};
} // namespace atum
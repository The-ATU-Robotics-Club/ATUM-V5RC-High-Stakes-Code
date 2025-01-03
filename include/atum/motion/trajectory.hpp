#pragma once

#include "../pose/pose.hpp"
#include "../time/timer.hpp"
#include "../utility/logger.hpp"
#include <map>

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
               const meters_per_second_t iMaxV = 0_mps,
               const meters_per_second_squared_t iMaxA = 0_mps_sq,
               const meter_t iTrack = 0_m,
               const meter_t iSpacing = 1_in,
               const meter_t iMaxSpacingError = 0.1_in,
               const bool iUsePosition = true,
               const double iBinarySearchScaling = 0.75);
    Parameters(
        const meters_per_second_t iMaxV, // No default value to disambiguate
                                         // default constructor.
        const double iCurviness = 0.0,
        const meters_per_second_squared_t iMaxA = 0_mps_sq,
        const meter_t iTrack = 0_m,
        const meter_t iSpacing = 1_in,
        const meter_t iMaxSpacingError = 0.1_in,
        const bool iUsePosition = true,
        const double iBinarySearchScaling = 0.75);
    Parameters(const Parameters &other);
    Parameters &operator=(const Parameters &other);
    // Decides how "curvy" the path is. Reasonable values from 3 to 15.
    double curviness{0.0};
    meters_per_second_t maxV{0_mps};
    meters_per_second_squared_t maxA{0_mps_sq};
    meter_t track{0_m};
    meter_t spacing{1_in};
    meter_t maxSpacingError{0.1_in};
    bool usePosition{true};
    double binarySearchScaling{0.75};
  };

  Trajectory(const std::pair<Pose, Pose> &waypoints,
             const std::optional<Parameters> &specialParams = {},
             const Logger::Level loggerLevel = Logger::Level::Debug);

  Pose getPose(const Pose &state);

  second_t getTotalTime();

  Parameters getParams() const;

  static void setDefaultParams(const Parameters &newParams);

  private:
  Pose getClosest(const Pose &state);
  Pose getTimed();
  void generate();
  void parameterize();
  double addNextPoint(double t0);
  Pose getPoint(const double t) const;
  degree_t getHeading(const Pose &deriv) const;
  double getCurvature(const double t, const UnwrappedPose &deriv) const;
  Pose getDerivative(const double t) const;
  Pose get2ndDerivative(const double t) const;
  void graphTrajectory();

  static Parameters defaultParams;
  Pose start;
  Pose startDirection;
  Pose end;
  Pose endDirection;
  Parameters params;
  Logger logger;
  std::map<second_t, Pose> trajectory;
  std::vector<second_t> times;
  second_t totalTime{0_s};
  std::vector<Pose> points;
  std::vector<double> curvatures;
  int closestIndex{0};
  Timer timer;
};
} // namespace atum


#pragma once

#include "../utility/logger.hpp"
#include "kinematics.hpp"
#include <limits>

namespace atum {
template <typename Unit>
class MotionProfile {
  public:
  using UnitsPerSecond = decltype(Unit{1} / 1_s);
  using UnitsPerSecondSq = decltype(UnitsPerSecond{1} / 1_s);
  using UnitsPerSecondCb = decltype(UnitsPerSecondSq{1} / 1_s);
  using UnitKinematics = Kinematics<Unit>;

  struct Parameters {
    UnitsPerSecond maxV;
    UnitsPerSecondSq maxA;
    // Default to a trapezoidal profile.
    UnitsPerSecondCb maxJ{std::numeric_limits<double>::max()};
  };

  struct Point {
    Unit s;
    UnitsPerSecond v;
    UnitsPerSecondSq a;
    UnitsPerSecondCb j;
    second_t t;
  };

  MotionProfile(const Unit start,
                const Unit iEnd,
                const Parameters &iParams,
                const Logger::Level &loggerLevel = Logger::Level::Info) :
      end{iEnd},
      params{iParams},
      logger{loggerLevel} {
    points[0].s = start;
    const Unit target{end - start};
    beginProfile(abs(target));
    finishProfile(target);
  }

  Point getPoint(const second_t t) {
    if(t < 0_s) {
      return points[0];
    } else if(t >= points[6].t) {
      done = true;
      return {end,
              UnitsPerSecond{0},
              UnitsPerSecondSq{0},
              UnitsPerSecondCb{0},
              points[6].t};
    }
    int i{0};
    while(t >= points[i].t) {
      i++;
    }
    second_t dt{t};
    if(i > 0) {
      dt -= points[i - 1].t;
    }
    Point p;
    p.s = UnitKinematics::position(
        dt, points[i].s, points[i].v, points[i].a, points[i].j);
    p.v = UnitKinematics::velocity(dt, points[i].v, points[i].a, points[i].j);
    p.a = points[i].a + points[i].j * dt;
    p.j = points[i].j;
    p.t = t;
    return p;
  }

  bool isDone() const {
    return done;
  }

  private:
  void beginProfile(const Unit &target) {
    const bool reachesMaxAccel{params.maxA * params.maxA / params.maxJ <
                               params.maxV};
    second_t jerkT{params.maxA / params.maxJ};
    if(!reachesMaxAccel) {
      jerkT = second_t{sqrt(getValueAs<second_t>(jerkT))};
    }
    const Unit jerkDistance{params.maxJ * 2.0 * pow<3>(jerkT)};
    Unit fullDistance{jerkDistance};
    if(reachesMaxAccel) {
      const second_t constantAccelT =
          (params.maxV - params.maxJ * jerkT * jerkT) / params.maxA;
      fullDistance = 2.0 * params.maxV * jerkT +
                     params.maxJ * jerkT * jerkT * constantAccelT +
                     params.maxA * constantAccelT * constantAccelT;
    }
    if(target < jerkDistance) {
      profile4Stage(target);
    } else if(!reachesMaxAccel) {
      profile5Stage(target);
    } else if(target < fullDistance) {
      profile6Stage(target);
    } else {
      profileAllStages(target);
    }
  }

  void profile4Stage(const Unit &target) {
    logger.debug("Generated profile is 4 stages (trapezoidal).");
    points[0].t = points[2].t = points[4].t = points[6].t =
        second_t{cbrt(getValueAs<Unit>(target) /
                      getValueAs<UnitsPerSecondCb>(params.maxJ) / 2.0)};
    points[1].s = points[0].t * points[0].t * points[0].t * params.maxJ / 6.0;
    points[3].s = points[5].s = 0.5 * target - points[1].s;
    points[1].v = points[2].v = points[5].v = points[6].v =
        0.5 * params.maxJ * points[0].t * points[0].t;
    points[3].v = points[4].v = points[1].v * 2.0;
    points[1].a = points[2].a = params.maxJ * points[0].t;
    points[5].a = points[6].a = -points[1].a;
  }

  void profile5Stage(const Unit &target) {
    logger.debug("Generated profile is 5 stages.");
    points[0].t = points[2].t = points[4].t = points[6].t =
        sqrt(params.maxV / params.maxJ);
    points[3].t = (target - params.maxV * points[0].t * 2.0) / params.maxV;
    points[1].s = points[0].t * points[0].t * points[0].t * params.maxJ / 6.0;
    points[3].s = points[5].s = params.maxV * points[0].t - points[1].s;
    points[4].s = target - points[1].s * 2.0 - points[3].s * 2.0;
    points[1].v = points[2].v = points[5].v = points[6].v = params.maxV / 2.0;
    points[3].v = points[4].v = params.maxV;
    points[1].a = points[2].a = params.maxJ * points[0].t;
    points[5].a = points[6].a = -points[1].a;
  }

  void profile6Stage(const Unit &target) {
    logger.debug("Generated profile is 6 stages.");
    const UnitsPerSecond b{3.0 * params.maxA * params.maxA / params.maxJ};
    const Unit c{2.0 * params.maxA * params.maxA * params.maxA / params.maxJ /
                     params.maxJ -
                 target};
    const auto discriminant = sqrt(b * b - 4.0 * params.maxA * c);
    const second_t t{max((-b + discriminant) / (2.0 * params.maxA),
                         (-b - discriminant) / (2.0 * params.maxA))};
    points[0].t = points[2].t = points[4].t = points[6].t =
        params.maxA / params.maxJ;
    points[1].t = points[5].t = t;
    points[1].a = points[2].a = params.maxJ * points[0].t;
    points[5].a = points[6].a = -points[1].a;
    points[1].v = points[6].v = 0.5 * params.maxJ * points[0].t * points[0].t;
    points[2].v = points[5].v = points[1].v + params.maxA * points[1].t;
    points[3].v = points[4].v = points[2].v + points[2].a * points[2].t -
                                0.5 * params.maxJ * points[2].t * points[2].t;
    points[1].s = params.maxJ * points[0].t * points[0].t * points[0].t / 6.0;
    points[2].s = points[6].s = points[1].v * points[1].t +
                                0.5 * points[1].a * points[1].t * points[1].t;
    points[3].s = points[5].s =
        points[2].v * points[2].t +
        0.5 * points[2].a * points[2].t * points[2].t -
        params.maxJ * points[2].t * points[2].t * points[2].t / 6.0;
  }

  void profileAllStages(const Unit &target) {
    logger.debug("Generated profile is 7 (all) stages.");
    points[0].t = params.maxA / params.maxJ;
    points[1].s = params.maxJ * points[0].t * points[0].t * points[0].t / 6.0;
    points[1].v = 0.5 * params.maxJ * points[0].t * points[0].t;
    points[1].a = params.maxA;
    points[1].t = (params.maxV - points[1].v * 2.0) / params.maxA;
    points[2].s = points[1].v * points[1].t +
                  0.5 * points[1].a * points[1].t * points[1].t;
    points[2].v = points[1].v + points[1].a * points[1].t;
    points[2].a = params.maxA;
    points[2].t = points[0].t;
    points[3].s = points[2].v * points[2].t +
                  0.5 * points[2].a * points[2].t * points[2].t -
                  params.maxJ * points[2].t * points[2].t * points[2].t / 6.0;
    points[3].v = params.maxV;
    points[3].t = (target - 2.0 * (points[1].s + points[2].s + points[3].s)) /
                  params.maxV;
    points[4].s = points[3].v * points[3].t;
    points[4].v = params.maxV;
    points[4].t = points[2].t;
    points[5].s = points[3].s;
    points[5].v = points[2].v;
    points[5].a = -params.maxA;
    points[5].t = points[1].t;
    points[6].s = points[2].s;
    points[6].v = points[1].v;
    points[6].a = -params.maxA;
    points[6].t = points[0].t;
  }

  void finishProfile(const Unit &target) {
    points[0].j = points[6].j = params.maxJ;
    points[2].j = points[4].j = -params.maxJ;
    if(target < Unit{0.0}) {
      for(Point &p : points) {
        p.s *= -1;
        p.v *= -1;
        p.a *= -1;
        p.j *= -1;
      }
    }
    for(std::size_t i{1}; i < 7; i++) {
      points[i].t += points[i - 1].t;
      points[i].s += points[i - 1].s;
    }
  }

  private:
  const Unit end;
  Parameters params;
  std::array<Point, 7> points;
  bool done{false};
  Logger logger;
};

using LateralProfile = MotionProfile<meter_t>;
using AngularProfile = MotionProfile<radian_t>;
} // namespace atum
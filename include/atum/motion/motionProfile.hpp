

#pragma once

#include "../time/timer.hpp"
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
    std::size_t searchIterations{20};
  };

  struct Point {
    Unit s;
    UnitsPerSecond v;
    UnitsPerSecondSq a;
    UnitsPerSecondCb j;
    second_t t;
  };

  MotionProfile(const Unit iStart,
                const Unit iEnd,
                const Parameters &iParams,
                const Logger::Level &loggerLevel = Logger::Level::Info) :
      start{iStart},
      end{iEnd},
      target{end - start},
      params{iParams},
      logger{loggerLevel} {
    if(logger.getLevel() == Logger::Level::Debug) {
      prepareGraphing();
    }
    const Unit target{end - start};
    beginProfile();
    finishProfile();
    timer = Timer{};
  }

  Point getPoint(const Unit s) {
    Point point{getTimedPoint()};
    const Point closestPoint{getClosestPoint(s)};
    if(abs(closestPoint.v) > abs(point.v)) {
      timer.resetAlarm(closestPoint.t);
      point = closestPoint;
    }
    if(logger.getLevel() == Logger::Level::Debug) {
      graphPoint(closestPoint); // TODO: Change this!
    }
    return point;
  }

  Point getClosestPoint(const Unit s) {
    // Return appropriate value if behind start or in front of end.
    if(distance(s, start) > Unit{0}) {
      return {start,
              UnitsPerSecond{0},
              UnitsPerSecondSq{0},
              UnitsPerSecondCb{(target < Unit{0}) ? -params.maxJ : params.maxJ},
              0_s};
    } else if(distance(s, end) < Unit{0}) {
      return {end,
              UnitsPerSecond{0},
              UnitsPerSecondSq{0},
              UnitsPerSecondCb{0},
              points[6].t};
    }
    int i{0};
    while(distance(s, points[i].s) < Unit{0}) {
      i++;
    }
    second_t t0{0_s};
    second_t t2{points[6].t};
    for(std::size_t n{0}; n < params.searchIterations; n++) {
      const second_t t1{(t0 + t2) / 2.0};
      if(distance(s, getPointAt(t1).s) >= Unit{0}) {
        t2 = t1;
      } else {
        t0 = t1;
      }
    }
    const second_t t1{(t0 + t2) / 2.0};
    return getPointAt(t1);
  }

  Unit distance(const Unit left, const Unit right) {
    Unit diff{difference(right, left)};
    if(target < Unit{0}) {
      diff *= -1;
    }
    return diff;
  }

  Point getTimedPoint() {
    const second_t t{timer.timeElapsed()};
    return getPointAt(t);
  }

  Point getPointAt(const second_t t) {
    if(t < 0_s) {
      return {start,
              UnitsPerSecond{0},
              UnitsPerSecondSq{0},
              UnitsPerSecondCb{(target < Unit{0}) ? -params.maxJ : params.maxJ},
              0_s};
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
  void beginProfile() {
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
    const Unit absTarget{abs(target)};
    if(absTarget < jerkDistance) {
      profile4Stage();
    } else if(!reachesMaxAccel) {
      profile5Stage();
    } else if(absTarget < fullDistance) {
      profile6Stage();
    } else {
      profileAllStages();
    }
  }

  void profile4Stage() {
    logger.debug("Generated profile is 4 stages.");
    const Unit absTarget{abs(target)};
    points[0].t = points[2].t = points[4].t = points[6].t =
        second_t{cbrt(getValueAs<Unit>(absTarget) /
                      getValueAs<UnitsPerSecondCb>(params.maxJ) / 2.0)};
    points[1].s = points[0].t * points[0].t * points[0].t * params.maxJ / 6.0;
    points[3].s = points[5].s = 0.5 * absTarget - points[1].s;
    points[1].v = points[2].v = points[5].v = points[6].v =
        0.5 * params.maxJ * points[0].t * points[0].t;
    points[3].v = points[4].v = points[1].v * 2.0;
    points[1].a = points[2].a = params.maxJ * points[0].t;
    points[5].a = points[6].a = -points[1].a;
  }

  void profile5Stage() {
    logger.debug("Generated profile is 5 stages.");
    const Unit absTarget{abs(target)};
    points[0].t = points[2].t = points[4].t = points[6].t =
        sqrt(params.maxV / params.maxJ);
    points[3].t = (absTarget - params.maxV * points[0].t * 2.0) / params.maxV;
    points[1].s = points[0].t * points[0].t * points[0].t * params.maxJ / 6.0;
    points[3].s = points[5].s = params.maxV * points[0].t - points[1].s;
    points[4].s = absTarget - points[1].s * 2.0 - points[3].s * 2.0;
    points[1].v = points[2].v = points[5].v = points[6].v = params.maxV / 2.0;
    points[3].v = points[4].v = params.maxV;
    points[1].a = points[2].a = params.maxJ * points[0].t;
    points[5].a = points[6].a = -points[1].a;
  }

  void profile6Stage() {
    logger.debug("Generated profile is 6 stages.");
    const Unit absTarget{abs(target)};
    const UnitsPerSecond b{3.0 * params.maxA * params.maxA / params.maxJ};
    const Unit c{2.0 * params.maxA * params.maxA * params.maxA / params.maxJ /
                     params.maxJ -
                 absTarget};
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

  void profileAllStages() {
    logger.debug("Generated profile is 7 (all) stages.");
    const Unit absTarget{abs(target)};
    points[0].t = points[2].t = points[4].t = points[6].t =
        params.maxA / params.maxJ;
    points[1].s = params.maxJ * points[0].t * points[0].t * points[0].t / 6.0;
    points[1].v = points[6].v = 0.5 * params.maxJ * points[0].t * points[0].t;
    points[1].a = points[2].a = params.maxA;
    points[1].t = points[5].t = (params.maxV - points[1].v * 2.0) / params.maxA;
    points[2].s = points[6].s = points[1].v * points[1].t +
                                0.5 * points[1].a * points[1].t * points[1].t;
    points[2].v = points[5].v = points[1].v + points[1].a * points[1].t;
    points[3].s = points[5].s =
        points[2].v * points[2].t +
        0.5 * points[2].a * points[2].t * points[2].t -
        params.maxJ * points[2].t * points[2].t * points[2].t / 6.0;
    points[3].v = points[4].v = params.maxV;
    points[3].t =
        (absTarget - 2.0 * (points[1].s + points[2].s + points[3].s)) /
        params.maxV;
    points[4].s = points[3].v * points[3].t;
    points[5].a = points[6].a = -params.maxA;
  }

  void finishProfile() {
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
    points[0].s = start;
    for(std::size_t i{1}; i < 7; i++) {
      points[i].t += points[i - 1].t;
      points[i].s += points[i - 1].s;
    }
  }

  void prepareGraphing() {
    atum::GUI::Graph::clearAll();
    const double rawStart{getValueAs<Unit>(start)};
    const double rawEnd{getValueAs<Unit>(end)};
    atum::GUI::Graph::setSeriesRange(
        {std::min(rawStart, rawEnd), std::max(rawStart, rawEnd)},
        atum::GUI::SeriesColor::White);
    atum::GUI::Graph::setSeriesRange(getValueAs<UnitsPerSecond>(params.maxV),
                                     atum::GUI::SeriesColor::Magenta);
    atum::GUI::Graph::setSeriesRange(getValueAs<UnitsPerSecondSq>(params.maxA),
                                     atum::GUI::SeriesColor::Red);
    atum::GUI::Graph::setSeriesRange(getValueAs<UnitsPerSecondCb>(params.maxJ),
                                     atum::GUI::SeriesColor::Blue);
  }

  void graphPoint(const Point &p) {
    atum::GUI::Graph::addValue(atum::getValueAs<meter_t>(p.s),
                               atum::GUI::SeriesColor::White);
    atum::GUI::Graph::addValue(atum::getValueAs<UnitsPerSecond>(p.v),
                               atum::GUI::SeriesColor::Magenta);
    atum::GUI::Graph::addValue(atum::getValueAs<UnitsPerSecondSq>(p.a),
                               atum::GUI::SeriesColor::Red);
    atum::GUI::Graph::addValue(atum::getValueAs<UnitsPerSecondCb>(p.j),
                               atum::GUI::SeriesColor::Blue);
  }

  const Unit start;
  const Unit end;
  const Unit target;
  Parameters params;
  std::array<Point, 7> points;
  bool done{false};
  Logger logger;
  Timer timer;
};

using LateralProfile = MotionProfile<meter_t>;
using AngularProfile = MotionProfile<radian_t>;
} // namespace atum
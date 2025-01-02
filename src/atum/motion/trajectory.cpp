#include "trajectory.hpp"

namespace atum {
Trajectory::Parameters::Parameters(double iCurviness,
                                   meters_per_second_t iMaxV,
                                   meters_per_second_squared_t iMaxA,
                                   meter_t iTrack,
                                   meter_t iSpacing,
                                   meter_t iMaxSpacingError,
                                   double iBinarySearchScaling) :
    curviness{iCurviness},
    maxV{iMaxV},
    maxA{iMaxA},
    track{iTrack},
    spacing{iSpacing},
    maxSpacingError{iMaxSpacingError},
    binarySearchScaling{iBinarySearchScaling} {}

Trajectory::Parameters::Parameters(meters_per_second_t iMaxV,
                                   double iCurviness,
                                   meters_per_second_squared_t iMaxA,
                                   meter_t iTrack,
                                   meter_t iSpacing,
                                   meter_t iMaxSpacingError,
                                   double iBinarySearchScaling) :
    curviness{iCurviness},
    maxV{iMaxV},
    maxA{iMaxA},
    track{iTrack},
    spacing{iSpacing},
    maxSpacingError{iMaxSpacingError},
    binarySearchScaling{iBinarySearchScaling} {}

Trajectory::Parameters::Parameters(const Trajectory::Parameters &other) :
    spacing{other.spacing},
    binarySearchScaling{other.binarySearchScaling} {
  if(other.curviness) {
    curviness = other.curviness;
  }
  if(other.maxV) {
    maxV = other.maxV;
  }
  if(other.maxA) {
    maxA = other.maxA;
  }
  if(other.track) {
    track = other.track;
  }
}

Trajectory::Parameters &Trajectory::Parameters::
    operator=(const Trajectory::Parameters &other) {
  if(this == &other) {
    return *this;
  }
  if(other.curviness) {
    curviness = other.curviness;
  }
  if(other.maxV) {
    maxV = other.maxV;
  }
  if(other.maxA) {
    maxA = other.maxA;
  }
  if(other.track) {
    track = other.track;
  }
  spacing = other.spacing;
  binarySearchScaling = other.binarySearchScaling;
  return *this;
}

Trajectory::Trajectory(const std::pair<Pose, Pose> &waypoints,
                       const std::optional<Parameters> &specialParams,
                       const Logger::Level loggerLevel) :
    start{waypoints.first},
    end{waypoints.second},
    params{defaultParams},
    logger{loggerLevel} {
  if(specialParams.has_value()) {
    params = specialParams.value();
  }
  const degree_t startH{90_deg - start.h};
  startDirection =
      params.curviness * Pose{1_m * cos(startH), 1_m * sin(startH)};
  const degree_t endH{90_deg - end.h};
  endDirection = params.curviness * Pose{1_m * cos(endH), 1_m * sin(endH)};
  generate();
  logger.debug("Trajectory has been generated!");
}

second_t Trajectory::getTotalTime() {
  logger.debug("Trajectory total time is: " + to_string(totalTime));
  return totalTime;
}

Trajectory::Parameters Trajectory::getParams() const {
  return params;
}

void Trajectory::setDefaultParams(const Parameters &newParams) {
  defaultParams = newParams;
}

void Trajectory::generate() {
  points.push_back(start);
  curvatures.push_back(getCurvature(0, getDerivative(0)));
  double t0{0.0};
  while(distance(points.back(), end) >
        params.spacing + params.maxSpacingError) {
    t0 = addNextPoint(t0);
  }
  points.push_back(end);
  curvatures.push_back(getCurvature(1, getDerivative(1)));
  parameterize();
}

void Trajectory::parameterize() {
  for(int i{1}; i < points.size() - 1; i++) {
    const meters_per_second_squared_t twoA{2.0 * params.maxA};
    const meters_per_second_t accelerated{
        sqrt(points[i - 1].v * points[i - 1].v + twoA * params.spacing)};
    points[i].v = units::math::min(accelerated, points[i].v);
    const int j{points.size() - 1 - i};
    const meters_per_second_t decelerated{
        sqrt(points[j + 1].v * points[j + 1].v + twoA * params.spacing)};
    points[j].v = units::math::min(decelerated, points[j].v);
  }
  trajectory[0_s] = start;
  for(int i{1}; i < points.size(); i++) {
    points[i].w = radians_per_second_t{
        getValueAs<meters_per_second_t>(points[i].v) * curvatures[i]};
    const meters_per_second_t avgV{(points[i - 1].v + points[i].v) / 2.0};
    totalTime += params.spacing / avgV;
    trajectory[totalTime] = points[i];
  }
  graphTrajectory();
}

double Trajectory::addNextPoint(double t0) {
  double t2{1.0};
  double t1{t0 * params.binarySearchScaling +
            t2 * (1.0 - params.binarySearchScaling)};
  Pose p0{points.back()};
  Pose p1{getPoint(t1)};
  meter_t distanceToNext{distance(p0, p1)};
  while(abs(distanceToNext - params.spacing) > params.maxSpacingError) {
    if(distanceToNext > params.spacing) {
      t2 = t1;
    } else {
      t0 = t1;
    }
    t1 = t0 * params.binarySearchScaling +
         t2 * (1.0 - params.binarySearchScaling);
    p1 = getPoint(t1);
    distanceToNext = distance(p0, p1);
  }
  const Pose deriv{getDerivative(t1)};
  p1.h = getHeading(deriv);
  curvatures.push_back(getCurvature(t1, deriv));
  p1.v = units::math::min(
      params.maxV,
      params.maxV /
          (abs(curvatures.back() *
               getValueAs<meter_t>(params.track)))); // Maybe multiply by 2?
  points.push_back(p1);
  return t1;
}

Pose Trajectory::getPoint(const double t) const {
  const double t2{t * t};
  const double t3{t2 * t};
  const double threeT2{3.0 * t2};
  const double twoT3{2.0 * t3};
  return (twoT3 - threeT2 + 1.0) * start +
         (t3 - 2.0 * t2 + t) * startDirection + (-twoT3 + threeT2) * end +
         (t3 - t2) * endDirection;
}

degree_t Trajectory::getHeading(const Pose &deriv) const {
  return 90_deg - atan2(deriv.y, deriv.x);
}

double Trajectory::getCurvature(const double t,
                                const UnwrappedPose &deriv) const {
  const UnwrappedPose deriv2{get2ndDerivative(t)};
  const double cross{deriv.x * deriv2.y - deriv.y * deriv2.x};
  if(!cross) {
    return infinitesimal;
  }
  const double denom{pow(deriv.x * deriv.x + deriv.y * deriv.y, 1.5)};
  return -cross / denom;
}

Pose Trajectory::getDerivative(const double t) const {
  const double t2{t * t};
  const double threeT2{3 * t2};
  return (6.0 * t2 - 6.0 * t) * (start - end) +
         (threeT2 - 4.0 * t + 1.0) * startDirection +
         (threeT2 - 2.0 * t) * endDirection;
}

Pose Trajectory::get2ndDerivative(const double t) const {
  const double sixT{6.0 * t};
  return (12.0 * t - 6.0) * (start - end) + (sixT - 4.0) * startDirection +
         (sixT - 2.0) * endDirection;
}

void Trajectory::graphTrajectory() {
  if(logger.getLevel() != Logger::Level::Debug) {
    return;
  }
  prepareGraph();
  for(Pose &pose : points) {
    GUI::Map::addPosition(pose, GUI::SeriesColor::Red);
    GUI::Graph::addValue(getValueAs<meters_per_second_t>(pose.v),
                         GUI::SeriesColor::Magenta);
    GUI::Graph::addValue(getValueAs<radians_per_second_t>(pose.w),
                         GUI::SeriesColor::Cyan);
  }
}

void Trajectory::prepareGraph() {
  GUI::Map::clearSeries(GUI::SeriesColor::Red);
  GUI::Graph::clearSeries(GUI::SeriesColor::Magenta);
  GUI::Graph::clearSeries(GUI::SeriesColor::Cyan);
  GUI::Graph::setSeriesRange(getValueAs<meters_per_second_t>(params.maxV),
                             GUI::SeriesColor::Magenta);
  GUI::Graph::setSeriesRange(2.0 *
                                 getValueAs<meters_per_second_t>(params.maxV) /
                                 getValueAs<meter_t>(params.track),
                             GUI::SeriesColor::Cyan);
}

Trajectory::Parameters Trajectory::defaultParams{};
} // namespace atum
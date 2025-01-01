#include "trajectory.hpp"

namespace atum {
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
                       const std::optional<Parameters> &specialParams) :
    params{defaultParams},
    start{waypoints.first},
    end{waypoints.second} {
  if(specialParams.has_value()) {
    params = specialParams.value();
  }
}


  Pose Trajectory::getPoint(const double t) const {
    const double t2{t * t};
    const double t3{t2 * t};
    const double threeT2{3 * t2};
    const double twoT3{2 * t3};
  }

  double Trajectory::getCurvature(const double t) const {

  }

  Pose Trajectory::getDerivative(const double t) const {

  }

  Pose Trajectory::get2ndDerivative(const double t) const {

  }


void Trajectory::generate() {
    
}
} // namespace atum
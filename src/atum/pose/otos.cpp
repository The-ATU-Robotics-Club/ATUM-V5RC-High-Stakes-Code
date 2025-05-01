#include "otos.hpp"

namespace atum {
OTOS::OTOS(const std::int8_t port,
           const UnwrappedPose &offset,
           const Logger::Level loggerLevel) :
    Tracker{loggerLevel, GUI::SeriesColor::Yellow},
    otos{std::make_unique<SerialDevice>(port, OTOS::Response::Error)},
    logger{loggerLevel} {
  otos->msg({Command::Initialize});
  wait(500_ms);
  otos->msg({Command::Reset});
  PiecedData<OTOSData> offsetPieced;
  offsetPieced.value = {offset.x, offset.y, offset.h};
  otos->msg(Packet{Command::SetOffset,
                   std::vector<uint8_t>(std::begin(offsetPieced.bytes),
                                        std::end(offsetPieced.bytes))},
            2);
  otos->msg({Command::Calibrate});
  Timer timeout{calibrationTimeout};
  while(!timeout.goneOff() &&
        otos->msg({Command::IsCalibrating}).id == Response::Waiting) {
    wait();
  }
  otos->msg({Command::SelfTest});
}

Pose OTOS::update() {
  UnwrappedPose newPose{pose};
  auto pos = otos->msg(Packet{Command::GetPosition}, 14);
  if(pos.id == Response::Success) {
    PiecedData<float[3]> posPieced;
    for(int i{0}; i < pos.data.size(); i++) {
      posPieced.bytes[i] = pos.data[i];
    }
    newPose.x = posPieced.value[0];
    newPose.y = posPieced.value[1];
    newPose.h = posPieced.value[2];
  }
  auto vel = otos->msg(Packet{Command::GetVelocity}, 14);
  if(vel.id == Response::Success) {
    PiecedData<float[3]> velPieced;
    for(int i{0}; i < vel.data.size(); i++) {
      velPieced.bytes[i] = vel.data[i];
    }
    const double vx{2 * velPieced.value[0]};
    const double vy{2 * velPieced.value[1]};
    const double h{M_PI_2 - newPose.h};
    newPose.vf = vx * std::cos(h) + vy * std::sin(h);
    newPose.vs = -vx * std::sin(h) + vy * std::cos(h);
    newPose.omega = -velPieced.value[2];
  }
  pose = newPose;
  return getPose(); // Use getPose() for logging purposes.
}

void OTOS::setPose(const Pose &iPose) {
  const UnwrappedPose raw{iPose};
  PiecedData<OTOSData> rawPieced;
  rawPieced.value = {raw.x, raw.y, raw.h};
  otos->msg(Packet{Command::SetPosition,
                   std::vector<uint8_t>(std::begin(rawPieced.bytes),
                                        std::end(rawPieced.bytes))},
            2);
  Tracker::setPose(iPose);
}

bool OTOS::check() const {
  auto response = otos->msg({Command::SelfTest});
  return response.correct() && response.id == Response::Success;
}
} // namespace atum
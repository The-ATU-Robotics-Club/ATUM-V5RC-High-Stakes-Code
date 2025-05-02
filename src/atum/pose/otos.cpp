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
  auto vel = otos->msg(Packet{Command::GetVelocity}, 14);
  if(vel.id == Response::Success && vel.correct()) {
    PiecedData<float[3]> velPieced;
    for(int i{0}; i < vel.data.size(); i++) {
      velPieced.bytes[i] = vel.data[i];
    }
    newPose.vf = -0.97 * velPieced.value[0];
    newPose.vs = 0.97 * velPieced.value[1];
    newPose.omega = -0.9825 * velPieced.value[2];
  }
  pose = newPose;
  return getPose(); // Use getPose() for logging purposes.
}

void OTOS::setPose(const Pose &iPose) {
  const UnwrappedPose raw{iPose};
  PiecedData<OTOSData> rawPieced;
  rawPieced.value = {raw.x, raw.y, -raw.h};
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
#include "robot15A.hpp"

namespace atum {
ROUTINE_DEFINITIONS_FOR(Robot15A) {
  START_ROUTINE("Test 1")
  intake->load();
  wait(5_s);
  for(int i{0}; i < 2; i++) {
    waitUntil(intake->checkStateIs(IntakeState::FinishedLoading), 3_s);
    ladybrown->score();
    waitUntil(ladybrown->checkStateIs(LadybrownState::Idle), 3_s);
  }
  intake->stop();
  END_ROUTINE
  START_ROUTINE("Test 2")
  END_ROUTINE
  START_ROUTINE("Test 3")
  END_ROUTINE
}
} // namespace atum
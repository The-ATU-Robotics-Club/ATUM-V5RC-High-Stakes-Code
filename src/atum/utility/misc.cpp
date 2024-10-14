#include "misc.hpp"

namespace atum {
bool fileExists(const std::string &filename) {
  std::ifstream file{filename};
  return file.good();
}
} // namespace atum
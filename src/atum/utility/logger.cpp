#include "logger.hpp"

namespace atum {
Logger::Logger(Level iLevel) : level{iLevel} {
  // Denote new logging session.
  logMutex.take(10);
  if(!beganLogging) {
    beganLogging = true;
    std::fstream file{logFilename, std::fstream::app};
    file << '\n' << "~~~~~~~~~~~~ BEGIN LOG ~~~~~~~~~~~~\n";
  }
  logMutex.give();
}

void Logger::debug(const std::string &msg) {
  log("DEBUG", msg, Level::Debug);
}

void Logger::info(const std::string &msg) {
  if(alreadyLogged(msg)) return;
  log("INFO", msg, Level::Info);
}

void Logger::warn(const std::string &msg) {
  if(alreadyLogged(msg)) return;
  log("WARN", msg, Level::Warn);
}

void Logger::error(const std::string &msg) {
  GUI::errorScreen();
  if(alreadyLogged(msg)) return;
  log("ERROR", msg, Level::Error);
}

void Logger::log(const std::string &prefix,
                 const std::string &msg,
                 Level msgLevel) {
  logMutex.take(10);
  if(level < msgLevel) return;
  std::stringstream fmtMsg{};
  fmtMsg << std::setw(5) << prefix << std::setw(0) << ": " + msg << '\n';
  std::cout << fmtMsg.str();
  if(msgLevel <= Level::Info) {
    std::fstream file{logFilename, std::fstream::app};
    file << fmtMsg.str();
    GUI::writeToLog(fmtMsg.str());
  }
  logMutex.give();
}

bool Logger::alreadyLogged(const std::string &msg) {
  if(std::find(logs.begin(), logs.end(), msg) != logs.end()) return true;
  logs.push_back(msg);
  return false;
}

const std::string Logger::logFilename{"log.txt"};

bool Logger::beganLogging{false};

pros::Mutex Logger::logMutex{};
} // namespace atum
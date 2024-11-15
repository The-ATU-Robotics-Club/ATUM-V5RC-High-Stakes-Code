#include "logger.hpp"

namespace atum {
Logger::Logger(LoggerLevel iLevel) : level{iLevel} {}

void Logger::debug(const std::string &msg) {
  log("DEBUG", msg, LoggerLevel::Debug);
}

void Logger::info(const std::string &msg) {
  log("INFO", msg, LoggerLevel::Info);
}

void Logger::warn(const std::string &msg) {
  log("WARN", msg, LoggerLevel::Warn);
}

void Logger::error(const std::string &msg) {
  log("ERROR", msg, LoggerLevel::Error);
}

void Logger::log(const std::string &prefix,
                 const std::string &msg,
                 LoggerLevel msgLevel) {
  logMutex.take();
  if(level < msgLevel) return;
  std::stringstream fmtMsg{};
  fmtMsg << std::setw(5) << prefix << std::setw(0) << ": " + msg << '\n';
  std::cout << fmtMsg.str();
  std::fstream file{logFilename, std::fstream::app};
  file << fmtMsg.str();
  GUI::writeToLog(fmtMsg.str());
  logMutex.give();
}

const std::string Logger::logFilename{"log.txt"};

pros::Mutex Logger::logMutex{};
} // namespace atum
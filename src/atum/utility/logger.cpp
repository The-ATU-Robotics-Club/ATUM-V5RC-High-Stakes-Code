#include "logger.hpp"

namespace atum {
Logger::Logger(LoggerLevel iLevel) : level{iLevel} {}

void Logger::debug(const std::string &msg) {
  log("DEBUG", msg, LoggerLevel::Debug);
}

void Logger::info(const std::string &msg) {
  if(alreadyLogged(msg)) return;
  log("INFO", msg, LoggerLevel::Info);
}

void Logger::warn(const std::string &msg) {
  if(alreadyLogged(msg)) return;
  log("WARN", msg, LoggerLevel::Warn);
}

void Logger::error(const std::string &msg) {
  if(alreadyLogged(msg)) return;
  log("ERROR", msg, LoggerLevel::Error);
}

void Logger::log(const std::string &prefix,
                 const std::string &msg,
                 LoggerLevel msgLevel) {
  logMutex.take(10);
  if(level < msgLevel) return;
  std::stringstream fmtMsg{};
  fmtMsg << std::setw(5) << prefix << std::setw(0) << ": " + msg << '\n';
  std::cout << fmtMsg.str();
  std::fstream file{logFilename, std::fstream::app};
  file << fmtMsg.str();
  if(msgLevel <= LoggerLevel::Info) GUI::writeToLog(fmtMsg.str());
  logMutex.give();
}

bool Logger::alreadyLogged(const std::string &msg) {
  if(std::find(logs.begin(), logs.end(), msg) != logs.end()) return true;
  logs.push_back(msg);
  return false;
}

const std::string Logger::logFilename{"log.txt"};

pros::Mutex Logger::logMutex{};
} // namespace atum
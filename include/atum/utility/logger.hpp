#pragma once

#include "../../pros/rtos.hpp"
#include "gui.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace atum {
class Logger {
  public:
  enum class LoggerLevel { Debug = 4, Info = 3, Warn = 2, Error = 1, Off = 0 };

  Logger(const LoggerLevel iLevel = LoggerLevel::Info);

  void debug(const std::string &msg);

  void info(const std::string &msg);

  void warn(const std::string &msg);

  void error(const std::string &msg);

  private:
  friend void GUI::writeToLog(const std::string &msg);

  void log(const std::string &prefix,
           const std::string &msg,
           LoggerLevel msgLevel);

  static const std::string logFilename;
  static pros::Mutex logMutex;

  const LoggerLevel level;
};

} // namespace atum
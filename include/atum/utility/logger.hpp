#pragma once

#include "../../pros/rtos.hpp"
#include "gui.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace atum {
class Logger {
  public:
  /**
   * @brief Various levels for logging. Controls what messages are logged.
   *
   */
  enum class LoggerLevel { Debug = 4, Info = 3, Warn = 2, Error = 1, Off = 0 };

  /**
   * @brief Constructs a new Logger object.
   *
   * @param iLevel
   */
  Logger(const LoggerLevel iLevel = LoggerLevel::Info);

  /**
   * @brief Log message if logger level is debug.
   *
   * @param msg
   */
  void debug(const std::string &msg);

  /**
   * @brief Log message if logger level is info or higher.
   *
   * @param msg
   */
  void info(const std::string &msg);

  /**
   * @brief Log message if logger level is warn or higher.
   *
   * @param msg
   */
  void warn(const std::string &msg);

  /**
   * @brief Log message if logger level is error or higher.
   *
   * @param msg
   */
  void error(const std::string &msg);

  private:
  // Allow Logger to use writeToLog.
  friend void GUI::writeToLog(const std::string &msg);

  /**
   * @brief Adds prefix to the message, and outputs to the terminal, file, and
   * screen depending on the message level.
   *
   * @param prefix
   * @param msg
   * @param msgLevel
   */
  void log(const std::string &prefix,
           const std::string &msg,
           LoggerLevel msgLevel);

  /**
   * @brief Checks if the message has already been printed.
   *
   * @param msg
   * @return true
   * @return false
   */
  bool alreadyLogged(const std::string &msg);

  static const std::string logFilename;
  static pros::Mutex logMutex;

  const LoggerLevel level;

  std::vector<std::string> logs;
};

} // namespace atum
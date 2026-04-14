#pragma once

#include <stdarg.h>
#include <stdio.h>

#ifdef ARDUINO
#include "Arduino.h"
#endif

/// @brief Severity level for logger output.
enum class LidarLogLevel {
  ERROR = 0,
  WARN = 1,
  INFO = 2,
  DEBUG = 3,
  TRACE = 4,
  OFF = 5,
};

/**
 * @brief Lightweight logger with vararg APIs for multiple log levels.
 *
 * Supports built-in output targets for Arduino (`Print`) and non-Arduino
 * (`FILE*`).
 */
class LidarLoggerClass {
 public:
  LidarLoggerClass() = default;

  /// @brief Set minimum level to output. Messages above this level are ignored.
  void setLevel(LidarLogLevel level) { level_ = level; }

  /// @brief Get currently configured minimum output level.
  LidarLogLevel level() const { return level_; }

#ifdef ARDUINO
  /// @brief Set Arduino print output used when no callback is configured.
  void setOutput(Print& output) { p_output_ = &output; }
#else
  /// @brief Set FILE output used when no callback is configured.
  void setOutput(FILE* output) { p_output_ = output; }
#endif

  /// @brief Generic vararg log method.
  void log(LidarLogLevel level, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(level, fmt, args);
    va_end(args);
  }

  /// @brief Log error message.
  void error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(LidarLogLevel::ERROR, fmt, args);
    va_end(args);
  }

  /// @brief Log warning message.
  void warn(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(LidarLogLevel::WARN, fmt, args);
    va_end(args);
  }

  /// @brief Log informational message.
  void info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(LidarLogLevel::INFO, fmt, args);
    va_end(args);
  }

  /// @brief Log debug message.
  void debug(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(LidarLogLevel::DEBUG, fmt, args);
    va_end(args);
  }

  /// @brief Log trace message.
  void trace(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlog(LidarLogLevel::TRACE, fmt, args);
    va_end(args);
  }

 protected:
  bool shouldLog(LidarLogLevel msg_level) const {
    return static_cast<int>(msg_level) <= static_cast<int>(level_);
  }

  const char* levelPrefix(LidarLogLevel level) const {
    switch (level) {
      case LidarLogLevel::ERROR:
        return "[ERROR] ";
      case LidarLogLevel::WARN:
        return "[WARN ] ";
      case LidarLogLevel::INFO:
        return "[INFO ] ";
      case LidarLogLevel::DEBUG:
        return "[DEBUG] ";
      case LidarLogLevel::TRACE:
        return "[TRACE] ";
      case LidarLogLevel::OFF:
      default:
        return "";
    }
  }

  void vlog(LidarLogLevel level, const char* fmt, va_list args) {
    if (fmt == nullptr) return;
    if (!shouldLog(level)) return;

    char message[256];
    vsnprintf(message, sizeof(message), fmt, args);

    const char* prefix = levelPrefix(level);
#ifdef ARDUINO
    if (p_output_ == nullptr) return;
    p_output_->print(prefix);
    p_output_->println(message);
#else
    FILE* out = p_output_;
    if (out == nullptr) {
      out = (level == LidarLogLevel::ERROR || level == LidarLogLevel::WARN)
                ? stderr
                : stdout;
    }
    fprintf(out, "%s%s\n", prefix, message);
    fflush(out);
#endif
  }

  LidarLogLevel level_ = LidarLogLevel::INFO;
#ifdef ARDUINO
  Print* p_output_ = &Serial;
#else
  FILE* p_output_ = nullptr;
#endif
};

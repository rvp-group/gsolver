/**
 * @file logger.h
 * @brief Lightweight logging utility for gsolver experiments.
 *
 * Provides structured logging with severity levels, timestamps, and consistent formatting.
 * Thread-safe and header-only.
 *
 * Usage:
 *   LOG_INFO("Processing {} poses", num_poses);
 *   LOG_WARN("Low confidence: {:.2f}", confidence);
 *   LOG_ERROR("Failed to open file: {}", path);
 */

#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

namespace examples {

  // ============================================================================
  // Log Levels
  // ============================================================================

  enum class LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3, FATAL = 4 };

  inline const char* levelToString(LogLevel level) {
    switch (level) {
      case LogLevel::DEBUG:
        return "DEBUG";
      case LogLevel::INFO:
        return "INFO ";
      case LogLevel::WARN:
        return "WARN ";
      case LogLevel::ERROR:
        return "ERROR";
      case LogLevel::FATAL:
        return "FATAL";
      default:
        return "?????";
    }
  }

  inline const char* levelToColor(LogLevel level) {
    switch (level) {
      case LogLevel::DEBUG:
        return "\033[36m"; // Cyan
      case LogLevel::INFO:
        return "\033[32m"; // Green
      case LogLevel::WARN:
        return "\033[33m"; // Yellow
      case LogLevel::ERROR:
        return "\033[31m"; // Red
      case LogLevel::FATAL:
        return "\033[35m"; // Magenta
      default:
        return "\033[0m";
    }
  }

  // ============================================================================
  // Logger Class
  // ============================================================================

  class Logger {
  public:
    static Logger& instance() {
      static Logger logger;
      return logger;
    }

    void setLevel(LogLevel level) {
      min_level_ = level;
    }
    void setColorEnabled(bool enabled) {
      color_enabled_ = enabled;
    }
    void setModuleName(const std::string& name) {
      module_name_ = name;
    }

    template <typename... Args>
    void log(LogLevel level, const std::string& file, int line, const std::string& format, Args&&... args) {
      if (level < min_level_)
        return;

      std::lock_guard<std::mutex> lock(mutex_);

      // Format message
      std::string message = formatMessage(format, std::forward<Args>(args)...);

      // Get timestamp
      auto now       = std::chrono::system_clock::now();
      auto time      = std::chrono::system_clock::to_time_t(now);
      auto ms        = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
      std::tm tm_buf = {};
      localtime_r(&time, &tm_buf);

      // Build log line
      std::ostringstream oss;

      if (color_enabled_) {
        oss << levelToColor(level);
      }

      oss << "[" << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";

      oss << "[" << levelToString(level) << "] ";

      if (!module_name_.empty()) {
        oss << "[" << module_name_ << "] ";
      }

      oss << message;

      if (color_enabled_) {
        oss << "\033[0m";
      }

      // Output
      if (level >= LogLevel::ERROR) {
        std::cerr << oss.str() << std::endl;
      } else {
        std::cout << oss.str() << std::endl;
      }
    }

    // Simple progress indicator
    void progress(const std::string& task, int current, int total) {
      std::lock_guard<std::mutex> lock(mutex_);

      int percent   = (total > 0) ? (current * 100 / total) : 0;
      int bar_width = 30;
      int filled    = (bar_width * current) / total;

      // Build timestamp
      auto now    = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      auto ms     = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
      std::tm tm_buf;
      localtime_r(&time_t, &tm_buf);

      std::ostringstream oss;
      oss << "\r";
      if (color_enabled_) {
        oss << "\033[32m"; // Green
      }
      oss << "[" << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      oss << "[" << levelToString(LogLevel::INFO) << "] ";
      if (!module_name_.empty()) {
        oss << "[" << module_name_ << "] ";
      }
      oss << task << " [";
      for (int i = 0; i < bar_width; ++i) {
        oss << (i < filled ? "█" : "░");
      }
      oss << "] " << percent << "% (" << current << "/" << total << ")";
      if (color_enabled_) {
        oss << "\033[0m";
      }

      std::cout << oss.str() << std::flush;

      if (current == total) {
        std::cout << std::endl;
      }
    }

    // Section header for structured output
    void section(const std::string& title) {
      std::lock_guard<std::mutex> lock(mutex_);
      std::string separator(60, '=');
      std::cout << "\n" << separator << "\n";
      std::cout << "  " << title << "\n";
      std::cout << separator << "\n" << std::endl;
    }

    // Step indicator for multi-step processes (with format support)
    template <typename... Args>
    void step(int num, int total, const std::string& format, Args&&... args) {
      std::string desc = formatMessage(format, std::forward<Args>(args)...);
      log(LogLevel::INFO, "", 0, "[{}/{}] {}", num, total, desc);
    }

  private:
    Logger() : min_level_(LogLevel::INFO), color_enabled_(true) {
    }

    // Simple format function (supports {} placeholders)
    template <typename T, typename... Args>
    std::string formatMessage(const std::string& format, T&& first, Args&&... rest) {
      std::ostringstream oss;
      size_t pos = format.find("{}");
      if (pos != std::string::npos) {
        oss << format.substr(0, pos) << first << formatMessage(format.substr(pos + 2), std::forward<Args>(rest)...);
      } else {
        oss << format;
      }
      return oss.str();
    }

    std::string formatMessage(const std::string& format) {
      return format;
    }

    LogLevel min_level_;
    bool color_enabled_;
    std::string module_name_;
    std::mutex mutex_;
  };

  // ============================================================================
  // Convenience Macros
  // ============================================================================

#define LOG_DEBUG(...) examples::Logger::instance().log(examples::LogLevel::DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...) examples::Logger::instance().log(examples::LogLevel::INFO, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...) examples::Logger::instance().log(examples::LogLevel::WARN, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...) examples::Logger::instance().log(examples::LogLevel::ERROR, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_FATAL(...) examples::Logger::instance().log(examples::LogLevel::FATAL, __FILE__, __LINE__, __VA_ARGS__)

#define LOG_PROGRESS(current, total, task) examples::Logger::instance().progress(task, current, total)
#define LOG_SECTION(title) examples::Logger::instance().section(title)
#define LOG_STEP(num, total, ...) examples::Logger::instance().step(num, total, __VA_ARGS__)

#define LOG_SET_LEVEL(level) examples::Logger::instance().setLevel(level)
#define LOG_SET_MODULE(name) examples::Logger::instance().setModuleName(name)
#define LOG_SET_COLOR(enabled) examples::Logger::instance().setColorEnabled(enabled)

} // namespace examples

#pragma once

#include <string>
#include <deque>
#include <mutex>
#include <fstream>
#include <memory>

// Enum for log message levels
enum class LogLevel {
  Debug,   // Debug information
  Info,    // Normal information
  Warning, // Warnings
  Error    // Errors
};

// Structure to store log messages with their level
struct LogMessage {
  std::string text;
  LogLevel level;
  std::string timestamp;

  LogMessage(const std::string& msg, LogLevel lvl, const std::string& time)
    : text(msg), level(lvl), timestamp(time) {
  }
};

// Logger class for handling text logging
class Logger {
private:
  // Singleton instance
  static std::unique_ptr<Logger> s_instance;

  // Container to store recent log messages
  std::deque<LogMessage> m_logMessages;

  // Maximum number of messages to keep in memory
  const size_t MAX_LOG_ENTRIES = 1000;

  // Mutex for thread safety
  std::mutex m_logMutex;

  // Current log file
  std::ofstream m_logFile;

  // Current date string
  std::string m_currentDate;

  // Log level threshold for console output
  LogLevel m_consoleLogLevel = LogLevel::Info;

  // Log level threshold for file output
  LogLevel m_fileLogLevel = LogLevel::Debug;

  // Enable/disable console output
  bool m_consoleOutputEnabled = true;

  // Constructor is private (singleton pattern)
  Logger();

  // Open a new log file for the current date
  void OpenLogFile();

  // Check if date has changed and update log file if needed
  void CheckAndUpdateLogFile();

  // Get current timestamp as string
  std::string GetTimestamp();

  // Get string representation of log level
  std::string GetLevelString(LogLevel level);

public:
  // Destructor
  ~Logger();

  // Get singleton instance
  static Logger* GetInstance();

  // Log a message with specified level
  void Log(const std::string& message, LogLevel level = LogLevel::Info);

  // Convenience methods for different log levels
  void LogDebug(const std::string& message);
  void LogInfo(const std::string& message);
  void LogWarning(const std::string& message);
  void LogError(const std::string& message);

  // Clear all logs in memory
  void Clear();

  // Configure logging levels
  void SetConsoleLogLevel(LogLevel level);
  void SetFileLogLevel(LogLevel level);
  void EnableConsoleOutput(bool enable);

  // Save current logs to a specific file
  bool SaveLogsToFile(const std::string& filename);
};
#include "logger.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <filesystem>

// Initialize static instance
std::unique_ptr<Logger> Logger::s_instance = nullptr;

Logger::Logger() {
  // Get current date and open initial log file
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm = {};

#ifdef _WIN32
  localtime_s(&tm, &time);
#else
  tm = *std::localtime(&time);
#endif

  std::stringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%d");
  m_currentDate = ss.str();

  // Open log file
  OpenLogFile();
}

Logger::~Logger() {
  // Close log file if open
  if (m_logFile.is_open()) {
    m_logFile.close();
  }
}

Logger* Logger::GetInstance() {
  if (!s_instance) {
    s_instance = std::unique_ptr<Logger>(new Logger());
  }
  return s_instance.get();
}

void Logger::OpenLogFile() {
  // Create logs directory if it doesn't exist
  std::filesystem::path logDir = "logs";
  if (!std::filesystem::exists(logDir)) {
    std::filesystem::create_directory(logDir);
  }

  // Create filename with current date
  std::string filename = "logs/log_" + m_currentDate + ".txt";

  // Close existing file if open
  if (m_logFile.is_open()) {
    m_logFile.close();
  }

  // Open new log file (append mode)
  m_logFile.open(filename, std::ios::app);

  if (!m_logFile) {
    // If failed to open, try to create logs directory again and retry
    std::filesystem::create_directory(logDir);
    m_logFile.open(filename, std::ios::app);

    if (!m_logFile) {
      std::cerr << "ERROR: Failed to open log file: " << filename << std::endl;
    }
  }
}

void Logger::CheckAndUpdateLogFile() {
  // Get current date
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm = {};

#ifdef _WIN32
  localtime_s(&tm, &time);
#else
  tm = *std::localtime(&time);
#endif

  std::stringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%d");
  std::string currentDate = ss.str();

  // If date has changed, update log file
  if (currentDate != m_currentDate) {
    m_currentDate = currentDate;
    OpenLogFile();
  }
}

std::string Logger::GetTimestamp() {
  // Get current timestamp
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm = {};

#ifdef _WIN32
  localtime_s(&tm, &time);
#else
  tm = *std::localtime(&time);
#endif

  std::stringstream ss;
  ss << std::put_time(&tm, "%H:%M:%S");
  return ss.str();
}

std::string Logger::GetLevelString(LogLevel level) {
  switch (level) {
  case LogLevel::Debug:   return "DEBUG";
  case LogLevel::Info:    return "INFO";
  case LogLevel::Warning: return "WARNING";
  case LogLevel::Error:   return "ERROR";
  default:                return "UNKNOWN";
  }
}

void Logger::Log(const std::string& message, LogLevel level) {
  // Lock mutex for thread safety
  std::lock_guard<std::mutex> lock(m_logMutex);

  // Check if date has changed and update log file if needed
  CheckAndUpdateLogFile();

  // Get current timestamp
  std::string timestamp = GetTimestamp();

  // Create log message
  LogMessage logMsg(message, level, timestamp);

  // Add to deque (limit to MAX_LOG_ENTRIES entries)
  m_logMessages.push_back(logMsg);
  if (m_logMessages.size() > MAX_LOG_ENTRIES) {
    m_logMessages.pop_front();
  }

  // Get level string
  std::string levelString = GetLevelString(level);

  // Format log message with timestamp and level
  std::string formattedMessage = "[" + timestamp + "] [" + levelString + "] " + message;

  // Output to console if enabled and level meets threshold
  if (m_consoleOutputEnabled && level >= m_consoleLogLevel) {
    // Use appropriate stream based on log level
    if (level == LogLevel::Error) {
      std::cerr << formattedMessage << std::endl;
    }
    else {
      std::cout << formattedMessage << std::endl;
    }
  }

  // Write to log file if level meets threshold
  if (m_logFile.is_open() && level >= m_fileLogLevel) {
    m_logFile << formattedMessage << std::endl;
    m_logFile.flush(); // Ensure it's written immediately
  }
}

// Convenience methods for different log levels
void Logger::LogDebug(const std::string& message) {
  Log(message, LogLevel::Debug);
}

void Logger::LogInfo(const std::string& message) {
  Log(message, LogLevel::Info);
}

void Logger::LogWarning(const std::string& message) {
  Log(message, LogLevel::Warning);
}

void Logger::LogError(const std::string& message) {
  Log(message, LogLevel::Error);
}

void Logger::Clear() {
  std::lock_guard<std::mutex> lock(m_logMutex);
  m_logMessages.clear();
}

void Logger::SetConsoleLogLevel(LogLevel level) {
  m_consoleLogLevel = level;
}

void Logger::SetFileLogLevel(LogLevel level) {
  m_fileLogLevel = level;
}

void Logger::EnableConsoleOutput(bool enable) {
  m_consoleOutputEnabled = enable;
}

bool Logger::SaveLogsToFile(const std::string& filename) {
  std::lock_guard<std::mutex> lock(m_logMutex);

  // Create logs directory if it doesn't exist
  std::filesystem::path logDir = "logs";
  if (!std::filesystem::exists(logDir)) {
    std::filesystem::create_directory(logDir);
  }

  // Open file for writing
  std::ofstream file(filename);
  if (!file.is_open()) {
    return false;
  }

  // Write all logs to file
  for (const auto& logMsg : m_logMessages) {
    // Format message with timestamp and level
    std::string levelString = GetLevelString(logMsg.level);
    file << "[" << logMsg.timestamp << "] [" << levelString << "] " << logMsg.text << std::endl;
  }

  file.close();
  return true;
}
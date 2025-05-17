// acs_controller.h
#pragma once
#include <Windows.h>  // Include this first
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <map>
#include "MotionTypes.h"  // Make sure this is included
#include "logger.h"
// Include ACS controller library
#include "acs_controller.h"
#include "ACSC.h"

class ACSController {
public:
  ACSController();
  ~ACSController();

  // Connection methods
  bool Connect(const std::string& ipAddress, int port = ACSC_SOCKET_STREAM_PORT);
  void Disconnect();
  bool IsConnected() const { return m_isConnected; }

  // Basic motion commands
  bool MoveToPosition(const std::string& axis, double position, bool blocking = true);
  bool MoveRelative(const std::string& axis, double distance, bool blocking = true);
  bool HomeAxis(const std::string& axis);
  bool StopAxis(const std::string& axis);
  bool StopAllAxes();

  // Status methods
  bool IsMoving(const std::string& axis);
  bool GetPosition(const std::string& axis, double& position);
  bool GetPositions(std::map<std::string, double>& positions);

  // Servo control
  bool EnableServo(const std::string& axis, bool enable);
  bool IsServoEnabled(const std::string& axis, bool& enabled);

  // Motion configuration
  bool SetVelocity(const std::string& axis, double velocity);
  bool GetVelocity(const std::string& axis, double& velocity);

  // Configuration from MotionDevice
  bool ConfigureFromDevice(const MotionDevice& device);

  // Moving to named positions from MotionTypes
  bool MoveToNamedPosition(const std::string& deviceName, const std::string& positionName);

  // Helper methods
  bool WaitForMotionCompletion(const std::string& axis, double timeoutSeconds = 30.0);

  // UI rendering
  void RenderUI();

  // Control window visibility
  void SetWindowVisible(bool visible) { m_showWindow = visible; }
  void SetWindowTitle(const std::string& title) { m_windowTitle = title; }

  // Add this method to expose available axes
  const std::vector<std::string>& GetAvailableAxes() const { return m_availableAxes; }
  // In acs_controller.h in the public section:
  bool MoveToPositionMultiAxis(const std::vector<std::string>& axes,
    const std::vector<double>& positions,
    bool blocking = true);
    // Set available axes
  bool SetAvailableAxes(const std::vector<std::string>& axes);

// Add to acs_controller.h private section:

  // Helper to format axes list as string
  std::string FormatAxesString(const std::vector<std::string>& axes);
private:
  // Communication thread methods
  void StartCommunicationThread();
  void StopCommunicationThread();
  void CommunicationThreadFunc();
  // Add to the private section of ACSController class in acs_controller.h
  void ProcessCommandQueue();
  void UpdatePositions();
  void UpdateMotorStatus();
  // Add to public methods section in acs_controller.h
  bool StartMotion(const std::string& axis);
  // Command queue structure
  struct MotorCommand {
    std::string axis;
    double distance;
    bool executed;
  };

  std::string m_windowTitle = "ACS Controller"; // Default title

  // Thread-related members
  std::thread m_communicationThread;
  std::mutex m_mutex;
  std::condition_variable m_condVar;
  std::atomic<bool> m_threadRunning{ false };
  std::atomic<bool> m_terminateThread{ false };
  std::atomic<bool> m_isConnected{ false };

  // Command queue
  std::vector<MotorCommand> m_commandQueue;
  std::mutex m_commandMutex;

  // Controller handle
  HANDLE m_controllerId;  // Handle for the ACS controller

  // Configuration
  std::string m_ipAddress;
  int m_port;
  std::vector<std::string> m_availableAxes;

  // Status monitoring
  std::map<std::string, double> m_axisPositions;
  std::map<std::string, bool> m_axisMoving;
  std::map<std::string, bool> m_axisServoEnabled;

  // Logging
  Logger* m_logger;

  // UI state
  bool m_showWindow = false;
  double m_jogDistance = 1.0;  // Default jog distance in mm

  // Convert between string axis names and ACS axis indices
  int GetAxisIndex(const std::string& axis);

  // Debug flag
  bool m_enableDebug = false;  // Enable debug logging

  // Cache status
  std::chrono::steady_clock::time_point m_lastStatusUpdate;
  std::chrono::steady_clock::time_point m_lastPositionUpdate;
  const int m_statusUpdateInterval = 200;  // 5Hz updates
};
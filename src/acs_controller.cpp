// acs_controller.cpp
#include "acs_controller.h"
#include <iostream>
#include <chrono>
#include <sstream>
#include <algorithm>

// Constructor - initialize with correct axis identifiers
ACSController::ACSController()
  : m_controllerId(ACSC_INVALID),
  m_port(ACSC_SOCKET_STREAM_PORT) {

  // Initialize atomic variables
  m_isConnected.store(false);
  m_threadRunning.store(false);
  m_terminateThread.store(false);

  m_logger = Logger::GetInstance();
  m_logger->LogInfo("ACSController: Initializing controller");

  // Initialize available axes with string identifiers (consistent with PI controller)
  m_availableAxes = { "X", "Y", "Z"};

  // Start communication thread
  StartCommunicationThread();
}

ACSController::~ACSController() {
  //m_logger->LogInfo("ACSController: Shutting down controller");

  // Stop communication thread
  StopCommunicationThread();

  // Disconnect if still connected
  if (m_isConnected) {
    Disconnect();
  }
}

void ACSController::StartCommunicationThread() {
  if (!m_threadRunning) {
    m_threadRunning.store(true);
    m_terminateThread.store(false);
    m_communicationThread = std::thread(&ACSController::CommunicationThreadFunc, this);
    //m_logger->LogInfo("ACSController: Communication thread started");
  }
}

void ACSController::StopCommunicationThread() {
  if (m_threadRunning) {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_terminateThread.store(true);
    }
    m_condVar.notify_all();

    if (m_communicationThread.joinable()) {
      m_communicationThread.join();
    }

    m_threadRunning.store(false);
    //m_logger->LogInfo("ACSController: Communication thread stopped");
  }
}

// In ACSController.cpp - improve CommunicationThreadFunc
void ACSController::CommunicationThreadFunc() {
  // Set update interval to 200ms (5 Hz)
  const auto updateInterval = std::chrono::milliseconds(200);

  // Frame counter for less frequent updates
  int frameCounter = 0;

  // Initialization of last update timestamps
  m_lastStatusUpdate = std::chrono::steady_clock::now();
  m_lastPositionUpdate = m_lastStatusUpdate;

  while (!m_terminateThread) {
    auto cycleStartTime = std::chrono::steady_clock::now();

    // Process any pending motor commands first for responsiveness
    {
      std::lock_guard<std::mutex> lock(m_commandMutex);
      for (auto& cmd : m_commandQueue) {
        if (!cmd.executed) {
          // Execute the command
          MoveRelative(cmd.axis, cmd.distance, false);
          cmd.executed = true;
        }
      }

      // Remove executed commands
      m_commandQueue.erase(
        std::remove_if(m_commandQueue.begin(), m_commandQueue.end(),
          [](const MotorCommand& cmd) { return cmd.executed; }),
        m_commandQueue.end());
    }

    // Only update if connected
    if (m_isConnected) {
      frameCounter++;

      // Always update positions
      std::map<std::string, double> positions;
      if (GetPositions(positions)) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_axisPositions = positions;
        m_lastPositionUpdate = std::chrono::steady_clock::now();
      }

      // Update other status less frequently (every 3rd frame, ~1.67Hz)
      if (frameCounter % 3 == 0) {
        // Update axis motion status for X, Y, Z only
        for (const auto& axis : { "X", "Y", "Z" }) {
          bool moving = IsMoving(axis);
          std::lock_guard<std::mutex> lock(m_mutex);
          m_axisMoving[axis] = moving;
        }

        // Update servo status for X, Y, Z only
        for (const auto& axis : { "X", "Y", "Z" }) {
          bool enabled;
          if (IsServoEnabled(axis, enabled)) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_axisServoEnabled[axis] = enabled;
          }
        }

        m_lastStatusUpdate = std::chrono::steady_clock::now();
      }
    }

    // Calculate how long to sleep to maintain consistent update rate
    auto cycleEndTime = std::chrono::steady_clock::now();
    auto cycleDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
      cycleEndTime - cycleStartTime);
    auto sleepTime = updateInterval - cycleDuration;

    // Wait for next update or termination
    std::unique_lock<std::mutex> lock(m_mutex);
    if (sleepTime.count() > 0) {
      m_condVar.wait_for(lock, sleepTime, [this]() { return m_terminateThread.load(); });
    }
    else {
      // No sleep needed if we're already behind schedule, but yield to let other threads run
      lock.unlock();
      std::this_thread::yield();
    }
  }
}


// Helper method to process the command queue
void ACSController::ProcessCommandQueue() {
  std::lock_guard<std::mutex> lock(m_commandMutex);
  for (auto& cmd : m_commandQueue) {
    if (!cmd.executed) {
      // Execute the command
      MoveRelative(cmd.axis, cmd.distance, false);
      cmd.executed = true;
    }
  }

  // Remove executed commands
  m_commandQueue.erase(
    std::remove_if(m_commandQueue.begin(), m_commandQueue.end(),
      [](const MotorCommand& cmd) { return cmd.executed; }),
    m_commandQueue.end());
}

// Helper method to update positions using batch query
void ACSController::UpdatePositions() {
  if (!m_isConnected) return;

  std::map<std::string, double> positions;
  if (GetPositions(positions)) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_axisPositions = positions;
    m_lastPositionUpdate = std::chrono::steady_clock::now();
  }
}

// Helper method to update motor status (moving, servo state)
void ACSController::UpdateMotorStatus() {
  if (!m_isConnected) return;

  auto now = std::chrono::steady_clock::now();

  // Use batch queries if possible, otherwise query each axis
  for (const auto& axis : m_availableAxes) {
    int axisIndex = GetAxisIndex(axis);
    if (axisIndex >= 0) {
      int state = 0;
      if (acsc_GetMotorState(m_controllerId, axisIndex, &state, NULL)) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_axisMoving[axis] = (state & ACSC_MST_MOVE) != 0;
        m_axisServoEnabled[axis] = (state & ACSC_MST_ENABLE) != 0;
      }
    }
  }

  m_lastStatusUpdate = now;
}

// Helper to convert string axis identifiers to ACS axis indices
int ACSController::GetAxisIndex(const std::string& axis) {
  if (axis == "X") return ACSC_AXIS_X;
  if (axis == "Y") return ACSC_AXIS_Y;
  if (axis == "Z") return ACSC_AXIS_Z;

  //m_logger->LogWarning("ACSController: Unknown axis identifier: " + axis);
  return -1;
}

bool ACSController::Connect(const std::string& ipAddress, int port) {
  // Check if already connected
  if (m_isConnected) {
    //m_logger->LogWarning("ACSController: Already connected to a controller");
    return true;
  }

  std::cout << "ACSController: Attempting connection to " << ipAddress << ":" << port << std::endl;
  //m_logger->LogInfo("ACSController: Connecting to controller at " + ipAddress + ":" + std::to_string(port));

  // Store connection parameters
  m_ipAddress = ipAddress;
  m_port = port;

  // Attempt to connect to the controller
  // ACS requires a non-const char buffer for the IP address
  char ipBuffer[64];
  strncpy(ipBuffer, m_ipAddress.c_str(), sizeof(ipBuffer) - 1);
  ipBuffer[sizeof(ipBuffer) - 1] = '\0'; // Ensure null termination

  m_controllerId = acsc_OpenCommEthernet(ipBuffer, m_port);

  if (m_controllerId == ACSC_INVALID) {
    int errorCode = acsc_GetLastError();
    std::string errorMsg = "ACSController: Failed to connect to controller. Error code: " + std::to_string(errorCode);
    //m_logger->LogError(errorMsg);
    std::cout << errorMsg << std::endl;
    return false;
  }

  m_isConnected.store(true);
  std::cout << "ACSController: Successfully connected to " << ipAddress << std::endl;
  //m_logger->LogInfo("ACSController: Successfully connected to controller");

  // Enable all configured axes
  for (const auto& axis : m_availableAxes) {
    int axisIndex = GetAxisIndex(axis);
    if (axisIndex >= 0) {
      if (acsc_Enable(m_controllerId, axisIndex, NULL)) {
        //m_logger->LogInfo("ACSController: Enabled axis " + axis);
      }
      else {
        int error = acsc_GetLastError();
        //m_logger->LogError("ACSController: Failed to enable axis " + axis + ". Error: " + std::to_string(error));
      }
    }
  }

  // Initialize position cache immediately
  std::map<std::string, double> initialPositions;
  if (GetPositions(initialPositions)) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_axisPositions = initialPositions;
    m_lastPositionUpdate = std::chrono::steady_clock::now();

    // Log initial positions for debugging
    if (m_enableDebug) {
      std::stringstream ss;
      ss << "Initial positions: ";
      for (const auto& [axis, pos] : initialPositions) {
        ss << axis << "=" << pos << " ";
      }
      //m_logger->LogInfo(ss.str());
    }
  }
  else {
    //m_logger->LogWarning("ACSController: Failed to initialize position cache after connection");
  }

  return true;
}

void ACSController::Disconnect() {
  if (!m_isConnected) {
    return;
  }

  //m_logger->LogInfo("ACSController: Disconnecting from controller");

  // Ensure all axes are stopped before disconnecting
  StopAllAxes();

  // Close connection
  acsc_CloseComm(m_controllerId);

  m_isConnected.store(false);
  m_controllerId = ACSC_INVALID;

  //m_logger->LogInfo("ACSController: Disconnected from controller");
}

bool ACSController::MoveToPosition(const std::string& axis, double position, bool blocking) {
  if (!m_isConnected) {
    //m_logger->LogError("ACSController: Cannot move axis - not connected");
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  m_logger->LogInfo("ACSController: Moving axis " + axis + " to position " + std::to_string(position));

  // Set up arrays for acsc_ToPointM
  int axes[2] = { axisIndex, -1 }; // -1 marks the end of the array
  double points[1] = { position };

  // Command the move - using absolute positioning
  if (!acsc_ToPointM(m_controllerId, ACSC_AMF_WAIT, axes, points, NULL)) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to move axis. Error code: " + std::to_string(error));
    return false;
  }

  // Start the motion
  if (!StartMotion(axis)) {
    return false;
  }

  // If blocking mode, wait for motion to complete
  if (blocking) {
    return WaitForMotionCompletion(axis);
  }

  return true;
}


bool ACSController::MoveRelative(const std::string& axis, double distance, bool blocking) {
  if (!m_isConnected) {
    m_logger->LogError("ACSController: Cannot move axis - not connected");
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  m_logger->LogInfo("ACSController: Moving axis " + axis + " relative distance " +     std::to_string(distance));

  // Log pre-move position if debugging is enabled
  if (m_enableDebug) {
    double currentPos = 0.0;
    if (GetPosition(axis, currentPos)) {
      std::cout << "ACSController: Pre-move position of axis " << axis << " = "        << currentPos << std::endl;
    }
  }

  // Set up arrays for acsc_ToPointM with RELATIVE flag
  int axes[2] = { axisIndex, -1 }; // -1 marks the end of the array
  double distances[1] = { distance };

  // Command the relative move
  if (!acsc_ToPointM(m_controllerId, ACSC_AMF_WAIT | ACSC_AMF_RELATIVE, axes, distances, NULL)) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to move axis relatively. Error code: " +       std::to_string(error));
    return false;
  }

  // Start the motion
  if (!StartMotion(axis)) {
    return false;
  }

  // If blocking mode, wait for motion to complete
  if (blocking) {
    return WaitForMotionCompletion(axis);
  }

  return true;
}



bool ACSController::HomeAxis(const std::string& axis) {
  if (!m_isConnected) {
    m_logger->LogError("ACSController: Cannot home axis - not connected");
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  m_logger->LogInfo("ACSController: Homing axis " + axis);

  // For ACS, we use a specific homing command
  // This implementation may need to be adjusted based on your specific hardware
 // Option 1: Use a direct FaultClear + Home sequence
  if (!acsc_FaultClear(m_controllerId, axisIndex, NULL)) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to clear faults for homing. Error code: " + std::to_string(error));
    // Continue anyway as the axis might not have faults
  }

  // Wait for homing to complete
  return WaitForMotionCompletion(axis);
}

bool ACSController::StopAxis(const std::string& axis) {
  if (!m_isConnected) {
    m_logger->LogError("ACSController: Cannot stop axis - not connected");
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  m_logger->LogInfo("ACSController: Stopping axis " + axis);

  // Command the stop
  if (!acsc_Halt(m_controllerId, axisIndex, NULL)) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to stop axis. Error code: " + std::to_string(error));
    return false;
  }

  return true;
}

bool ACSController::StopAllAxes() {
  if (!m_isConnected) {
    m_logger->LogError("ACSController: Cannot stop all axes - not connected");
    return false;
  }

  m_logger->LogInfo("ACSController: Stopping all axes");

  // Command the stop for all axes
  if (!acsc_KillAll(m_controllerId, NULL)) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to stop all axes. Error code: " + std::to_string(error));
    return false;
  }

  return true;
}

// In ACSController.cpp - modify IsMoving to use cached values
bool ACSController::IsMoving(const std::string& axis) {
  if (!m_isConnected) {
    return false;
  }

  // Check if we have a recent cached value (less than 200ms old)
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - m_lastStatusUpdate).count();

  if (elapsed < m_statusUpdateInterval) {
    // Use cached value if it exists and is recent
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_axisMoving.find(axis);
    if (it != m_axisMoving.end()) {
      return it->second;
    }
  }

  // If no recent cached value, do direct query
  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  // Get the motion state
  int state = 0;
  if (!acsc_GetMotorState(m_controllerId, axisIndex, &state, NULL)) {
    return false;
  }

  // Check if the axis is moving based on state bits
  bool isMoving = (state & ACSC_MST_MOVE) != 0;

  // Update the cache
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_axisMoving[axis] = isMoving;
    m_lastStatusUpdate = now;
  }

  return isMoving;
}


// Modify logging in performance-critical areas
// For example, in GetPosition method:
bool ACSController::GetPosition(const std::string& axis, double& position) {
  if (!m_isConnected) {
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  if (!acsc_GetFPosition(m_controllerId, axisIndex, &position, NULL)) {
    int error = acsc_GetLastError();
    // Only log in debug mode to reduce overhead
    if (m_enableDebug) {
      std::cout << "ACSController: Error getting position for axis " << axis << ": " << error << std::endl;
    }
    return false;
  }

  return true;
}



// In ACSController::GetPositions - modify to use batch queries
bool ACSController::GetPositions(std::map<std::string, double>& positions) {
  if (!m_isConnected || m_availableAxes.empty()) {
    return false;
  }

  // Since we now have only X, Y, Z axes, we can optimize this for exactly 3 axes
  // Create arrays for the axes we know we have
  int axisIndices[3] = {
      GetAxisIndex("X"), GetAxisIndex("Y"), GetAxisIndex("Z")
  };
  double posArray[3] = { 0.0 };

  // Query positions individually for each axis
  // This could be optimized with a batch call if the ACS API supports it
  bool success = true;
  for (int i = 0; i < 3; i++) {
    if (axisIndices[i] >= 0) {
      if (!acsc_GetFPosition(m_controllerId, axisIndices[i], &posArray[i], NULL)) {
        success = false;
      }
    }
  }

  if (success) {
    // Fill the map with results
    for (int i = 0; i < 3; i++) {
      if (axisIndices[i] >= 0) {
        positions[m_availableAxes[i]] = posArray[i];
      }
    }
  }

  return success;
}

bool ACSController::EnableServo(const std::string& axis, bool enable) {
  if (!m_isConnected) {
    m_logger->LogError("ACSController: Cannot change servo state - not connected");
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  m_logger->LogInfo("ACSController: Setting servo state for axis " + axis + " to " +     (enable ? "enabled" : "disabled"));

  // Enable or disable the servo
  bool result = false;
  if (enable) {
    result = acsc_Enable(m_controllerId, axisIndex, NULL) != 0;
  }
  else {
    result = acsc_Disable(m_controllerId, axisIndex, NULL) != 0;
  }

  if (!result) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to set servo state. Error code: " + std::to_string(error));
  }

  return result;
}

bool ACSController::IsServoEnabled(const std::string& axis, bool& enabled) {
  if (!m_isConnected) {
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  // Get the motor state
  int state = 0;
  if (!acsc_GetMotorState(m_controllerId, axisIndex, &state, NULL)) {
    return false;
  }

  // Check if the axis is enabled based on state bits
  enabled = (state & ACSC_MST_ENABLE) != 0;
  return true;
}

bool ACSController::SetVelocity(const std::string& axis, double velocity) {
  if (!m_isConnected) {
    m_logger->LogError("ACSController: Cannot set velocity - not connected");
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  m_logger->LogInfo("ACSController: Setting velocity for axis " + axis + " to " + std::to_string(velocity));

  // Set the velocity
  if (!acsc_SetVelocity(m_controllerId, axisIndex, velocity, NULL)) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to set velocity. Error code: " + std::to_string(error));
    return false;
  }

  return true;
}

bool ACSController::GetVelocity(const std::string& axis, double& velocity) {
  if (!m_isConnected) {
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  // Get the velocity
  if (!acsc_GetVelocity(m_controllerId, axisIndex, &velocity, NULL)) {
    return false;
  }

  return true;
}

bool ACSController::WaitForMotionCompletion(const std::string& axis, double timeoutSeconds) {
  if (!m_isConnected) {
    m_logger->LogError("ACSController: Cannot wait for motion completion - not connected");
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    return false;
  }

  m_logger->LogInfo("ACSController: Waiting for motion completion on axis " + axis);

  // Use system clock for timeout
  auto startTime = std::chrono::steady_clock::now();
  int checkCount = 0;

  while (true) {
    checkCount++;
    bool stillMoving = IsMoving(axis);

    if (!stillMoving) {
      m_logger->LogInfo("ACSController: Motion completed on axis " + axis);
      return true;
    }

    // Check for timeout
    auto currentTime = std::chrono::steady_clock::now();
    auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();

    if (elapsedSeconds > timeoutSeconds) {
      m_logger->LogWarning("ACSController: Timeout waiting for motion completion on axis " + axis);
      return false;
    }

    // Sleep to avoid CPU spikes
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

bool ACSController::ConfigureFromDevice(const MotionDevice& device) {
  if (m_isConnected) {
    m_logger->LogWarning("ACSController: Cannot configure from device while connected");
    return false;
  }

  m_logger->LogInfo("ACSController: Configuring from device: " + device.Name);

  // Store the IP address and port from the device configuration
  m_ipAddress = device.IpAddress;
  m_port = device.Port;

  // Handle axes configuration - in case the device has specific axes defined
  // The MotionConfigManager would have loaded the axes from the JSON
  // This requires the caller to also pass a vector of available axes

  return true;
}


bool ACSController::MoveToNamedPosition(const std::string& deviceName, const std::string& positionName) {
  m_logger->LogInfo("ACSController: Moving to named position " + positionName + " for device " + deviceName);

  // This method would typically lookup the position from a configuration source
  // For now, it's a placeholder that could be extended with your position data

  return true;
}

bool ACSController::StartMotion(const std::string& axis) {
  if (!m_isConnected) {
    m_logger->LogError("ACSController: Cannot start motion - not connected");
    return false;
  }

  int axisIndex = GetAxisIndex(axis);
  if (axisIndex < 0) {
    m_logger->LogError("ACSController: Invalid axis for starting motion: " + axis);
    return false;
  }

  m_logger->LogInfo("ACSController: Starting motion on axis " + axis);

  // Set up axis array for GoM (ending with -1)
  int axes[2] = { axisIndex, -1 };

  // Call acsc_GoM to start the motion
  if (!acsc_GoM(m_controllerId, axes, NULL)) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to start motion on axis " + axis +       ". Error code: " + std::to_string(error));
    return false;
  }

  return true;
}


bool ACSController::MoveToPositionMultiAxis(const std::vector<std::string>& axes,
  const std::vector<double>& positions,
  bool blocking) {
  if (!m_isConnected) {
    m_logger->LogError("ACSController: Cannot move axes - not connected");
    return false;
  }

  if (axes.size() != positions.size() || axes.empty()) {
    m_logger->LogError("ACSController: Invalid axes/positions arrays for multi-axis move");
    return false;
  }

  // Log the motion command
  std::stringstream ss;
  ss << "ACSController: Moving multiple axes to positions: ";
  for (size_t i = 0; i < axes.size(); i++) {
    ss << axes[i] << "=" << positions[i] << " ";
  }
  m_logger->LogInfo(ss.str());

  // Convert string axes to ACS axis indices
  std::vector<int> axisIndices;
  for (const auto& axis : axes) {
    int axisIndex = GetAxisIndex(axis);
    if (axisIndex < 0) {
      m_logger->LogError("ACSController: Invalid axis: " + axis);
      return false;
    }
    axisIndices.push_back(axisIndex);
  }

  // Set up arrays for acsc_ToPointM
  std::vector<int> axesArray(axisIndices.size() + 1);  // +1 for the terminating -1
  for (size_t i = 0; i < axisIndices.size(); i++) {
    axesArray[i] = axisIndices[i];
  }
  axesArray[axisIndices.size()] = -1;  // Mark the end of the array with -1

  // Command the move using acsc_ToPointM
  if (!acsc_ToPointM(m_controllerId, ACSC_AMF_WAIT, axesArray.data(),
    const_cast<double*>(positions.data()), NULL)) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to move axes. Error code: " +       std::to_string(error));
    return false;
  }

  // Start the motion - for multi-axis we'll need to use GoM instead of Go
  if (!acsc_GoM(m_controllerId, axesArray.data(), NULL)) {
    int error = acsc_GetLastError();
    m_logger->LogError("ACSController: Failed to start motion. Error code: " +       std::to_string(error));
    return false;
  }

  // If blocking mode, wait for motion to complete on all axes
  if (blocking) {
    bool allCompleted = true;
    for (const auto& axis : axes) {
      if (!WaitForMotionCompletion(axis)) {
        m_logger->LogError("ACSController: Timeout waiting for motion completion on axis " + axis);
        allCompleted = false;
      }
    }
    return allCompleted;
  }

  return true;
}


// Add a method to set available axes directly
bool ACSController::SetAvailableAxes(const std::vector<std::string>& axes) {
  if (m_isConnected) {
    m_logger->LogWarning("ACSController: Cannot set available axes while connected");
    return false;
  }

  m_availableAxes = axes;
  m_logger->LogInfo("ACSController: Set available axes: " + FormatAxesString(axes));
  return true;
}

// Helper method to format a vector of axes for logging
std::string ACSController::FormatAxesString(const std::vector<std::string>& axes) {
  std::stringstream ss;
  for (size_t i = 0; i < axes.size(); ++i) {
    if (i > 0) ss << ", ";
    ss << axes[i];
  }
  return ss.str();
}

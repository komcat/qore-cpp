#include <iostream>
#include <thread>
#include <chrono>
#include <map>
#include "acs_controller.h"
#include "MotionTypes.h"

// Function to print current position of all axes
void printCurrentPositions(ACSController& controller) {
  std::map<std::string, double> positions;
  if (controller.GetPositions(positions)) {
    std::cout << "Current Positions: ";
    for (const auto& [axis, position] : positions) {
      std::cout << axis << "=" << position << " ";
    }
    std::cout << std::endl;
  }
  else {
    std::cout << "Failed to get positions" << std::endl;
  }
}

int main() {
  std::cout << "ACS Controller Test Program" << std::endl;

  // Create controller instance
  ACSController controller;

  // Connection parameters from screenshot
  std::string ipAddress = "127.0.0.100";
  int port = 701;

  std::cout << "Connecting to ACS controller at " << ipAddress << ":" << port << "..." << std::endl;

  // Connect to the controller
  if (controller.Connect(ipAddress, port)) {
    std::cout << "Successfully connected to ACS controller!" << std::endl;

    // Poll the positions every second for 30 seconds
    const int pollDurationSeconds = 30;
    std::cout << "Polling positions for " << pollDurationSeconds << " seconds..." << std::endl;

    for (int i = 0; i < pollDurationSeconds; i++) {
      std::cout << "Poll [" << (i + 1) << "/" << pollDurationSeconds << "]: ";
      printCurrentPositions(controller);

      // Check if axes are moving
      for (const auto& axis : controller.GetAvailableAxes()) {
        bool moving = controller.IsMoving(axis);
        bool enabled = false;
        controller.IsServoEnabled(axis, enabled);
        std::cout << "  " << axis << " - Moving: " << (moving ? "Yes" : "No")
          << ", Servo: " << (enabled ? "Enabled" : "Disabled") << std::endl;
      }

      // Optional: Try a small relative move on X axis after 5 seconds
      if (i == 5) {
        std::cout << "\nTrying a relative move of 10 units on X axis...\n" << std::endl;
        controller.MoveRelative("X", 10.0, false);  // non-blocking move
      }

      // Wait 1 second before next poll
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Disconnect from the controller
    std::cout << "Test complete. Disconnecting from controller..." << std::endl;
    controller.Disconnect();
    std::cout << "Disconnected." << std::endl;
  }
  else {
    std::cout << "Failed to connect to ACS controller at " << ipAddress << ":" << port << std::endl;
    std::cout << "Make sure the controller is powered on and reachable on the network." << std::endl;
  }

  return 0;
}
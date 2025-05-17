#include <iostream>
#include <thread>
#include <chrono>
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
#include "acs_controller.h"
#include "MotionTypes.h"

// Function to print current position of all axes on a single line
void printCurrentPositions(ACSController& controller, int pollCount) {
  std::map<std::string, double> positions;

  // Clear the current line
  std::cout << "\r";
  // Show poll count
  std::cout << "Poll [" << std::setw(2) << pollCount << "] ";

  if (controller.GetPositions(positions)) {
    // Build status string with all axes
    std::cout << "Positions: ";
    for (const auto& [axis, position] : positions) {
      std::cout << axis << "=" << std::fixed << std::setprecision(3) << position << " ";
    }

    // Add axis status inline
    for (const auto& axis : controller.GetAvailableAxes()) {
      bool moving = controller.IsMoving(axis);
      bool enabled = false;
      controller.IsServoEnabled(axis, enabled);

      std::cout << "| " << axis
        << (moving ? "(Moving)" : "(Idle)")
        << (enabled ? "(Enabled)" : "(Disabled)") << " ";
    }

    // Pad with spaces to ensure clean overwrite and flush
    std::cout << std::string(10, ' ') << std::flush;
  }
  else {
    std::cout << "Failed to get positions" << std::string(50, ' ') << std::flush;
  }
}

int main() {
  // Clear screen once at the beginning
  std::cout << "\033[2J\033[1;1H";  // ANSI escape code to clear screen and move cursor to top-left
  std::cout << "ACS Controller Test Program" << std::endl;
  std::cout << "-------------------------" << std::endl;

  // Create controller instance
  ACSController controller;

  // Connection parameters
  std::string ipAddress = "127.0.0.100";
  int port = 701;

  std::cout << "Connecting to ACS controller at " << ipAddress << ":" << port << "..." << std::endl;

  // Connect to the controller
  if (controller.Connect(ipAddress, port)) {
    std::cout << "Successfully connected to ACS controller!" << std::endl;
    std::cout << "Starting position polling (30 seconds)..." << std::endl;
    std::cout << std::endl;  // Create an empty line where the position updates will appear

    // Poll the positions every second for 30 seconds
    const int pollDurationSeconds = 30;

    for (int i = 0; i < pollDurationSeconds; i++) {
      // Print positions on a single line that updates
      printCurrentPositions(controller, i + 1);

      // Optional: Try a small relative move on X axis after 5 seconds
      if (i == 5) {
        // Move to a new line to show the message
        std::cout << std::endl << std::endl;
        std::cout << "Executing relative move of 10 units on X axis..." << std::endl;
        std::cout << std::endl;

        controller.MoveRelative("X", 10.0, false);  // non-blocking move
      }

      // Wait 1 second before next poll
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Move to a new line before showing completion message
    std::cout << std::endl << std::endl;
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
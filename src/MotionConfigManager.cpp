// MotionConfigManager.cpp
#include "MotionConfigManager.h"
#include <fstream>
#include <iostream>
#include <queue>
#include <set>

MotionConfigManager::MotionConfigManager(const std::string& configFilePath) {
    LoadConfig(configFilePath);
}

void MotionConfigManager::LoadConfig(const std::string& filePath) {
    m_configFilePath = filePath;

    try {
        // Open and read the file
        std::ifstream file(filePath);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open configuration file: " + filePath);
        }

        // Parse JSON
        json configJson;
        file >> configJson;

        // Parse devices
        if (configJson.contains("MotionDevices") && configJson["MotionDevices"].is_object()) {
            for (auto& [deviceName, deviceJson] : configJson["MotionDevices"].items()) {
                MotionDevice device;
                device.Name = deviceName;

                if (deviceJson.contains("IsEnabled")) device.IsEnabled = deviceJson["IsEnabled"];
                if (deviceJson.contains("IpAddress")) device.IpAddress = deviceJson["IpAddress"];
                if (deviceJson.contains("Port")) device.Port = deviceJson["Port"];
                if (deviceJson.contains("Id")) device.Id = deviceJson["Id"];

                // Parse positions if they exist
                if (deviceJson.contains("Positions") && deviceJson["Positions"].is_object()) {
                    for (auto& [posName, posJson] : deviceJson["Positions"].items()) {
                      PositionStruct pos;
                        if (posJson.contains("x")) pos.x = posJson["x"];
                        if (posJson.contains("y")) pos.y = posJson["y"];
                        if (posJson.contains("z")) pos.z = posJson["z"];
                        if (posJson.contains("u")) pos.u = posJson["u"];
                        if (posJson.contains("v")) pos.v = posJson["v"];
                        if (posJson.contains("w")) pos.w = posJson["w"];

                        device.Positions[posName] = pos;
                    }
                }

                m_devices[deviceName] = device;
            }
        }

        // Parse graphs
        if (configJson.contains("Graphs") && configJson["Graphs"].is_object()) {
            for (auto& [graphName, graphJson] : configJson["Graphs"].items()) {
                Graph graph;

                // Parse nodes
                if (graphJson.contains("Nodes") && graphJson["Nodes"].is_array()) {
                    for (const auto& nodeJson : graphJson["Nodes"]) {
                        Node node;
                        if (nodeJson.contains("Id")) node.Id = nodeJson["Id"];
                        if (nodeJson.contains("Label")) node.Label = nodeJson["Label"];
                        if (nodeJson.contains("Device")) node.Device = nodeJson["Device"];
                        if (nodeJson.contains("Position")) node.Position = nodeJson["Position"];
                        if (nodeJson.contains("X")) node.X = nodeJson["X"];
                        if (nodeJson.contains("Y")) node.Y = nodeJson["Y"];

                        graph.Nodes.push_back(node);
                    }
                }

                // Parse edges
                if (graphJson.contains("Edges") && graphJson["Edges"].is_array()) {
                    for (const auto& edgeJson : graphJson["Edges"]) {
                        Edge edge;
                        if (edgeJson.contains("Id")) edge.Id = edgeJson["Id"];
                        if (edgeJson.contains("Source")) edge.Source = edgeJson["Source"];
                        if (edgeJson.contains("Target")) edge.Target = edgeJson["Target"];
                        if (edgeJson.contains("Label")) edge.Label = edgeJson["Label"];

                        // In the section where edge conditions are parsed:
                        if (edgeJson.contains("Conditions") && edgeJson["Conditions"].is_object()) {
                            const auto& condJson = edgeJson["Conditions"];
                            if (condJson.contains("RequiresOperatorApproval"))
                                edge.Conditions.RequiresOperatorApproval = condJson["RequiresOperatorApproval"];
                            if (condJson.contains("TimeoutSeconds"))
                                edge.Conditions.TimeoutSeconds = condJson["TimeoutSeconds"];
                            if (condJson.contains("IsBidirectional"))  // Add this condition
                                edge.Conditions.IsBidirectional = condJson["IsBidirectional"];
                        }

                        graph.Edges.push_back(edge);
                    }
                }

                m_graphs[graphName] = graph;
            }
        }

        // Parse settings
        if (configJson.contains("Settings") && configJson["Settings"].is_object()) {
            const auto& settingsJson = configJson["Settings"];
            if (settingsJson.contains("DefaultSpeed")) m_settings.DefaultSpeed = settingsJson["DefaultSpeed"];
            if (settingsJson.contains("DefaultAcceleration")) m_settings.DefaultAcceleration = settingsJson["DefaultAcceleration"];
            if (settingsJson.contains("LogLevel")) m_settings.LogLevel = settingsJson["LogLevel"];
            if (settingsJson.contains("AutoReconnect")) m_settings.AutoReconnect = settingsJson["AutoReconnect"];
            if (settingsJson.contains("ConnectionTimeout")) m_settings.ConnectionTimeout = settingsJson["ConnectionTimeout"];
            if (settingsJson.contains("PositionTolerance")) m_settings.PositionTolerance = settingsJson["PositionTolerance"];
        }

        // Validate the loaded configuration
        if (!ValidateConfig()) {
            throw std::runtime_error("Invalid configuration format");
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading configuration: " << e.what() << std::endl;
        throw;
    }
}

bool MotionConfigManager::ValidateConfig() {
    // Basic validation - can be expanded as needed
    if (m_devices.empty()) {
        std::cerr << "Warning: No devices found in configuration" << std::endl;
        return false;
    }

    return true;
}

const std::map<std::string, MotionDevice>& MotionConfigManager::GetAllDevices() const {
    return m_devices;
}

std::optional<std::reference_wrapper<const MotionDevice>> MotionConfigManager::GetDevice(const std::string& deviceName) const {
    auto it = m_devices.find(deviceName);
    if (it != m_devices.end()) {
        return std::cref(it->second);
    }
    return std::nullopt;
}

std::map<std::string, std::reference_wrapper<const MotionDevice>> MotionConfigManager::GetEnabledDevices() const {
    std::map<std::string, std::reference_wrapper<const MotionDevice>> enabledDevices;

    for (const auto& [name, device] : m_devices) {
        if (device.IsEnabled) {
            enabledDevices.emplace(name, std::cref(device));  // Using emplace instead of []
        }
    }

    return enabledDevices;
}

std::optional<std::reference_wrapper<const std::map<std::string, PositionStruct>>> MotionConfigManager::GetDevicePositions(const std::string& deviceName) const {
    auto deviceOpt = GetDevice(deviceName);
    if (!deviceOpt.has_value()) {  // Using has_value() for clarity
        return std::nullopt;
    }

    return std::cref(deviceOpt.value().get().Positions);  // Using value() to access the contained value
}

std::optional<std::reference_wrapper<const PositionStruct>> MotionConfigManager::GetNamedPosition(const std::string& deviceName, const std::string& positionName) const {
    auto positionsOpt = GetDevicePositions(deviceName);
    if (!positionsOpt.has_value()) {  // Using has_value() for clarity
        return std::nullopt;
    }

    const auto& positions = positionsOpt.value().get();  // Using value() to access the contained value
    auto it = positions.find(positionName);
    if (it != positions.end()) {
        return std::cref(it->second);
    }

    return std::nullopt;
}

const std::map<std::string, Graph>& MotionConfigManager::GetAllGraphs() const {
    return m_graphs;
}

std::optional<std::reference_wrapper<const Graph>> MotionConfigManager::GetGraph(const std::string& graphName) const {
    auto it = m_graphs.find(graphName);
    if (it != m_graphs.end()) {
        return std::cref(it->second);
    }
    return std::nullopt;
}

const Node* MotionConfigManager::GetNodeById(const std::string& graphName, const std::string& nodeId) const {
    auto graphOpt = GetGraph(graphName);
    if (!graphOpt.has_value()) {  // Using has_value() for clarity
        return nullptr;
    }

    const auto& graph = graphOpt.value().get();  // Using value() to access the contained value
    for (const auto& node : graph.Nodes) {
        if (node.Id == nodeId) {
            return &node;
        }
    }

    return nullptr;
}

std::vector<std::reference_wrapper<const Node>> MotionConfigManager::GetNodesByDevice(const std::string& graphName, const std::string& deviceName) const {
    std::vector<std::reference_wrapper<const Node>> result;

    auto graphOpt = GetGraph(graphName);
    if (!graphOpt.has_value()) {  // Using has_value() for clarity
        return result;
    }

    const auto& graph = graphOpt.value().get();  // Using value() to access the contained value
    for (const auto& node : graph.Nodes) {
        if (node.Device == deviceName) {
            result.emplace_back(std::cref(node));  // Using emplace_back instead of push_back
        }
    }

    return result;
}

std::vector<std::reference_wrapper<const Edge>> MotionConfigManager::GetEdgesBySource(const std::string& graphName, const std::string& sourceNodeId) const {
    std::vector<std::reference_wrapper<const Edge>> result;

    auto graphOpt = GetGraph(graphName);
    if (!graphOpt.has_value()) {  // Using has_value() for clarity
        return result;
    }

    const auto& graph = graphOpt.value().get();  // Using value() to access the contained value
    for (const auto& edge : graph.Edges) {
        if (edge.Source == sourceNodeId) {
            result.emplace_back(std::cref(edge));  // Using emplace_back instead of push_back
        }
    }

    return result;
}

std::vector<std::reference_wrapper<const Node>> MotionConfigManager::FindPath(
  const std::string& graphName,
  const std::string& startNodeId,
  const std::string& endNodeId) const {
  std::vector<std::reference_wrapper<const Node>> path;

  auto graphOpt = GetGraph(graphName);
  if (!graphOpt.has_value()) {
    return path;
  }

  const auto& graph = graphOpt.value().get();

  // Breadth-first search for the shortest path
  std::queue<std::vector<std::string>> nodeQueue;
  std::set<std::string> visited;

  // Start with the initial node
  nodeQueue.push({ startNodeId });
  visited.insert(startNodeId);

  while (!nodeQueue.empty()) {
    auto currentPath = nodeQueue.front();
    nodeQueue.pop();

    std::string currentNodeId = currentPath.back();

    // If we've reached the destination, build and return the path
    if (currentNodeId == endNodeId) {
      for (const auto& nodeId : currentPath) {
        const Node* node = GetNodeById(graphName, nodeId);
        if (node) {
          path.emplace_back(std::cref(*node));
        }
      }
      return path;
    }

    // Explore all edges
    for (const auto& edge : graph.Edges) {
      std::string targetNodeId;
      bool isValidEdge = false;

      // Check for direct and bidirectional connections
      if (edge.Source == currentNodeId) {
        targetNodeId = edge.Target;
        isValidEdge = true;
      }
      else if (edge.Conditions.IsBidirectional && edge.Target == currentNodeId) {
        targetNodeId = edge.Source;
        isValidEdge = true;
      }

      if (isValidEdge && visited.find(targetNodeId) == visited.end()) {
        visited.insert(targetNodeId);

        // Create a new path with the target node added
        auto newPath = currentPath;
        newPath.push_back(targetNodeId);
        nodeQueue.push(newPath);
      }
    }
  }

  // No path found
  return path;
}




const Settings& MotionConfigManager::GetSettings() const {
    return m_settings;
}

void MotionConfigManager::UpdateDevice(const std::string& deviceName, const MotionDevice& updatedDevice) {
    auto it = m_devices.find(deviceName);
    if (it != m_devices.end()) {
        // Preserve the name to ensure consistency
        MotionDevice device = updatedDevice;
        device.Name = deviceName;
        m_devices[deviceName] = device;
    }
    else {
        throw std::runtime_error("Device not found: " + deviceName);
    }
}

void MotionConfigManager::AddPosition(const std::string& deviceName, const std::string& positionName, const PositionStruct& position) {
    auto it = m_devices.find(deviceName);
    if (it != m_devices.end()) {
        it->second.Positions[positionName] = position;
    }
    else {
        throw std::runtime_error("Device not found: " + deviceName);
    }
}

void MotionConfigManager::AddDevice(const std::string& deviceName, const MotionDevice& device) {
    // Check if device already exists
    if (m_devices.find(deviceName) != m_devices.end()) {
        throw std::runtime_error("Device already exists: " + deviceName);
    }

    // Create a copy of the device
    MotionDevice newDevice = device;

    // Ensure the name is set correctly
    newDevice.Name = deviceName;

    // Add the device to the map
    m_devices[deviceName] = newDevice;
}

bool MotionConfigManager::DeleteDevice(const std::string& deviceName) {
    auto it = m_devices.find(deviceName);
    if (it == m_devices.end()) {
        // Device not found
        return false;
    }

    // Check if device is referenced in any graphs
    for (const auto& [graphName, graph] : m_graphs) {
        for (const auto& node : graph.Nodes) {
            if (node.Device == deviceName) {
                throw std::runtime_error("Cannot delete device '" + deviceName +
                    "' because it is referenced in graph '" + graphName + "'");
            }
        }
    }

    // If no references were found, delete the device
    m_devices.erase(it);
    return true;
}

bool MotionConfigManager::DeletePosition(const std::string& deviceName, const std::string& positionName) {
    auto deviceIt = m_devices.find(deviceName);
    if (deviceIt == m_devices.end()) {
        // Device not found
        return false;
    }

    // Get the positions map
    auto& positions = deviceIt->second.Positions;
    auto posIt = positions.find(positionName);
    if (posIt == positions.end()) {
        // Position not found
        return false;
    }

    // Check if position is referenced in any graphs
    for (const auto& [graphName, graph] : m_graphs) {
        for (const auto& node : graph.Nodes) {
            if (node.Device == deviceName && node.Position == positionName) {
                throw std::runtime_error("Cannot delete position '" + positionName +
                    "' from device '" + deviceName +
                    "' because it is referenced in graph '" +
                    graphName + "'");
            }
        }
    }

    // If no references were found, delete the position
    positions.erase(posIt);
    return true;
}

void MotionConfigManager::UpdateSettings(const Settings& newSettings) {
    // Update all settings
    m_settings = newSettings;
}

bool MotionConfigManager::SaveConfig(const std::string& filePath) {
    std::string savePath = filePath.empty() ? m_configFilePath : filePath;

    try {
        // Build the JSON structure
        json configJson;

        // Add devices
        for (const auto& [name, device] : m_devices) {
            json deviceJson;
            deviceJson["IsEnabled"] = device.IsEnabled;
            deviceJson["IpAddress"] = device.IpAddress;
            deviceJson["Port"] = device.Port;
            deviceJson["Id"] = device.Id;
            deviceJson["Name"] = device.Name;

            // Add positions
            json positionsJson;
            for (const auto& [posName, pos] : device.Positions) {
                json posJson;
                posJson["x"] = pos.x;
                posJson["y"] = pos.y;
                posJson["z"] = pos.z;
                posJson["u"] = pos.u;
                posJson["v"] = pos.v;
                posJson["w"] = pos.w;

                positionsJson[posName] = posJson;
            }

            if (!device.Positions.empty()) {
                deviceJson["Positions"] = positionsJson;
            }

            configJson["MotionDevices"][name] = deviceJson;
        }

        // Add graphs
        for (const auto& [name, graph] : m_graphs) {
            json graphJson;

            // Add nodes
            json nodesJson = json::array();
            for (const auto& node : graph.Nodes) {
                json nodeJson;
                nodeJson["Id"] = node.Id;
                nodeJson["Label"] = node.Label;
                nodeJson["Device"] = node.Device;
                nodeJson["Position"] = node.Position;
                nodeJson["X"] = node.X;
                nodeJson["Y"] = node.Y;

                nodesJson.push_back(nodeJson);
            }
            graphJson["Nodes"] = nodesJson;

            // Add edges
            json edgesJson = json::array();
            for (const auto& edge : graph.Edges) {
                json edgeJson;
                edgeJson["Id"] = edge.Id;
                edgeJson["Source"] = edge.Source;
                edgeJson["Target"] = edge.Target;
                edgeJson["Label"] = edge.Label;

                // In the section where edge conditions are saved:
                json conditionsJson;
                conditionsJson["RequiresOperatorApproval"] = edge.Conditions.RequiresOperatorApproval;
                conditionsJson["TimeoutSeconds"] = edge.Conditions.TimeoutSeconds;
                conditionsJson["IsBidirectional"] = edge.Conditions.IsBidirectional;  // Add this line

                edgeJson["Conditions"] = conditionsJson;
                edgesJson.push_back(edgeJson);
            }
            graphJson["Edges"] = edgesJson;

            configJson["Graphs"][name] = graphJson;
        }

        // Add settings
        json settingsJson;
        settingsJson["DefaultSpeed"] = m_settings.DefaultSpeed;
        settingsJson["DefaultAcceleration"] = m_settings.DefaultAcceleration;
        settingsJson["LogLevel"] = m_settings.LogLevel;
        settingsJson["AutoReconnect"] = m_settings.AutoReconnect;
        settingsJson["ConnectionTimeout"] = m_settings.ConnectionTimeout;
        settingsJson["PositionTolerance"] = m_settings.PositionTolerance;

        configJson["Settings"] = settingsJson;

        // Write to file
        std::ofstream file(savePath);
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file for writing: " + savePath);
        }

        file << configJson.dump(2);  // Pretty print with 2-space indentation
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error saving configuration: " << e.what() << std::endl;
        return false;
    }
}

void MotionConfigManager::UpdateGraph(const std::string& graphName, const Graph& updatedGraph) {
    // Check if the graph exists
    auto graphIter = m_graphs.find(graphName);
    if (graphIter == m_graphs.end()) {
        throw std::runtime_error("Graph not found: " + graphName);
    }

    // Update the graph
    m_graphs[graphName] = updatedGraph;

    // You might want to log this operation
    // If you have a logger in this class, use it here

    // Note: This only updates the in-memory representation
    // Call SaveConfig() to persist changes to the file
}
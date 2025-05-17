// MotionConfigManager.h
#pragma once

#include "MotionTypes.h"
#include <nlohmann/json.hpp>
#include <string>
#include <map>
#include <vector>
#include <optional>
#include <functional>

using json = nlohmann::json;

class MotionConfigManager {
public:
    // Constructor that takes a path to the JSON configuration file
    MotionConfigManager(const std::string& configFilePath);

    // Get all motion devices
    const std::map<std::string, MotionDevice>& GetAllDevices() const;

    // Get a specific device by name
    std::optional<std::reference_wrapper<const MotionDevice>> GetDevice(const std::string& deviceName) const;

    // Get all enabled devices
    std::map<std::string, std::reference_wrapper<const MotionDevice>> GetEnabledDevices() const;

    // Get all positions for a device
    std::optional<std::reference_wrapper<const std::map<std::string, PositionStruct>>> GetDevicePositions(const std::string& deviceName) const;

    // Get a specific named position for a device
    std::optional<std::reference_wrapper<const PositionStruct>> GetNamedPosition(const std::string& deviceName, const std::string& positionName) const;

    // Get all graphs
    const std::map<std::string, Graph>& GetAllGraphs() const;

    // Get a specific graph
    std::optional<std::reference_wrapper<const Graph>> GetGraph(const std::string& graphName) const;

    // Find a node by ID within a graph
    const Node* GetNodeById(const std::string& graphName, const std::string& nodeId) const;

    // Get all nodes associated with a specific device
    std::vector<std::reference_wrapper<const Node>> GetNodesByDevice(const std::string& graphName, const std::string& deviceName) const;

    // Get all edges with a specific source node
    std::vector<std::reference_wrapper<const Edge>> GetEdgesBySource(const std::string& graphName, const std::string& sourceNodeId) const;

    // Find a path between two nodes in a graph
    std::vector<std::reference_wrapper<const Node>> FindPath(const std::string& graphName, const std::string& startNodeId, const std::string& endNodeId) const;

    // Get global settings
    const Settings& GetSettings() const;

    // Update a device configuration
    void UpdateDevice(const std::string& deviceName, const MotionDevice& updatedDevice);

    // Add a new position to a device
    void AddPosition(const std::string& deviceName, const std::string& positionName, const PositionStruct& position);

    // New methods for adding, deleting devices and positions
    void AddDevice(const std::string& deviceName, const MotionDevice& device);
    bool DeleteDevice(const std::string& deviceName);
    bool DeletePosition(const std::string& deviceName, const std::string& positionName);

    // Updated settings
    void UpdateSettings(const Settings& newSettings);

    // Save the configuration to a file
    bool SaveConfig(const std::string& filePath = "");

    void UpdateGraph(const std::string& graphName, const Graph& updatedGraph);

private:
    // Parse and load the configuration
    void LoadConfig(const std::string& filePath);

    // Validate the configuration
    bool ValidateConfig();

    // Data members
    std::map<std::string, MotionDevice> m_devices;
    std::map<std::string, Graph> m_graphs;
    Settings m_settings;
    std::string m_configFilePath;
};
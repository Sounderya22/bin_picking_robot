#include "sensor_manager.hpp"
#include <iostream>

// Constructor
SensorManager::SensorManager(PerceptionSystem& perceptionSystem) : perceptionSystem_(perceptionSystem) {
    
}

// Destructor
SensorManager::~SensorManager() {}

std::vector<std::string> SensorManager::getSensorData() const {
    std::vector<std::string> data;
    return data;
}

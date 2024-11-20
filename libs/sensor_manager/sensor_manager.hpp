#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include "perception_system/perception_system.hpp"
#include <vector>
#include <string>

struct Sensor {
    std::string name;
};

// SensorManager class definition 
class SensorManager {
public:
    // Constructor
    SensorManager(PerceptionSystem& perceptionSystem);

    // Destructor
    ~SensorManager();

    // Method to retrieve data from all sensors
    std::vector<std::string> getSensorData() const;


private:
    std::vector<Sensor> sensors_;
    PerceptionSystem& perceptionSystem_;
};

#endif // SENSOR_MANAGER_HPP

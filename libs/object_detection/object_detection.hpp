#ifndef OBJECT_DETECTION_HPP
#define OBJECT_DETECTION_HPP

#include <vector>
#include <string>

struct Object {
    std::string name;
    int id;
};

class ObjectDetection {
public:
    // Constructor
    ObjectDetection();

    // Destructor
    ~ObjectDetection();

    // Runs inference to detect objects
    void runInference();

private:
    // List of detected objects
    std::vector<Object> detectedObjects;
};

#endif // OBJECT_DETECTION_HPP

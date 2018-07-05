/*! @file PositionOffsetMission.h
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_POSITIONOFFSETMISSION_H
#define MATRICE210_POSITIONOFFSETMISSION_H

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252       // pi/180

// System includes
#include <pthread.h>

// DJI OSDK includes
#include <dji_vehicle.hpp>
#include <dji_telemetry.hpp>

// User includes
#include "../timer.h"

class FlightController;
class PackageManager;

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class PositionOffsetMission {
private:
    FlightController* flightController{nullptr};
    Vehicle* vehicle{nullptr};
    // Offset values
    Vector3f offset{};              // Offset desired [m]
    double yawDesired{0.0};         // yaw desired [rad]
    // Mission parameters
    bool missionRunning{false};     // Mission running flag
    int missionTimeout{10000};      // Timeout to finish mission [ms]
    int controlFreq{50};            // Sent control frame frequency [Hz]
    int outOfBoundsLimit{10};       // TODO explain
    int withinBoundsRequirement{50};// Requirement cycles to reach target
    int speedFactor{2};             // Max speed factor
    float posThreshold{0.2};        // Position threshold [m]
    double yawThreshold{1.0};       // Yaw threshold [rad]
    // Missions values
    int elapsedTime{0};             // Elapsed time since mission beginning
    int withinBoundsCnt{0};         // Within bounds counter
    int outOfBoundsCnt{0};          // Out of bounds counter
    int brakeCounter{0};            // Brake counter
    // Subscription
    int pkgIndex{0};                // Package index used by subscription
    // Mutex
    static pthread_mutex_t missionRunning_mutex;
public:
    PositionOffsetMission(Vector3f* offset, float yawDesiredDeg,
                          float posThresholdInM, float yawThresholdInDeg);
    bool init(FlightController* fc);
    bool moveToPosition();
    void stopMission();
private:
    // Mission functions
    int getCycleTimeMs();
    int getOutOfBoundsTimeLimit();
    int getWithinBoundsTimeRequirement();
    void initMission();
    void setOffset(Vector3f* o, double y);
    void setThreshold(float pos, double yaw);
    void setMissionRunning(bool state);
    bool startGlobalPositionBroadcast();
    };


#endif //MATRICE210_POSITIONOFFSETMISSION_H

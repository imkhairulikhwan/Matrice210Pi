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
#include "../util/timer.h"

class FlightController;

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class PositionOffsetMission {
private:
    FlightController* flightController{nullptr};
    Vehicle* vehicle{nullptr};
    // Offset values
    Vector3f offset{};              // Offset desired [m]
    double targetYaw{0.0};         // yaw desired [rad]
    Telemetry::Vector3f positionToMove;
    // There is a deadband in position control
    // the z cmd is absolute height
    // while x and y are in relative
    float zDeadband{0.12};
    // Mission parameters
    bool missionRunning{false};     // TODO explain
    long missionTimeout{10000};      // Timeout to finish mission [ms]
    int controlFreq{50};            // Sent control frame frequency [Hz]
    int outOfBoundsLimit{10};       // TODO explain
    int withinBoundsRequirement{50};// Requirement cycles to reach target
    int speedFactor{2};             // Max speed factor
    float posThreshold{0.2};        // Position threshold [m]
    double yawThreshold{1.0};       // Yaw threshold [rad]
    // Missions values
    long startTime{0};              // Mission start time [ms]
    int withinBoundsCnt{0};         // Within bounds counter
    int outOfBoundsCnt{0};          // Out of bounds counter
    int brakeCnt{0};                // Brake counter
    // Subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
    int pkgIndex{0};                // Package index used by subscription
public:
    explicit PositionOffsetMission(FlightController* flightController);
    /**
     * Move aircraft of desired offset in m and set yaw in deg
     * @param offset            offset desired [m]
     * @param yaw               yaw desired [deg]
     * @param posThreshold      position threshold [m]
     * @param yawThreshold      yaw threshold [deg]
     * @return True if mission correctly initialized
     */
    bool move(Vector3f* offset, float yaw,
                          float posThreshold, float yawThreshold);
    bool update();
    unsigned int getCycleTimeMs();
private:
    bool moveToPosition();
    void stop();
    // Mission functions
    unsigned int getOutOfBoundsTimeLimit();
    unsigned int getWithinBoundsTimeRequirement();
    void resetMissionCounters();
    void setOffset(Vector3f* o, double y);
    void setThreshold(float posThreshold, double yawThreshold);
    bool startGlobalPositionBroadcast();
    };


#endif //MATRICE210_POSITIONOFFSETMISSION_H

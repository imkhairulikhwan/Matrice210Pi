/*! @file FlightController.h
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_FLIGHTCONTROLLER_HPP
#define MATRICE210_FLIGHTCONTROLLER_HPP

// System includes
#include <cmath>
#include <iostream>
#include <pthread.h>

// DJI OSDK includes
#include <dji_vehicle.hpp>
#include <dji_linux_helpers.hpp>

// User
#include "timer.h"

using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class PositionOffsetMission;
class VelocityMission;

class FlightController {
public:
    // Fight controller thread
    bool flightControllerThreadRunning;
    pthread_t flightControllerThreadID;
    pthread_attr_t flightControllerThreadAttr;
    static void* flightControllerThread(void* param);
    void startFlightControllerThread();
    void stopFlightControllerThread();
private:
    // Aircraft
    bool emergencyState;
    Vehicle* vehicle;
    LinuxSetup* linuxEnvironment;
    // Moving modes
    enum movingMode_ {
        WAIT,
        STOP,
        VELOCITY,
        POSITION
    } movingMode;
    // Velocity and Position mission
    PositionOffsetMission* positionOffsetMission;
    VelocityMission* velocityMission;
    // Mutex
    static pthread_mutex_t movingMode_mutex;
    static pthread_mutex_t emergencyState_mutex;
    static pthread_mutex_t sendDataToMSDK_mutex;
public :
    FlightController();
    void setupVehicle(int argc, char** argv);
    // Mobile-On board communication
    void sendDataToMSDK(uint8_t* data, uint8_t length);
    // Movement control
    bool monitoredTakeoff(int timeout = 1);
    bool monitoredLanding(int timeout = 1);

    void moveByPositionOffset(Vector3f *offset, float yawDesiredDeg,
                              float posThresholdInM = 0.2,
                              float yawThresholdInDeg = 1.0);
    void moveByVelocity(Vector3f *velocity, float yaw);
    void stopAircraft();
    // Emergency safe ObSdk call
    void positionAndYawCtrl(Vector3f* position, float32_t yaw);
    void velocityAndYawRateCtrl(Vector3f *velocity, float32_t yaw);
    // Emergency
    void emergencyStop();
    void emergencyRelease();
    void setEmergencyState( bool state);
    bool isEmergencyState() { return emergencyState; }
    // Getters and setters functions // TODO Mutex !
    Vehicle* getVehicle() { return vehicle; }
    movingMode_ getMovingMode() { return movingMode; }
    void setMovingMode(movingMode_ mode);

// Static functions
public:
/*! Very simple calculation of local NED offset between
 *  two pairs of GPS coordinates.
 * Accurate when distances are small.
!*/
    static void localOffsetFromGpsOffset(Vector3f& deltaNed,
                                         void* target, void* origin);
    /**
     * toEulerAngle
     * @param quaternionData quaternion
     * @return Rad yaw value
     */
    static Telemetry::Vector3f toEulerAngle(void* quaternionData);
};

#endif // MATRICE210_FLIGHTCONTROLLER_HPP
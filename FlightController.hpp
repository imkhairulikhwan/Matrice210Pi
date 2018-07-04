/*! @file FlightController.hpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_FLIGHTCONTROLLER_HPP
#define MATRICE210_FLIGHTCONTROLLER_HPP

// System Includes
#include <cmath>
#include <iostream>

// DJI OSDK includes
#include <dji_status.hpp>
#include <dji_vehicle.hpp>
#include <pthread.h>
// Helpers
#include "dji_linux_helpers.hpp"

using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252       // pi/180

class FlightController {
public:
    // Fight controller thread
    bool flightControllerThreadRunning;
    pthread_t flightControllerThreadID;
    pthread_attr_t flightControllerThreadAttr;
    static void* flightControllerThread(void* param);
private:
    //
    bool emergencyState;
    Vehicle* vehicle;
    LinuxSetup* linuxEnvironment;
    // Velocity
    bool velocityActivated;
    Vector3f velocity;
    float32_t yaw;
public :
    FlightController();
    void setupVehicle(int argc, char** argv);
    // Mobile-On board communication
    void sendDataToMSDK(uint8_t* data, uint8_t length);
    // Position control
    bool monitoredTakeoff(int timeout = 1);
    bool monitoredLanding(int timeout = 1);

    bool moveByPositionOffset(Vector3f *offset, float yawDesired,
                              float posThresholdInM = 0.2,
                              float yawThresholdInDeg = 1.0);
    void moveByVelocity(Vector3f *velocity, float yaw);
    void stopVelocity();
    void startFlightControllerThread();
    void stopFlightControllerThread();
    // Emergency safe OBSDK call
    void positionAndYawCtrl(Vector3f* position, float32_t yaw);
    void velocityAndYawRateCtrl(Vector3f *velocity, float32_t yaw);
    // Emergency
    void emergencyStop();
    void emergencyRelease();
    bool isEmergencyState() {return emergencyState; }
    // Setup functions
    LinuxSetup* getLinuxSetup() { return linuxEnvironment; }
    Vehicle* getVehicle() {return vehicle; }
    // Getters
    Vector3f* getVelocity() { return &velocity; }
    float32_t getYaw() { return yaw; }
    bool isvelocityActivated() { return velocityActivated; }

private:
    /*! Very simple calculation of local NED offset between two pairs of GPS
 * coordinates.
 *
 * Accurate when distances are small.
!*/
    void localOffsetFromGpsOffset(Vector3f& deltaNed, void* target,
                                  void* origin);

    Telemetry::Vector3f toEulerAngle(void* quaternionData);
    bool startGlobalPositionBroadcast();
};

#endif // MATRICE210_FLIGHTCONTROLLER_HPP
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

class PositionMission;
class VelocityMission;
class PositionOffsetMission;

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
        POSITION_OFFSET,
        POSITION
    } movingMode;
    // Missions
    PositionMission* positionMission;
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
    void sendDataToMSDK(uint8_t* data, size_t length);
    // Movement control
    /**
     * Monitored take-off. Blocking call
     * @param timeout
     * @return true if success
     */
    bool monitoredTakeoff(int timeout = 1);
    /**
     *  Monitored landing. Blocking call
     * @param timeout
     * @return true if success
     */
    bool monitoredLanding(int timeout = 1);
    /**
     * Control the position and yaw angle of the vehicle.
     * Here to try DJI SDK positionAndYawCtrl() method
     * To move aircraft of a desired offset please use moveByPositionOffset()
     * @param offset [m]
     * @param yaw [deg}
     */
    void moveByPosition(Vector3f *offset, float yaw);
    /**
     * Velocity Control. Allows user to set a velocity vector.
     * The aircraft will move as described by vector until stopAircraft() call.
     * @param velocity [deg/s]
     * @param yaw [deg/s]
     */
    void moveByVelocity(Vector3f *velocity, float yaw);
    /**
     * Position Control. Allows user to set an offset from current location.
     * The aircraft will move to that position and stay there.
     * @param offset [m]
     * @param yaw [deg]
     * @param posThreshold [m]
     * @param yawThreshold [m]
     */
    void moveByPositionOffset(Vector3f *offset, float yaw,
                              float posThreshold = 0.2,
                              float yawThreshold = 1.0);
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
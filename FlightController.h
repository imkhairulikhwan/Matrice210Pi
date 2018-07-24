/*! @file FlightController.h
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_FLIGHTCONTROLLER_HPP
#define MATRICE210_FLIGHTCONTROLLER_HPP

// System includes
#include <pthread.h>

// DJI OSDK includes
#include <dji_vehicle.hpp>

using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class LinuxSetup;
class PositionMission;
class VelocityMission;
class PositionOffsetMission;

class FlightController {
public:
    void launchFlightControllerThread();
    void stopFlightControllerThread();
private:
    // Fight controller thread
    bool flightControllerThreadRunning;
    pthread_t flightControllerThreadID;
    pthread_attr_t flightControllerThreadAttr;
    static void* flightControllerThread(void* param);
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
    static pthread_mutex_t sendDataToMSDK_mutex;
    static pthread_mutex_t emergencyState_mutex;
    static pthread_mutex_t movingMode_mutex;
public :
    FlightController();
    void setupVehicle(int argc, char** argv);
    // Mobile-On board communication
    void sendDataToMSDK(const uint8_t* data, size_t length) const;
    // Movement control
    /**
     * Monitored take-off. Blocking call
     * @param timeout
     * @return true if success
     */
    bool monitoredTakeoff(int timeout = 1) const;
    /**
     *  Monitored landing. Blocking call
     * @param timeout
     * @return true if success
     */
    bool monitoredLanding(int timeout = 1) const;
    /**
     * Control the position and yaw angle of the vehicle.
     * Here to try DJI SDK positionAndYawCtrl() method
     * To move aircraft of a desired offset please use moveByPositionOffset()
     * @param offset [m]
     * @param yaw [deg}
     */
    void moveByPosition(const Vector3f *offset, float yaw);
    /**
     * Velocity Control. Allows user to set a velocity vector.
     * The aircraft will move as described by vector until stopAircraft() call.
     * @param velocity [deg/s]
     * @param yaw [deg/s]
     */
    void moveByVelocity(const Vector3f *velocity, float yaw);
    /**
     * Position Control. Allows user to set an offset from current location.
     * The aircraft will move to that position and stay there.
     * @param offset [m]
     * @param yaw [deg]
     * @param posThreshold [m]
     * @param yawThreshold [m]
     */
    void moveByPositionOffset(const Vector3f *offset, float yaw,
                              float posThreshold = 0.2,
                              float yawThreshold = 1.0);
    void stopAircraft();
    // Emergency safe ObSdk call
    void positionAndYawCtrl(const Vector3f* position, float yaw) const;
    void velocityAndYawRateCtrl(const Vector3f *velocity, float yaw) const;
    // Emergency
    void emergencyStop();
    void emergencyRelease();
    void setEmergencyState(bool state);
    bool isEmergencyState() const { return emergencyState; }
    // Getters and setters functions
    const Vehicle* getVehicle() const { return vehicle; }
    movingMode_ getMovingMode() const { return movingMode; }
    void setMovingMode(movingMode_ mode);

// Static functions
public:
/*! Very simple calculation of local NED offset between
 *  two pairs of GPS coordinates.
 * Accurate when distances are small.
!*/
    static void localOffsetFromGpsOffset(Vector3f& deltaNed,
                                         const Telemetry::GPSFused *subscriptionTarget,
                                         const Telemetry::GPSFused *subscriptionOrigin);
    /**
     * toEulerAngle
     * @param quaternionData quaternion
     * @return Rad yaw value
     */
    static Telemetry::Vector3f toEulerAngle(const Telemetry::Quaternion *quaternion);
};

#endif // MATRICE210_FLIGHTCONTROLLER_HPP
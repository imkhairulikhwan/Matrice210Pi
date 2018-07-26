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

namespace M210 {
    class Watchdog;
    class Emergency;
    class MonitoredMission;
    class PositionMission;
    class VelocityMission;
    class PositionOffsetMission;
    class WaypointMission;

    class FlightController {
    public:
        void launchFlightControllerThread();

        void stopFlightControllerThread();      // unused
    private:
        // Fight controller thread
        bool flightControllerThreadRunning;
        pthread_t flightControllerThreadID;
        pthread_attr_t flightControllerThreadAttr;

        static void *flightControllerThread(void *param);

        // Aircraft
        Emergency *emergency;
        Vehicle *vehicle;
        LinuxSetup *linuxEnvironment;
        Watchdog *watchdog;
        // Moving modes
        enum movingMode_ {
            WAIT,
            STOP,
            VELOCITY,
            POSITION_OFFSET,
            POSITION
        } movingMode;
        // Missions
        M210::MonitoredMission *monitoredMission;
        M210::PositionMission *positionMission;
        M210::PositionOffsetMission *positionOffsetMission;
        M210::VelocityMission *velocityMission;
        M210::WaypointMission *waypointMission;
        // Mutex
        static pthread_mutex_t sendDataToMSDK_mutex;
        static pthread_mutex_t movingMode_mutex;
    public :
        FlightController();

        ~FlightController();

        void setupVehicle(int argc, char **argv);

        // Mobile-On board communication
        void sendDataToMSDK(const uint8_t *data, size_t length) const;

        // Movement control
        bool takeoff();

        bool landing();

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
        // TODO Temporary tests, remove
        void waypointMissionAction(unsigned numWaypoints);

        // Stop and emergency
        void stopAircraft();

        void emergencyStop();

        void emergencyRelease();

        // Emergency safe ObSdk call
        void positionAndYawCtrl(const Vector3f *position, float yaw);

        void velocityAndYawRateCtrl(const Vector3f *velocity, float yaw);

        // Getters and setters functions
        Vehicle *getVehicle() const { return vehicle; }

        movingMode_ getMovingMode() const { return movingMode; }

        Watchdog *getWatchdog() const { return watchdog; }

        void setMovingMode(movingMode_ mode);

// Static functions
    public:
        // TODO Global broadcast manager implementation !
        static bool startGlobalPositionBroadcast(Vehicle *vehicle);
        /*! Very simple calculation of local NED offset between
         *  two pairs of GPS coordinates.
         * Accurate when distances are small.
        !*/
        static void localOffsetFromGpsOffset(Vector3f &deltaNed,
                                             const Telemetry::GPSFused *subscriptionTarget,
                                             const Telemetry::GPSFused *subscriptionOrigin);

        /**
         * toEulerAngle
         * @param quaternionData quaternion
         * @return Rad yaw value
         */
        static Telemetry::Vector3f toEulerAngle(const Telemetry::Quaternion *quaternion);
    };
}

#endif // MATRICE210_FLIGHTCONTROLLER_HPP
/*! @file FlightController.h
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 *  @brief This class handles all aircraft status, actions and missions.
 *  It contains a dedicated thread that implements a state machine. The goal is to
 *  continuously send order to the aircraft when a mission is running.
 *  Many types of missions are existing, for details see Missions folder
 */

#ifndef MATRICE210_FLIGHTCONTROLLER_HPP
#define MATRICE210_FLIGHTCONTROLLER_HPP

#define R_EARTH (double)6378137.0   /*!< Earth equatorial radius [m] */

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
    private:
        // Flight controller thread
        bool flightControllerThreadRunning;         /*!< Flight controller thread state */
        pthread_t flightControllerThreadID;         /*!< Flight controller thread id */
        pthread_attr_t flightControllerThreadAttr;  /*!< Flight controller thread attributes */
        static void *flightControllerThread(void *param); /*!< Flight controller thread, state machine */
        enum SMState_ {                             /*!< State machine states, used by flight controller thread */
            WAIT,
            STOP,
            VELOCITY,
            POSITION_OFFSET,
            POSITION
        } SMState;

        // Aircraft
        LinuxSetup *linuxEnvironment;   /*!< Pointer to used linux environment */
        Vehicle *vehicle;               /*!< Pointer to used vehicle */
        Emergency *emergency;           /*!< Emergency state */
        Watchdog *watchdog;             /*!< Watchdog */

        // Missions
        M210::MonitoredMission *monitoredMission;               /*!< Monitored mission */
        M210::PositionMission *positionMission;                 /*!< Position mission */
        M210::PositionOffsetMission *positionOffsetMission;     /*!< Position offset mission */
        M210::VelocityMission *velocityMission;                 /*!< Velocity mission */
        M210::WaypointMission *waypointMission;                 /*!< Waypoints mission */
        // Mutex
        static pthread_mutex_t sendDataToMSDK_mutex;            /*!< Ensure that data are sent one by one to the mobile */
        static pthread_mutex_t smState_mutex;                   /*!< Protect state machine states modification */
    public :
        /**
         * Initialize flight controller and create mission
         */
        FlightController();

        /**
         *  Delete missions and flight controller attributes
         */
        ~FlightController();

        /**
         * Configure linux environment, get vehicle and
         * obtain control authority
         * @param argc main parameter
         * @param argv main parameter
         */
        void setupVehicle(int argc, char **argv);

        /**
         * Launch flight controller thread
         */
        void launchFlightControllerThread();

        /**
         * Send data to mobile SDK
         * @param data Pointer to data to send
         * @param length Length of data to send
         */
        void sendDataToMSDK(const uint8_t *data, size_t length) const;

        // Movement control
        /**
         * Monitored take-off blocking call
         * @return True if take-off success
         */
        bool takeOff();

        /**
         * Monitored landing blocking call.
         * @return True is landing success
         *
         */
        bool landing();

        /**
         * Control the position and yaw angle of the vehicle.
         * Here to try DJI SDK positionAndYawCtrl() method
         * To move aircraft of a desired offset please use moveByPositionOffset()
         * @param offset Relative offset vector to move [m]
         * Vector is relative to the ground
         * x face to north, y face to east, z face to sky
         * TODO explain difference with PositionOffset
         * @param yaw Absolute yaw angle to set [deg}
         */
        void moveByPosition(const Vector3f *offset, float yaw);

        /**
         * Velocity Control. Allows user to set a velocity vector.
         * The aircraft will move as described by vector until stopAircraft() is called.
         * @param velocity Absolute velocity vector to set [m/s]
         * Vector is relative to the ground
         * x face to north, y face to east, z face to sky
         * @param yaw Absolute yaw rate to set [deg/s]
         */
        void moveByVelocity(const Vector3f *velocity, float yaw);

        /**
         * Position Control. Allows user to set an offset from current location.
         * The aircraft will move to that position and stay there.
         * @param offset Relative offset vector to move [m]
         * Vector is relative to the ground
         * x face to north, y face to east, z face to sky
         * @param yaw Absolute yaw angle to set [deg]
         * @param posThreshold Position threshold used by mission to consider position as reached [m]
         * @param yawThreshold Angle threshold used by mission to consider angle as reached [deg]
         */
        void moveByPositionOffset(const Vector3f *offset, float yaw,
                                  float posThreshold = 0.2,
                                  float yawThreshold = 1.0);
        /**
         * Modify action flow of the waypoints mission
         * @param task Task to do, value of Action::MissionAction (Action.h) structure
         */
        void waypointsMissionAction(unsigned task);

        // Stop and emergency
        /**
         * Stop aircraft
         * Stop flight controller state machine and stop all current missions
         */
        void stopAircraft();

        /**
         *  Set flight controller emergency state and stop aircraft
         */
        void emergencyStop();

        /**
         * Reset flight controller emergency state
         */
        void emergencyRelease();

        // Emergency safe ObSdk call
        /**
         * Control the velocity and yaw rate of the aircraft
         * On-board SDK method call with safety verification (emergency and watchdog states)
         * This method must be used by all missions instead of direct call to vehicle method
         * @param velocity Absolute velocity vector to set [m/s]
         * Vector is relative to the ground
         * x face to north, y face to east, z face to sky
         * @param yaw Absolute yaw rate to set [deg/s]
         */
        void velocityAndYawRateCtrl(const Vector3f *velocity, float yaw);

        /**
         * Control the position and yaw angle of the vehicle.
         * On-board SDK method call with safety verification (emergency and watchdog states)
         * This method must be used by all missions instead of direct call to vehicle method
         * @param position Relative position vector to move [m]
         * Vector is relative to the ground
         * x face to north, y face to east, z face to sky
         * @param yaw Absolute yaw angle to set [deg}
         * TODO Explain why multiple calls are needed
         */
        void positionAndYawCtrl(const Vector3f *position, float yaw);

        /** Getters and setters functions */
        Vehicle *getVehicle() const { return vehicle; }

        SMState_ getMovingMode() const { return SMState; }

        Watchdog *getWatchdog() const { return watchdog; }

        void setMovingMode(SMState_ mode);

    // Static functions
    public:
        // TODO Global broadcast manager implementation !
        /**
         * Start broadcast send of position from aircaft
         * @param vehicle Vehicle concerned
         * @return True if broadcast successfully started
         */
        static bool startGlobalPositionBroadcast(Vehicle *vehicle);

        /**
         * Calculate local NED offset between two pairs of GPS coordinates.
         * Accurate when distances are small.
         * @param deltaNed Float vector used to return offset
         * @param target Target GPS coordinates
         * @param origin Origin GPS coordinates
         */
        static void localOffsetFromGpsOffset(Vector3f &deltaNed,
                                             const Telemetry::GPSFused *target,
                                             const Telemetry::GPSFused *origin);

        /**
         * Calculate euler angle from quaternion data
         * @param quaternionData quaternion
         * @return Rad euler angle
         */
        static Telemetry::Vector3f toEulerAngle(const Telemetry::Quaternion *quaternion);
    };
}

#endif // MATRICE210_FLIGHTCONTROLLER_HPP
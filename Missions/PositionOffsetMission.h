/*! @file PositionOffsetMission.h
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 *  @brief
 *  Calculate the inputs to send the position controller.
 *  Basic receding setpoint position control with the setpoint always 2m away
 *  from the current position - until aircraft get within a threshold of the goal.
 *  From that point on, the remaining distance is sent as the setpoint.
 */

 // TODO More explanation, test setPointDistance modifications, test moveToPosition
 // TODO get relative time on update method !

#ifndef MATRICE210_POSITIONOFFSETMISSION_H
#define MATRICE210_POSITIONOFFSETMISSION_H

#define DEG2RAD 0.01745329252       /*!< Deg to rad factor (pi/180) */

// DJI OSDK includes
#include <dji_vehicle.hpp>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

namespace M210 {
    class FlightController;

    class PositionOffsetMission {
    private:
        FlightController *flightController{nullptr};
        Vehicle *vehicle{nullptr};
        // Offset values
        Vector3f offset{};              // Offset desired [m]
        double targetYaw{0.0};         // yaw desired [rad]
        Telemetry::Vector3f positionToMove;
        // There is a deadband in position control
        // the z cmd is absolute height
        // while x and y are in relative
        float zDeadband{0.12};
        // Mission parameters
        bool missionRunning{false};     /*!< Prevent mission to be launched multiples times */
        long missionTimeout{10000};     /*!< Timeout to finish mission [ms] */
        int controlFreq{50};            /*!< Sent control frame frequency [Hz] */
        int outOfBoundsLimit{10};       /*!< Limit cycles to consider aircraft as out of bounds [cycles] */
        int withinBoundsRequirement{50};/*!< Requirement cycles to reach target [cycles] */
        int setPointDistance{2};        /*!< Set point distance [m] */
        float posThreshold{0.2};        /*!< Position threshold [m] */
        double yawThreshold{1.0};       /*!< Yaw threshold [rad] */
        // Missions values
        long startTime{0};              /*!< Mission start time [ms] */
        int withinBoundsCnt{0};         /*!< Within bounds counter [ms] */
        int outOfBoundsCnt{0};          /*!< Out of bounds counter [ms]*/
        int brakeCnt{0};                /*!< Brake counter [ms] */
        // Subscription
        Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
        int pkgIndex{0};                /*!< Package index used by subscription */
    public:
        explicit PositionOffsetMission(FlightController *flightController);

        /**
         * Allows user to move aircraft of an offset from current location.
         * The aircraft will move to that position and stay there.
         * @param offset Relative offset vector to move [m]
         * Vector is relative to the ground
         * x face to north, y face to east, z face to sky
         * @param yaw Absolute yaw angle to set [deg]
         * @param posThreshold Position threshold used by mission to consider position as reached [m]
         * @param yawThreshold Angle threshold used by mission to consider angle as reached [deg]
         * @return true if mission is correctly initialized
         */
        bool move(const Vector3f *offset, float yaw,
                  float posThreshold, float yawThreshold);

        /**
         * Has to be called continuously
         * @return true if destination is reached, false otherwise
         */
        bool update();

        unsigned int getCycleTimeMs() const;

    private:
        /**
         * Stop aircraft and mission
         */
        void stop();

        // Mission functions
        /**
         * Get out of bounds time limit depending on outOfBoundsLimit and cycle time
         * @return out of bounds time limit [ms]
         */
        unsigned int getOutOfBoundsTimeLimit() const;

        /**
         * Get within time requirement depending on withinBoundsRequirement and cycle time
         * @return within time requirement [ms]
         */
        unsigned int getWithinBoundsTimeRequirement() const;

        /**
         * Reset all mission time counters
         */
        void resetMissionCounters();

        /**
         * Private setter
         * @param offset Relative offset vector to move [m]
         * @param yaw Absolute yaw angle to set [deg]
         */
        void setOffset(const Vector3f *offset, double yaw);

        /**
         * Private setter
         * @param posThreshold Position threshold [m]
         * @param yawThreshold Angle threshold [deg]
         */
        void setThreshold(float posThreshold, double yawThreshold);
    };
}


#endif //MATRICE210_POSITIONOFFSETMISSION_H

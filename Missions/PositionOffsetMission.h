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

#ifndef MATRICE210_POSITIONOFFSETMISSION_H
#define MATRICE210_POSITIONOFFSETMISSION_H

// DJI OSDK includes
#include <dji_vehicle.hpp>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

#define RAD2DEG 57.2957795129       // 1 /(pi/180)

namespace M210 {
    class FlightController;

    class PositionOffsetMission {
    private:
        FlightController *flightController{nullptr};
        Vehicle *vehicle{nullptr};
        // Offset values
        Vector3f targetOffset{};         /*!< Offset desired [m] */
        float targetYaw{0.0};           /*!< yaw desired [deg] */
        Telemetry::Vector3f positionToMove;
        // There is a deadband in position control
        // the z cmd is absolute height
        // while x and y are in relative
        float zDeadband{0.12};
        // Mission parameters
        bool missionRunning{false};         /*!< Prevent mission to be launched multiples times */
        long missionTimeout{10000};         /*!< Timeout to finish mission [ms] */
        long outOfBoundsLimit{200};         /*!< Limit time to consider aircraft as out of bounds [ms] */
        long withinBoundsRequirement{1000}; /*!< Requirement time to consider target as reached [ms] */
        int setPointDistance{2};            /*!< Set point distance [m] */
        float posThreshold{0.2};            /*!< Position threshold [m] */
        double yawThreshold{1.0};           /*!< Yaw threshold [deg] */
        // Missions values
        long startTime{0};              /*!< Mission absolute start time [ms] */
        long lastUpdateTime{0};         /*!< Last absolute time update method was called [ms] */
        long withinBoundsCnt{0};        /*!< Within bounds counter [ms] */
        long outOfBoundsCnt{0};         /*!< Out of bounds counter [ms]*/
        long brakeCnt{0};               /*!< Brake counter [ms] */
        // Subscription
        Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originGpsPosition;
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
    private:
        /**
         * Stop aircraft and mission.
         * Send brake order multiple times
         */
        void stop();

        // Mission functions
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

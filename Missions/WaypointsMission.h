/*! @file WaypointsMission.h
 *  @version 1.0
 *  @date Jul 26 2018
 *  @author Jonathan Michel
 *  @brief This class allows user to add GPS coordinates to a
 *  waypoints mission.
 *
 *  The goal is to make autonomous flight with a sequence of
 *  coordinates that the aircraft will reach.
 *  Once the missions played, it can be paused/resumed and stopped.
 *  Current implementation use current position of the aircraft as
 *  the new point to add to the waypoints mission
 */

#ifndef MATRICE210_WAYPOINTSMISSION_H
#define MATRICE210_WAYPOINTSMISSION_H

#include <pthread.h>
#include <vector>

#include "dji_vehicle.hpp"

using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

#define MAX_WAYPOINTS 15 /*<! Maximum waypoints list size, DJI maximum waypoints number is 99 */

namespace M210 {
    class FlightController;

    class WaypointMission {
    private:
        FlightController* flightController;                 /*!< Flight controller concerned by the mission */
        vector<DJI::OSDK::WayPointSettings> waypointsList;  /*!< List of all mission waypoints  */
        WayPointInitSettings waypointsSettings;             /*!< Settings of waypoints mission */
        uint8_t index;                                      /*!< Waypoints index, index + 1 indicates number of waypoints saved */
        static pthread_mutex_t mutex;                       /*!< Mutex to protect index and waypointsList */
        /**
         * Initialize waypoint settings with default values
         * @param wp Waypoint settings to initialize
         */
        void setWaypointDefaults(WayPointSettings *wp);
        /**
         * Initialize waypoint mission settings with default values
         * @param wp Waypoint mission settings to initialize
         */
        void setWaypointSettingsDefaults(WayPointInitSettings *wp);
        bool isMissionInitialized();
        /**
         * Get current position (long, lat, alt, hei) of aircraft
         * @param position GlobalPosition structure where return position
         */
        void currentPosition(GlobalPosition &position);
        // Mission functions
        /**
         * Add current position to waypoints mission list
         * @return false if list is filled, true otherwise
         */
        bool add();
        /**
         * Delete all saved waypoints
         */
        void reset();
        /**
         * Initialize waypoints mission, upload waypoints list
         * and start mission
         * @return true is mission has successfully started,
         * false if a problem occurred
         */
        bool start();
        /**
         * Pause waypoints mission
         * @return true is mission has successfully been paused,
         * false if a problem occurred
         */
        bool pause();
        /**
         * Pause waypoints mission
         * @return true is mission has successfully been paused,
         * false if a problem occurred
         */
        bool resume();
        /**
         * Stop waypoints mission
         * @return true is mission has successfully been stopped,
         * false if a problem occurred
         */
        bool stop();

    public:
        /**
         * Waypoints mission constructor
         * @param flightController The flight controller concerned by the mission
         */
        explicit WaypointMission(FlightController* flightController);
        /**
         * Modify action flow with a mission task
         * @param task Task to do, value of Action::MissionAction structure
         */
        void action(unsigned int task);
    };
}

#endif //MATRICE210_WAYPOINTSMISSION_H
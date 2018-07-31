/*! @file WaypointMission.h
 *  @version 1.0
 *  @date Jul 26 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_WAYPOINTSMISSION_H
#define MATRICE210_WAYPOINTSMISSION_H

#include <pthread.h>
#include <vector>

#include "dji_vehicle.hpp"

using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

#define MAX_WAYPOINTS 15

namespace M210 {
    class FlightController;

    class WaypointMission {
    private:
        FlightController* flightController;
        vector<DJI::OSDK::WayPointSettings> waypointsList;
        WayPointInitSettings waypointsSettings;
        uint8_t index;
        // Mutex to protect index and waypointsList
        static pthread_mutex_t mutex;
        // Default struct constructors
        void setWaypointDefaults(WayPointSettings *wp);
        void setWaypointSettingsDefaults(WayPointInitSettings *fdata);
        bool isMissionInitialized();
        //
        void currentPosition(GlobalPosition &position);
        // Mission functions
        void add();
        bool start();
        void reset();
        bool pause();
        bool resume();
        bool stop();
    public:
        explicit WaypointMission(FlightController* flightController);
        void action(unsigned int task);
    };
}


#endif //MATRICE210_WAYPOINTSMISSION_H

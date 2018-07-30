/*! @file WaypointMission.h
 *  @version 1.0
 *  @date Jul 26 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_WAYPOINTSMISSION_H
#define MATRICE210_WAYPOINTSMISSION_H

#include <vector>

#include "dji_vehicle.hpp"

using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

#define MAX_WAYPOINTS 8

namespace M210 {
    class FlightController;

    class WaypointMission {
    private:
        FlightController* flightController;
        vector<DJI::OSDK::WayPointSettings> waypointsList;
        WayPointInitSettings waypointsSettings;
        uint8_t index;

        // Default struct constructors
        void setWaypointDefaults(WayPointSettings *wp);
        void setWaypointSettingsDefaults(WayPointInitSettings *fdata);
        //
        void currentPosition(GlobalPosition &position);
    public:
        explicit WaypointMission(FlightController* flightController);

        // Mission
        void add();
        bool start();
        void reset();
        // TODO implement in MOC
        bool pause();
        bool resume();
        bool stop();
        // TODO better !
        void action(unsigned int task);

        bool isMissionInitialized();
    };
}


#endif //MATRICE210_WAYPOINTSMISSION_H

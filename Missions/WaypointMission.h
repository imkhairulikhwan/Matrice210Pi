/*! @file WaypointMission.h
 *  @version 1.0
 *  @date Jul 26 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_WAYPOINTMISSION_H
#define MATRICE210_WAYPOINTMISSION_H

#include <vector>

#include "dji_vehicle.hpp"

using namespace std;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

namespace M210 {
    class FlightController;

    class WaypointMission {
    private:
        FlightController* flightController;
        GlobalPosition savedPosition;

        bool polygonExample(uint8_t numWaypoints);
        bool start();
        bool stop();
        bool pause();
        bool resume();
        void setWaypointDefaults(WayPointSettings *wp);

        void setWaypointInitDefaults(WayPointInitSettings *fdata);

        vector<WayPointSettings> createWaypoints( int numWaypoints,
                                                  float64_t distanceIncrement, float start_alt);

        vector<WayPointSettings> generateWaypointsPolygon( WayPointSettings *start_data,
                                                           float64_t increment, int num_wp);

        void uploadWaypoints(vector<WayPointSettings> &wp_list);
    public:
        explicit WaypointMission(FlightController* flightController);
        void action(unsigned id);
        // Tests
        void currentPosition(GlobalPosition &position);
        void returnToSavedPosition();

        void saveCurrentPosition();
    };
}


#endif //MATRICE210_WAYPOINTMISSION_H

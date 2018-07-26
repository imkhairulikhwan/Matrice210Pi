/*! @file WaypointMission.h
 *  @version 1.0
 *  @date Jul 26 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_WAYPOINTMISSION_H
#define MATRICE210_WAYPOINTMISSION_H

#include <vector>

#include "dji_vehicle.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

namespace M210 {
    class FlightController;

    class WaypointMission {
    private:
        FlightController* flightController;
        bool start();
        void setWaypointDefaults(WayPointSettings *wp);

        void setWaypointInitDefaults(WayPointInitSettings *fdata);

        std::vector<DJI::OSDK::WayPointSettings> createWaypoints(
                int numWaypoints, float64_t distanceIncrement, float32_t start_alt);

        std::vector<DJI::OSDK::WayPointSettings>
        generateWaypointsPolygon(WayPointSettings *start_data, float64_t increment,
                                 int num_wp);

        void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
                             int responseTimeout);
    public:
        explicit WaypointMission(FlightController* flightController);
        bool run(uint8_t numWaypoints);
        bool stop();
        bool pause();
        bool resume();
    };
}


#endif //MATRICE210_WAYPOINTMISSION_H

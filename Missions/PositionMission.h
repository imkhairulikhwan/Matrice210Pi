/*! @file PositionMission.h
 *  @version 1.0
 *  @date Jul 09 2018
 *  @author Jonathan Michel
 *  @brief This class provides position and yaw control fo the aircraft
 *  This control mode is useless because position offsets in x-y plan have
 *  to be relative while altitude and yaw values are absolute
 */

#ifndef MATRICE210_POSITIONMISSION_H
#define MATRICE210_POSITIONMISSION_H

#include <dji_vehicle.hpp>

using namespace DJI;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

namespace M210 {
    class FlightController;

    class PositionMission {
    private:
        FlightController *flightController;
        Vector3f position;
        float yaw;
    public:
        explicit PositionMission(FlightController *flightController);
        /**
        * Allows user to move aircraft in position and yaw mode
        * @param position Position relative values to move in X and Y direction [m]
        * x face to north, y face to east
        * Z value is absolute altitude from take-off point
        * @param yaw Absolute yaw angle to set [deg]
        */
        void move(const Vector3f *position, float yaw);

        /**
         * Has to be called continuously
         */
        void update();
    };
}

#endif //MATRICE210_POSITIONMISSION_H
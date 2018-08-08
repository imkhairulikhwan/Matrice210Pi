/*! @file VelocityMission.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 *  @brief This class implements velocity mission. It allows user
 *  to set a velocity vector to the aircraft
 */

#ifndef MATRICE210_VELOCITYMISSION_H
#define MATRICE210_VELOCITYMISSION_H

#include <dji_vehicle.hpp>

using namespace DJI;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

namespace M210 {
    class FlightController;

    class VelocityMission {
    private:
        FlightController *flightController;
        Vector3f velocity;
        float yaw;
    public:
        explicit VelocityMission(FlightController *flightController);

        /**
         * Velocity Control. Allows user to set a velocity vector.
         * The aircraft will move as described by vector until update()
         * is no more called.
         * @param velocity Absolute velocity vector to set [m/s]
         * Vector is relative to the ground
         * x face to north, y face to east, z face to sky
         * @param yaw Absolute yaw rate to set [deg/s]
         */
        void move(const Vector3f *velocity, float yaw);

        /**
         * Has to be called frequently to send moving order to the
         * aircraft. DJI recommend to send orders at 50Hz
         */
        void update();
    };
}


#endif //MATRICE210_VELOCITYMISSION_H

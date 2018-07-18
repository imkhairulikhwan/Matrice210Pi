/*! @file VelocityMission.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_VELOCITYMISSION_H
#define MATRICE210_VELOCITYMISSION_H


#include <dji_vehicle.hpp>

#include "../util/timer.h"

using namespace DJI;
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class FlightController;

class VelocityMission {
private:
    FlightController* flightController;
    Vector3f velocity;
    float32_t yaw;
public:
    explicit VelocityMission(FlightController *flightController);
    void move(Vector3f *velocity, float yaw);
    void update();
};


#endif //MATRICE210_VELOCITYMISSION_H

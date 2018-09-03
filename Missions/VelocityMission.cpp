/*! @file VelocityMission.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 *  @brief VelocityMission.h implementation
 */
#include "VelocityMission.h"

#include "../Aircraft/FlightController.h"
#include "../util/timer.h"
#include "../util/Log.h"

using namespace M210;

VelocityMission::VelocityMission(FlightController *flightController) {
    this->flightController = flightController;
}

void VelocityMission::move(const Vector3f *velocity, float yaw) {
    LSTATUS("VelocityMission move : x = % .2f m/s, y = % .2f m/s, z = % .2f m/s, yaw = % .2f deg/s",
        velocity->x, velocity->y, velocity->z, yaw);
    this->velocity.x = velocity->x;
    this->velocity.y = velocity->y;
    this->velocity.z = velocity->z;
    this->yaw = yaw;
}

void VelocityMission::update() {
    flightController->velocityAndYawRateCtrl(&velocity, yaw);
}

/*! @file VelocityMission.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */
#include "VelocityMission.h"

#include "../FlightController.h"
#include "../util/timer.h"

VelocityMission::VelocityMission(FlightController *flightController) {
    this->flightController = flightController;
}

void VelocityMission::move(Vector3f *velocity, float yaw) {
    DSTATUS("VelocityMission move : x = % .2f m/s, y = % .2f m/s, z = % .2f m/s, % .2f deg/s",
        velocity->x, velocity->y, velocity->z, yaw);
    this->velocity.x = velocity->x;
    this->velocity.y = velocity->y;
    this->velocity.z = velocity->z;
    this->yaw = yaw;
}

void VelocityMission::update() {
    flightController->velocityAndYawRateCtrl(&velocity, yaw);
}

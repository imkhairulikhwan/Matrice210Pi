/*! @file VelocityMission.cpp
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */
#include "VelocityMission.h"

#include "../FlightController.h"

VelocityMission::VelocityMission(FlightController *flightController) {
    this->flightController = flightController;
}

void VelocityMission::move(Vector3f *velocity, float yaw) {
    this->velocity.x = velocity->x;
    this->velocity.y = velocity->y;
    this->velocity.z = velocity->z;
    this->yaw = yaw;
}

void VelocityMission::update() {
    flightController->velocityAndYawRateCtrl(&velocity, yaw);
    delay_ms(20);
}

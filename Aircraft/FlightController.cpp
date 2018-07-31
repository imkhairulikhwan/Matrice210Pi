/*! @file FlightController.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include "FlightController.h"

#include <cmath>
#include <iostream>

#include <dji_linux_helpers.hpp>

#include "Emergency.h"
#include "Watchdog.h"
#include "../util/Log.h"
#include "../util/timer.h"
#include "../Managers/PackageManager.h"
#include "../Managers/ThreadManager.h"
#include "../Missions/MonitoredMission.h"
#include "../Missions/PositionMission.h"
#include "../Missions/VelocityMission.h"
#include "../Missions/PositionOffsetMission.h"
#include "../Missions/WaypointsMission.h"
#include "../Action/Action.h"

using namespace M210;

pthread_mutex_t FlightController::sendDataToMSDK_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t FlightController::movingMode_mutex = PTHREAD_MUTEX_INITIALIZER;

FlightController::FlightController() {
    linuxEnvironment = nullptr;
    vehicle = nullptr;
    flightControllerThreadRunning = false;
    watchdog = new Watchdog(1000);
    emergency = new Emergency();
    setMovingMode(STOP);
    // Missions
    monitoredMission = new M210::MonitoredMission(this);
    positionMission = new M210::PositionMission(this);
    velocityMission = new M210::VelocityMission(this);
    positionOffsetMission = new M210::PositionOffsetMission(this);
    waypointMission = new M210::WaypointMission(this);
}


FlightController::~FlightController() {
    delete watchdog;
    delete emergency;
    delete positionMission;
    delete velocityMission;
    delete positionOffsetMission;
}


void FlightController::setupVehicle(int argc, char **argv) {
    // Get vehicle
    do {
        // If linuxEnvironment is already set, an error occurred
        // delete linuxEnvironment and retry after 1 second
        if (linuxEnvironment != nullptr) {
            delete linuxEnvironment;
            delay_ms(1000);
        }
        linuxEnvironment = new LinuxSetup(argc, argv);

        vehicle = linuxEnvironment->getVehicle();
        // Retry to get vehicle until it works
        if (vehicle == nullptr) {
            DERROR("Vehicle not initialized, retrying...");
        }
    } while (vehicle == nullptr);

    bool errorMsgDisplayed = false;
    // Obtain control authority
    ACK::ErrorCode ack;
    do {
        ack = vehicle->obtainCtrlAuthority(1);
        if (ACK::getError(ack) != ACK::SUCCESS && !errorMsgDisplayed) {
            DERROR("Obtain control authority failed, retrying...");
            errorMsgDisplayed = true;
        }
    } while (ACK::getError(ack) != ACK::SUCCESS);

    launchFlightControllerThread();
}

void FlightController::launchFlightControllerThread() {
    if (!flightControllerThreadRunning) {
        flightControllerThreadRunning = true;
        ThreadManager::start("flightCtrThread",
                             &flightControllerThreadID, &flightControllerThreadAttr,
                             flightControllerThread, (void *) this);
    }
}

void FlightController::stopFlightControllerThread() {
    flightControllerThreadRunning = false;
    ThreadManager::stop(&flightControllerThreadID);
}

void *FlightController::flightControllerThread(void *param) {
    auto fc = (FlightController *) param;
    while (fc->flightControllerThreadRunning) {
        switch (fc->getMovingMode()) {
            case WAIT:

                break;
            case STOP:
                // TODO Remove if packages need to be keep while aircraft is stopped
                fc->stopAircraft();
                PackageManager::instance().clear();
                fc->setMovingMode(WAIT);
                break;
            case POSITION:
                fc->positionMission->update();
                fc->setMovingMode(WAIT);
                break;
            case VELOCITY:
                fc->velocityMission->update();
                delay_ms(20);
                break;
            case POSITION_OFFSET:
                fc->positionOffsetMission->update();
                delay_ms(fc->positionOffsetMission->getCycleTimeMs());
                break;
        }
    }
    return nullptr;
}

bool FlightController::takeoff() {
    return monitoredMission->takeOff();
}

bool FlightController::landing() {
    return monitoredMission->landing();
}

void FlightController::moveByPosition(const Vector3f *position, float yaw) {
    if(emergency->isEnabled(Emergency::displayError))
        return;
    // Mission parameters
    positionMission->move(position, yaw);
    setMovingMode(POSITION);

}

void FlightController::moveByVelocity(const Vector3f *velocity, float yaw) {
    if(emergency->isEnabled(Emergency::displayError))
        return;
    // Mission parameters
    velocityMission->move(velocity, yaw);
    setMovingMode(VELOCITY);
}


void FlightController::moveByPositionOffset(const Vector3f *offset, float yaw,
                                            float posThreshold, float yawThreshold) {
    if(emergency->isEnabled(Emergency::displayError))
        return;
    positionOffsetMission->move(offset, yaw,
                                posThreshold, yawThreshold);
    setMovingMode(POSITION_OFFSET);
}

void FlightController::stopAircraft() {
    LSTATUS("Aircraft stopped");
    // Stop aircraft
    vehicle->control->emergencyBrake();
    // Stop waypoints mission
    waypointMission->action(Action::MissionAction::STOP);
    // Stop state machine sending moving commands
    setMovingMode(STOP);
}

void FlightController::velocityAndYawRateCtrl(const Vector3f *velocity, float yaw) {
    if (!emergency->isEnabled()) {
        if(!watchdog->isEnabled()) {
            watchdog->increment();
            vehicle->control->velocityAndYawRateCtrl(velocity->x, velocity->y, velocity->z, yaw);
        }
    }
}

void FlightController::positionAndYawCtrl(const Vector3f *position, float yaw) {
    if (!emergency->isEnabled()) {
        if(!watchdog->isEnabled()) {
            watchdog->increment();
            vehicle->control->positionAndYawCtrl(position->x, position->y, position->z, yaw);
        }
    }
}

void FlightController::emergencyStop() {
    // First of all set emergency state (to stop sending moving order)
    emergency->set();
    // Stop aircraft
    vehicle->control->emergencyBrake();
    // Set FlightController thread in stop mode
    setMovingMode(STOP);
    // Stop waypoints mission
    waypointMission->action(Action::MissionAction::STOP);
    LERROR("Emergency break set !");
}

void FlightController::emergencyRelease() {
    emergency->release();
    setMovingMode(STOP);
    LSTATUS("Emergency break released !");
}

void FlightController::sendDataToMSDK(const uint8_t *data, size_t length) const {
    pthread_mutex_lock(&sendDataToMSDK_mutex);
    vehicle->moc->sendDataToMSDK(const_cast<uint8_t*>(data), (uint8_t) length);
    pthread_mutex_unlock(&sendDataToMSDK_mutex);
}

void FlightController::setMovingMode(FlightController::movingMode_ mode) {
    pthread_mutex_lock(&movingMode_mutex);
    movingMode = mode;
    pthread_mutex_unlock(&movingMode_mutex);
}

bool FlightController::startGlobalPositionBroadcast(Vehicle *vehicle) {
    uint8_t freq[16];
    // Channels definition for A3/N3/M600
    freq[0]  = DataBroadcast::FREQ_HOLD; // Timestamp
    freq[1]  = DataBroadcast::FREQ_HOLD; // Attitude Quaternions
    freq[2]  = DataBroadcast::FREQ_HOLD; // Acceleration
    freq[3]  = DataBroadcast::FREQ_HOLD; // Velocity (Ground Frame)
    freq[4]  = DataBroadcast::FREQ_HOLD; // Angular Velocity (Body Frame)
    freq[5]  = DataBroadcast::FREQ_50HZ; // Position - This is the only one we want to change
    freq[6]  = DataBroadcast::FREQ_HOLD; // GPS Detailed Information
    freq[7]  = DataBroadcast::FREQ_HOLD; // RTK Detailed Information
    freq[8]  = DataBroadcast::FREQ_HOLD; // Magnetometer
    freq[9]  = DataBroadcast::FREQ_HOLD; // RC Channels Data
    freq[10] = DataBroadcast::FREQ_HOLD; //  Gimbal Data
    freq[11] = DataBroadcast::FREQ_HOLD; //  Flight Status
    freq[12] = DataBroadcast::FREQ_HOLD; //  Battery Level
    freq[13] = DataBroadcast::FREQ_HOLD; //  Control Information

    ACK::ErrorCode ack =  vehicle->broadcast->setBroadcastFreq(freq, 1);
    if (ACK::getError(ack)) {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    return true;
}

/*! Very simple calculation of local NED offset between two pairs of GPS coordinates.
    Accurate when distances are small.
    Functions given by DJI Programming Guide
!*/
void FlightController::localOffsetFromGpsOffset(Telemetry::Vector3f &deltaNed,
                                                const Telemetry::GPSFused *subscriptionTarget,
                                                const Telemetry::GPSFused *subscriptionOrigin) {
    double deltaLon = subscriptionTarget->longitude - subscriptionOrigin->longitude;
    double deltaLat = subscriptionTarget->latitude - subscriptionOrigin->latitude;
    deltaNed.x = (float32_t) (deltaLat * C_EARTH);
    deltaNed.y = (float32_t) (deltaLon * C_EARTH *
                              cos(subscriptionTarget->latitude / 2.0 + subscriptionOrigin->latitude / 2.0));
    deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
}

Telemetry::Vector3f FlightController::toEulerAngle(const Telemetry::Quaternion *quaternion) {
    Telemetry::Vector3f ans;

    double q2sqr = quaternion->q2 * quaternion->q2;
    double t0 =
            -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
    double t1 =
            +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
    double t2 =
            -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    double t3 =
            +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
    double t4 =
            -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = (float32_t) asin(t2);       // pitch
    ans.y = (float32_t) atan2(t3, t4);  // roll
    ans.z = (float32_t) atan2(t1, t0);  // yaw

    return ans;
}

void FlightController::waypointsMissionAction(unsigned task) {
    waypointMission->action(task);
}

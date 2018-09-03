/*! @file FlightController.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 *  @brief FlightController.h implementation
 */

#include "FlightController.h"

#include <cmath>
#include <iostream>

#include <dji_linux_helpers.hpp>

#include "Emergency.h"
#include "Watchdog.h"
#include "../util/Log.h"
#include "../util/timer.h"
#include "../util/define.h"
#include "../Managers/PackageManager.h"
#include "../Managers/ThreadManager.h"
#include "../Missions/MonitoredMission.h"
#include "../Missions/PositionMission.h"
#include "../Missions/VelocityMission.h"
#include "../Missions/PositionOffsetMission.h"
#include "../Missions/WaypointsMission.h"
#include "../Action/Action.h"
#include "../Gps/GpsAxis.h"

using namespace M210;

pthread_mutex_t FlightController::sendDataToMSDK_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t FlightController::smState_mutex = PTHREAD_MUTEX_INITIALIZER;

FlightController::FlightController() {
    linuxEnvironment = nullptr;
    vehicle = nullptr;
    flightControllerThreadRunning = false;
    watchdog = new Watchdog(50);
    emergency = new Emergency();
    setSMState(STOP);
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
        // Configure linux environment with dji Linux helpers
        linuxEnvironment = new LinuxSetup(argc, argv);

        vehicle = linuxEnvironment->getVehicle();
        // Retry to get vehicle until it works
        if (vehicle == nullptr) {
            DERROR("Vehicle not initialized, retrying...");
        }
    } while (vehicle == nullptr);
}

void FlightController::obtainCtrlAuthority() {
    if(vehicle == nullptr) {
        LERROR("Vehicle not initialized, setup vehicle first");
        return;
    }

    ACK::ErrorCode ack;
    do {
        ack = vehicle->obtainCtrlAuthority(1);
        if (ACK::getError(ack) != ACK::SUCCESS) {
            LERROR("Obtain control authority failed, retrying...");
            LERROR("Be sure the flight controller is in mode P");
            LERROR("Be sure aircraft has multiple flight modes enabled");
            delay_ms(1000);
        }
    } while (ACK::getError(ack) != ACK::SUCCESS);
    LSTATUS("Control authority obtained");
}

void FlightController::launchFlightControllerThread() {
    // Launch flight controller thread if it is not already running
    if (!flightControllerThreadRunning) {
        flightControllerThreadRunning = true;
        ThreadManager::start("flightCtrThread",
                             &flightControllerThreadID, &flightControllerThreadAttr,
                             flightControllerThread, (void *) this);
    }
}

void *FlightController::flightControllerThread(void *param) {
    auto fc = (FlightController *) param;
    while (fc->flightControllerThreadRunning) {
        switch (fc->getSMState()) {
            case WAIT:

                break;
            case STOP:
                // TODO Remove if packages need to be keep while aircraft is stopped
                PackageManager::instance().clear();
                fc->setSMState(WAIT);
                break;
            case POSITION:
                fc->positionMission->update();
                // Orders are send at 50 Hz, as recommended by DJI
                delay_ms(20);
                break;
            case VELOCITY:
                fc->velocityMission->update();
                // Orders are send at 50 Hz, as recommended by DJI
                delay_ms(20);
                break;
            case POSITION_OFFSET:
                fc->positionOffsetMission->update();
                // Orders are send at 50 Hz, as recommended by DJI
                delay_ms(20);
                break;
        }
    }
    return nullptr;
}

bool FlightController::takeOff() {
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
    setSMState(POSITION);

}

void FlightController::moveByVelocity(const Vector3f *velocity, float yaw) {
    if(emergency->isEnabled(Emergency::displayError))
        return;
    // Mission parameters
    velocityMission->move(velocity, yaw);
    setSMState(VELOCITY);
}


void FlightController::moveByPositionOffset(const Vector3f *offset, float yaw,
                                            float posThreshold, float yawThreshold) {
    setSMState(STOP);
    if(emergency->isEnabled(Emergency::displayError))
        return;
    positionOffsetMission->move(offset, yaw,
                                posThreshold, yawThreshold);
    setSMState(POSITION_OFFSET);
}

void FlightController::stopAircraft() {
    // Stop aircraft
    vehicle->control->emergencyBrake();
    // Stop state machine sending moving commands
    setSMState(STOP);
    // Stop waypoints mission
    waypointMission->action(Action::MissionAction::STOP);
    LSTATUS("Aircraft stopped");
}

void FlightController::velocityAndYawRateCtrl(const Vector3f *velocity, float yaw) {
    if (!emergency->isEnabled()) {
        if(!watchdog->isEnabled()) {
            watchdog->increment();
            // Send cardinal orders to the aircraft
            Vector2 v{velocity->x, velocity->y};
            Vector2 projected = GpsAxis::instance().projectVector(v);
            vehicle->control->velocityAndYawRateCtrl((float32_t)projected.x, (float32_t)projected.y, velocity->z, yaw);
        }
    }
}

void FlightController::positionAndYawCtrl(const Vector3f *position, float yaw) {
    if (!emergency->isEnabled()) {
        if(!watchdog->isEnabled()) {
            watchdog->increment();
            // Send cardinal orders to the aircraft
            Vector2 v{position->x, position->y};
            Vector2 projected = GpsAxis::instance().projectVector(v);
            vehicle->control->positionAndYawCtrl((float32_t)projected.x, (float32_t)projected.y, position->z, yaw);
        }
    }
}

void FlightController::emergencyStop() {
    // First of all set emergency state (to stop sending moving order)
    emergency->set();
    // Stop aircraft
    stopAircraft();
    LERROR("Emergency break set !");
}

void FlightController::emergencyRelease() {
    emergency->release();
    setSMState(STOP);
    LSTATUS("Emergency break released !");
}

void FlightController::sendDataToMSDK(const uint8_t *data, size_t length) const {
    pthread_mutex_lock(&sendDataToMSDK_mutex);
    vehicle->moc->sendDataToMSDK(const_cast<uint8_t*>(data), (uint8_t) length);
    pthread_mutex_unlock(&sendDataToMSDK_mutex);
}

void FlightController::setSMState(FlightController::SMState_ mode) {
    pthread_mutex_lock(&smState_mutex);
    SMState = mode;
    pthread_mutex_unlock(&smState_mutex);
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

void FlightController::waypointsMissionAction(unsigned task) {
    waypointMission->action(task);
}

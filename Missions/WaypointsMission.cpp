/*! @file WaypointMission.cpp
 *  @version 1.0
 *  @date Jul 26 2018
 *  @author Jonathan Michel
 */

#include "WaypointsMission.h"

#include <iostream>
#include <cmath>

#include "dji_subscription.hpp"
#include "dji_linux_helpers.hpp"

#include "../Aircraft/FlightController.h"
#include "../Managers/PackageManager.h"
#include "../Action/Action.h"
#include "../util/timer.h"
#include "../util/Log.h"

using namespace M210;

pthread_mutex_t M210::WaypointMission::mutex = PTHREAD_MUTEX_INITIALIZER;

M210::WaypointMission::WaypointMission(FlightController *flightController) {
    this->flightController = flightController;
    setWaypointSettingsDefaults(&waypointsSettings);
    index = 0;
}

bool M210::WaypointMission::start() {
    LSTATUS("Start Waypoints Mission : %u waypoints", index);
    ACK::ErrorCode initAck = flightController->getVehicle()->missionManager->init(
            DJI_MISSION_TYPE::WAYPOINT, 1, &waypointsSettings);
    if (ACK::getError(initAck)) {
        LERROR("Mission initialization failed");
        ACK::getErrorCodeMessage(initAck, __func__);
        return false;
    }

    flightController->getVehicle()->missionManager->printInfo();

    // Upload waypoints
    for (auto &wp : waypointsList) {
        LSTATUS("Upload Waypoint (Lon Lat Att): %f \t%f \t%f ", wp.latitude,
                wp.longitude, wp.altitude);
        ACK::WayPointIndex wpDataACK =
                flightController->getVehicle()->missionManager->wpMission->uploadIndexData(&wp, 1);
        if (ACK::getError(wpDataACK.ack)) {
            LERROR("Waypoint upload failed");
            ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
            return false;
        }
    }

    // Start mission
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->start(1);
    if (ACK::getError(ack)) {
        LERROR("Start waypoints mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    LSTATUS("Start waypoints mission successfully");
    return true;
}


void M210::WaypointMission::reset() {
    stop();
    pthread_mutex_lock(&mutex);
    index = 0;
    waypointsList.clear();
    setWaypointSettingsDefaults(&waypointsSettings);
    pthread_mutex_unlock(&mutex);
    LSTATUS("Waypoints mission reset");
}

void M210::WaypointMission::currentPosition(GlobalPosition &position) {
    // TODO Move, nothing to do here !
    if (!FlightController::startGlobalPositionBroadcast(flightController->getVehicle())) {
        LERROR("getCurrentPosition aborted");
        return;
    }

    // Wait data
    delay_ms(500);

    // Get ultrasonic height in m from broadcast
    GlobalPosition globalPosition = flightController->getVehicle()->broadcast->getGlobalPosition();
    position.longitude = globalPosition.longitude;
    position.latitude = globalPosition.latitude;
    position.altitude = globalPosition.altitude;
    position.height = globalPosition.height;
}

void M210::WaypointMission::add() {
    if(index < MAX_WAYPOINTS) {
        GlobalPosition position;
        currentPosition(position);

        WayPointSettings wp;
        setWaypointDefaults(&wp);

        pthread_mutex_lock(&mutex);
        waypointsSettings.indexNumber = index+(uint8_t)1;

        wp.index = index;
        wp.longitude = position.longitude;
        wp.latitude = position.latitude;
        wp.altitude = position.height;
        waypointsList.push_back(wp);

        LSTATUS("Waypoint %u added (Lon Lat Hei): %f \t%f \t%f", index, wp.longitude,
                wp.latitude, wp.altitude);
        index++;
        pthread_mutex_unlock(&mutex);
    } else {
        LERROR("Max waypoints reached");
    }
}

void M210::WaypointMission::setWaypointDefaults(WayPointSettings* wp)
{
    wp->damping         = 0;
    wp->yaw             = 0;
    wp->gimbalPitch     = 0;
    wp->turnMode        = 0;
    wp->hasAction       = 0;
    wp->actionTimeLimit = 100;
    wp->actionNumber    = 0;
    wp->actionRepeat    = 0;
    for (int i = 0; i < 16; ++i)
    {
        wp->commandList[i]      = 0;
        wp->commandParameter[i] = 0;
    }
}

void M210::WaypointMission::setWaypointSettingsDefaults(WayPointInitSettings *fdata)
{
    fdata->maxVelocity    = 6;
    fdata->idleVelocity   = 4;
    fdata->finishAction   = 0;
    fdata->executiveTimes = 1;
    fdata->yawMode        = 0;
    fdata->traceMode      = 0;
    fdata->RCLostAction   = 1;
    fdata->gimbalPitch    = 0;
    fdata->latitude       = 0;
    fdata->longitude      = 0;
    fdata->altitude       = 0;
}

bool M210::WaypointMission::stop() {
    if(!isMissionInitialized())
        return false;
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->stop(1);
    if (ACK::getError(ack)) {
        LERROR("Stop waypoints mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    LSTATUS("Waypoints mission stopped");
    return true;
}

bool M210::WaypointMission::pause() {
    if(!isMissionInitialized())
        return false;
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->pause(1);
    if (ACK::getError(ack)) {
        LERROR("Pause waypoints mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    LSTATUS("Waypoints mission paused");
    return true;
}

bool M210::WaypointMission::resume() {
    if(!isMissionInitialized())
        return false;
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->resume(1);
    if (ACK::getError(ack)) {
        LERROR("Resume waypoints mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    LSTATUS("Waypoints mission resumed");
    return true;
}

void M210::WaypointMission::action(unsigned int task) {
    switch (task) {
        case Action::MissionAction::ADD:
            add();
            break;
        case Action::MissionAction::RESET:
            reset();
            break;
        case Action::MissionAction::START:
            start();
            break;
        case Action::MissionAction::STOP:
            stop();
            break;
        case Action::MissionAction::PAUSE:
            pause();
            break;
        case Action::MissionAction::RESUME:
            resume();
            break;
        default:
            LERROR("Unknown action");
    }
}

bool M210::WaypointMission::isMissionInitialized() {
    return flightController->getVehicle()->missionManager->wpMission != nullptr;
}

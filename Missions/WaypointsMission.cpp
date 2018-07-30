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
    index = 0;
    waypointsList.clear();
    setWaypointSettingsDefaults(&waypointsSettings);
    LSTATUS("Waypoints mission reset");
}

void M210::WaypointMission::currentPosition(GlobalPosition &position) {
    // TODO Nothing to do here !
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
    // TODO Add mutex
    if(index < MAX_WAYPOINTS) {
        GlobalPosition position;
        currentPosition(position);
        waypointsSettings.indexNumber = index+(uint8_t)1;

        WayPointSettings wp;
        setWaypointDefaults(&wp);
        wp.index = index;
        wp.longitude = position.longitude;
        wp.latitude = position.latitude;
        wp.altitude = position.height;
        waypointsList.push_back(wp);

        LSTATUS("Waypoint %u added (Lon Lat Hei): %f \t%f \t%f \t%f", index, wp.longitude,
                wp.latitude, wp.altitude);
        index++;
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
    fdata->maxVelocity    = 10;
    fdata->idleVelocity   = 5;
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
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->stop(1);
    if (ACK::getError(ack)) {
        LERROR("Stop waypoints mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    LSTATUS("Stopping waypoints mission");
    return true;
}

bool M210::WaypointMission::pause() {
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->pause(1);
    if (ACK::getError(ack)) {
        LERROR("Pause waypoints mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    LSTATUS("Pause Waypoints mission");
    return true;
}

bool M210::WaypointMission::resume() {
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->resume(1);
    if (ACK::getError(ack)) {
        LERROR("Resume waypoints mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    LSTATUS("Resume waypoints Mission");
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
        default:
            LERROR("Unknown action");
    }
}
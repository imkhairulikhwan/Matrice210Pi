/*! @file WaypointMission.cpp
 *  @version 1.0
 *  @date Jul 26 2018
 *  @author Jonathan Michel
 */

#include "WaypointMission.h"

#include <iostream>
#include <cmath>

#include "dji_subscription.hpp"
#include "dji_linux_helpers.hpp"

#include "../Aircraft/FlightController.h"
#include "../Managers/PackageManager.h"
#include "../Action/Action.h"

#include "../util/timer.h"

using namespace M210;

M210::WaypointMission::WaypointMission(FlightController *flightController) {
    this->flightController = flightController;
    setWaypointSettingsDefaults(&waypointsSettings);
    index = 0;
    missionInitialized = false;
}

bool M210::WaypointMission::start() {
    if(isMissionInitialited()) {
        DSTATUS("Initializing Waypoints Mission : %u waypoints", index);
        flightController->getVehicle()->missionManager->printInfo();

        uploadWaypoints();

        ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->start(1);
        if (ACK::getError(ack)) {
            DERROR("Start waypoint mission failed");
            ACK::getErrorCodeMessage(ack, __func__);
            return false;
        }
        DERROR("Start waypoint mission successfully");
        return true;
    }
    return false;
}


void M210::WaypointMission::reset() {
    stop();
    index = 0;
    waypointsList.clear();
    setWaypointSettingsDefaults(&waypointsSettings);
    missionInitialized = false;
}

void M210::WaypointMission::currentPosition(GlobalPosition &position) {
    // TODO Nothing to do here !
    if (!FlightController::startGlobalPositionBroadcast(flightController->getVehicle())) {
        DERROR("getCurrentPosition aborted");
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

    DSTATUS("Current position (Lon Lat Att Hei): %f \t%f \t%f \t%f", globalPosition.longitude,
            globalPosition.latitude, globalPosition.altitude, globalPosition.height);
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

        DSTATUS("Waypoint added : %u", index);
        index++;
    }
}

void M210::WaypointMission::uploadWaypoints()
{
    if(isMissionInitialited()) {
        for (auto &wp : waypointsList) {
            DSTATUS("Waypoint uploaded (Lon Lat Att): %f \t%f \t%f ", wp.latitude,
                    wp.longitude, wp.altitude);
            ACK::WayPointIndex wpDataACK =
                    flightController->getVehicle()->missionManager->wpMission->uploadIndexData(&wp, 1);
            if (ACK::getError(wpDataACK.ack))
                ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
        }
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
        DERROR("Stop waypoint mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    DSTATUS("Stopping waypoint Mission");
    return true;
}

bool M210::WaypointMission::pause() {
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->pause(1);
    if (ACK::getError(ack)) {
        DERROR("Pause waypoint mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    DSTATUS("Pause Waypoint Mission");
    return true;
}

bool M210::WaypointMission::resume() {
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->resume(1);
    if (ACK::getError(ack)) {
        DERROR("Resume waypoint mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    DSTATUS("Resume Waypoint Mission");
    return true;
}

bool M210::WaypointMission::isMissionInitialited() {
    if(!missionInitialized) {
        init();
    }
    return missionInitialized;
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
            DERROR("Unknown action");
    }
}

void M210::WaypointMission::init() {
    ACK::ErrorCode initAck = flightController->getVehicle()->missionManager->init(
            DJI_MISSION_TYPE::WAYPOINT, 1, &waypointsSettings);
    if (ACK::getError(initAck)) {
        DERROR("Mission initialized failed");
        ACK::getErrorCodeMessage(initAck, __func__);
    } else {
        missionInitialized = true;
    }
}

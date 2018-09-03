/*! @file WaypointsMission.cpp
 *  @version 1.0
 *  @date Jul 26 2018
 *  @author Jonathan Michel
 *  @brief WaypointsMission.h implementation
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
    // todo Move, nothing to do here !
    if (!FlightController::startGlobalPositionBroadcast(flightController->getVehicle())) {
        LERROR("getCurrentPosition aborted");
        return;
    }

    // Wait data
    delay_ms(500);

    // Get ultrasonic height in m from broadcast
    // todo get height from subscription package and remove broadcast
    GlobalPosition globalPosition = flightController->getVehicle()->broadcast->getGlobalPosition();
    position.longitude = globalPosition.longitude;
    position.latitude = globalPosition.latitude;
    position.altitude = globalPosition.altitude;
    position.height = globalPosition.height;
}

bool M210::WaypointMission::add() {
    if(index < MAX_WAYPOINTS) {
        GlobalPosition position;
        currentPosition(position);

        WayPointSettings wp;
        setWaypointDefaults(&wp);

        pthread_mutex_lock(&mutex);
        waypointsSettings.indexNumber = index+(uint8_t)1; /*!< Total number of waypoints */

        wp.index = index;                    /*!< Index to be uploaded */
        wp.longitude = position.longitude;  /*!< Latitude (rad) */
        wp.latitude = position.latitude;    /*!< Longitude (rad) */
        wp.altitude = position.height;      /*!< Altitude (relative altitude from takeoff point) */
        waypointsList.push_back(wp);

        LSTATUS("Waypoint %u added (Lon Lat Hei): %f \t%f \t%f", index, wp.longitude,
                wp.latitude, wp.altitude);
        index++;
        pthread_mutex_unlock(&mutex);
        return true;
    } else {
        LERROR("Max waypoints reached");
        return false;
    }
}

void M210::WaypointMission::setWaypointDefaults(WayPointSettings* wp)
{
    // todo use actions
    wp->damping         = 0;    /*!< Bend length */
    wp->yaw             = 0;    /*!< Yaw [deg]) */
    wp->gimbalPitch     = 0;    /*!< Gimbal pitch */
    wp->turnMode        = 0;    /*!< Turn mode */
                                    /*!< 0: clockwise */
                                    /*!< 1: counter-clockwise */
    wp->hasAction       = 0;    /*!< Action flag */
                                    /*!< 0: no action */
                                    /*!< 1: has action */
    wp->actionTimeLimit = 100;  /*!< Action time limit */
    wp->actionNumber    = 0;    /*!< Total number of actions */
    wp->actionRepeat    = 0;    /*!< Total running times */
    for (int i = 0; i < 16; ++i)
    {
        wp->commandList[i]      = 0;    /*!< Command list */
        wp->commandParameter[i] = 0;    /*!< Command parameters */
    }
}

void M210::WaypointMission::setWaypointSettingsDefaults(WayPointInitSettings *wp)
{
    wp->maxVelocity    = 6;  /*!< Maximum speed joystick input(2~15m)*/
    wp->idleVelocity   = 4;  /*!< Cruising Speed */
    wp->finishAction   = 0;  /*!< Action on finish */
                                    /*!< 0: no action */
                                    /*!< 1: return to home */
                                    /*!< 2: auto landing */
                                    /*!< 3: return to point 0 */
                                    /*!< 4: infinite mode， no exit */
    wp->executiveTimes = 1;  /*!< Function execution times */
                                    /*!< 1: once */
                                    /*!< 2: twice */
    wp->yawMode        = 0;  /*!< Yaw mode */
                                    /*!< 0: auto mode(point to next waypoint) */
                                    /*!< 1: lock as an initial value */
                                    /*!< 2: controlled by RC */
                                    /*!< 3: use waypoint's yaw(tgt_yaw) */
    wp->traceMode      = 0;  /*!< Trace mode */
                                    /*!< 0: point to point, after reaching the target waypoint hover, 
                                     * complete waypoints action (if any), 
                                     * then fly to the next waypoint  */
                                    /*!< 1: Coordinated turn mode, smooth transition between waypoints,
                                     * no waypoints task  */
    wp->RCLostAction   = 1;  /*!< Action on rc lost */
                                    /*!< 0: exit waypoint and failsafe */
                                    /*!< 1: continue the waypoint */
    wp->gimbalPitch    = 0;  /*!< Gimbal pitch mode */
                                    /*!< 0: free mode, no control on gimbal */
                                    /*!< 1: auto mode, Smooth transition between waypoints */
    wp->latitude       = 0;  /*!< Focus latitude [rad] */
    wp->longitude      = 0;  /*!< Focus longitude [rad] */
    wp->altitude       = 0;  /*!< Focus altitude (relative takeoff point height) */
}

bool M210::WaypointMission::stop() {
    // Verify is mission is initialized
    if(!isMissionInitialized())
        return false;
    // Send stop order
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->stop(1);
    // If an error occured, display error message
    if (ACK::getError(ack)) {
        LERROR("Stop waypoints mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    LSTATUS("Waypoints mission stopped");
    return true;
}

bool M210::WaypointMission::pause() {
    // Verify is mission is initialized
    if(!isMissionInitialized())
        return false;
    // Send pause order
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->pause(1);
    // If an error occured, display error message
    if (ACK::getError(ack)) {
        LERROR("Pause waypoints mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    LSTATUS("Waypoints mission paused");
    return true;
}

bool M210::WaypointMission::resume() {
    // Verify is mission is initialized
    if(!isMissionInitialized())
        return false;
    // Send resume order
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->resume(1);
    // If an error occured, display error message
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
            LERROR("Waypoints mission unknown action");
    }
}

bool M210::WaypointMission::isMissionInitialized() {
    return flightController->getVehicle()->missionManager->wpMission != nullptr;
}

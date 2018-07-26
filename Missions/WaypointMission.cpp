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

#include "../util/timer.h"

using namespace M210;

M210::WaypointMission::WaypointMission(FlightController *flightController) {
    this->flightController = flightController;
}

bool M210::WaypointMission::polygonExample(uint8_t numWaypoints)
{
    uint16_t frequency = 10;
    TopicName topics[] = { TOPIC_GPS_FUSED };
    int numTopics = sizeof(topics) / sizeof(topics[0]);

    int pkgIndex = PackageManager::instance().subscribe(topics, numTopics, frequency, false);
    if (pkgIndex < 0) {
        DERROR("Waypoint Mission aborted - Failed to start package");
        return false;
    }
    delay_ms(1000);

    // Waypoint Mission : Initialization
    WayPointInitSettings fdata;
    setWaypointInitDefaults(&fdata);

    fdata.indexNumber = (uint8_t)(numWaypoints + 1); // We add 1 to get the aircraft back to the start.

    float64_t increment = 0.000001;
    float start_alt = 5;

    ACK::ErrorCode initAck = flightController->getVehicle()->missionManager->init(
            DJI_MISSION_TYPE::WAYPOINT, 1, &fdata);
    if (ACK::getError(initAck)) {
        ACK::getErrorCodeMessage(initAck, __func__);
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    flightController->getVehicle()->missionManager->printInfo();
    DSTATUS("Initializing Waypoint Mission");

    // Waypoint Mission: Create Waypoints
    vector<WayPointSettings> generatedWaypts =
            createWaypoints(numWaypoints, increment, start_alt);
    DSTATUS("Creating Waypoints");

    // Waypoint Mission: Upload the waypoints
    uploadWaypoints(generatedWaypts);
    DSTATUS("Uploading Waypoints");

    // Waypoint Mission: Start
    if(!start()) {
        // Clean-up
        PackageManager::instance().unsubscribe(pkgIndex);
        return false;
    }

    DSTATUS("Starting Polygon waypoint Mission");

    // Cleanup before return. The mission isn't done yet, but it doesn't need any
    // more input from our side.
    return PackageManager::instance().unsubscribe(pkgIndex) >= 0;
}

vector<WayPointSettings> M210::WaypointMission::createWaypoints(int numWaypoints,
                float64_t distanceIncrement, float start_alt)
{
    // Create Start Waypoint
    WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;

    subscribeGPosition = flightController->getVehicle()->subscribe->getValue<TOPIC_GPS_FUSED>();
    start_wp.latitude  = subscribeGPosition.latitude;
    start_wp.longitude = subscribeGPosition.longitude;
    start_wp.altitude  = start_alt;
    DSTATUS("Waypoint created at (LLA): %f \t%f \t%f",
           subscribeGPosition.latitude, subscribeGPosition.longitude,
           start_alt);

    vector<DJI::OSDK::WayPointSettings> wpVector =
            generateWaypointsPolygon(&start_wp, distanceIncrement, numWaypoints);
    return wpVector;
}

vector<DJI::OSDK::WayPointSettings>
M210::WaypointMission::generateWaypointsPolygon(WayPointSettings* start_data, float64_t increment,
                         int num_wp)
{
    // Let's create a vector to store our waypoints in.
    vector<DJI::OSDK::WayPointSettings> wp_list;

    // Some calculation for the polygon
    float64_t extAngle = 2 * M_PI / num_wp;

    // First waypoint
    start_data->index = 0;
    wp_list.push_back(*start_data);

    // Iterative algorithm
    for (int i = 1; i < num_wp; i++)
    {
        WayPointSettings  wp;
        WayPointSettings* prevWp = &wp_list[i - 1];
        setWaypointDefaults(&wp);
        wp.index     = (uint8_t)i;
        wp.latitude  = (prevWp->latitude + (increment * cos(i * extAngle)));
        wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
        wp.altitude  = prevWp->altitude;
        wp_list.push_back(wp);
    }

    // Come back home
    start_data->index = (uint8_t)num_wp;
    wp_list.push_back(*start_data);

    return wp_list;
}
void M210::WaypointMission::uploadWaypoints(vector<WayPointSettings>& wp_list)
{
    for (auto &wp : wp_list) {
        DSTATUS("Waypoint created at (LLA): %f \t%f \t%f ", wp.latitude,
               wp.longitude, wp.altitude);
        ACK::WayPointIndex wpDataACK =
                flightController->getVehicle()->missionManager->wpMission->uploadIndexData(&wp, 1);
        if(ACK::getError(wpDataACK.ack))
            ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
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

void M210::WaypointMission::setWaypointInitDefaults(WayPointInitSettings* fdata)
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

bool M210::WaypointMission::start() {
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->start(1);
    if (ACK::getError(ack)) {
        DERROR("Start waypoint mission failed");
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    DERROR("Start waypoint mission successfully");
    return true;
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

void M210::WaypointMission::currentPosition(GlobalPosition &position) {
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

void M210::WaypointMission::saveCurrentPosition() {
    currentPosition(savedPosition);
    DSTATUS("Current position saved");
}

void M210::WaypointMission::returnToSavedPosition() {
    // Waypoint Mission : Initialization
    WayPointInitSettings fdata;
    setWaypointInitDefaults(&fdata);

    fdata.indexNumber = 2;

    ACK::ErrorCode initAck = flightController->getVehicle()->missionManager->init(
            DJI_MISSION_TYPE::WAYPOINT, 1, &fdata);
    if (ACK::getError(initAck))
        ACK::getErrorCodeMessage(initAck, __func__);

    flightController->getVehicle()->missionManager->printInfo();
    DSTATUS("Initializing Return to saved position Waypoint Mission");

    GlobalPosition pos;
    currentPosition(pos);
    vector<DJI::OSDK::WayPointSettings> wp_list;

    // Create Waypoint
    WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);
    start_wp.index = 0;
    start_wp.longitude = pos.longitude;
    start_wp.latitude = pos.latitude;
    start_wp.altitude = pos.height;
    wp_list.push_back(start_wp);

    WayPointSettings dest_wp;
    setWaypointDefaults(&dest_wp);
    dest_wp.index = 1;
    dest_wp.longitude = savedPosition.longitude;
    dest_wp.latitude = savedPosition.latitude;
    dest_wp.altitude = savedPosition.height;
    wp_list.push_back(dest_wp);

    // Upload waypoints
    uploadWaypoints(wp_list);

    // Start mission
    start();

    DSTATUS("Return to saved position Waypoint Mission launched");
}

void M210::WaypointMission::action(unsigned id) {
    switch(id) {
        case 1:
            polygonExample(5);
            break;
        case 2:
            resume();
            break;
        case 3:
            pause();
            break;
        case 4:
            stop();
            break;
        case 5:
            saveCurrentPosition();
            break;
        case 6:
            returnToSavedPosition();
            break;
        default:
            DERROR("Unknown action");
    }
}
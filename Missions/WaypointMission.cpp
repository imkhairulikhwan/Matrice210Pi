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


bool M210::WaypointMission::run(uint8_t numWaypoints)
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
    float32_t start_alt = 5;

    ACK::ErrorCode initAck = flightController->getVehicle()->missionManager->init(
            DJI_MISSION_TYPE::WAYPOINT, 1, &fdata);
    if (ACK::getError(initAck))
        ACK::getErrorCodeMessage(initAck, __func__);

    flightController->getVehicle()->missionManager->printInfo();
    DSTATUS("Initializing Waypoint Mission");

    // Waypoint Mission: Create Waypoints
    std::vector<WayPointSettings> generatedWaypts =
            createWaypoints(numWaypoints, increment, start_alt);
    DSTATUS("Creating Waypoints");

    // Waypoint Mission: Upload the waypoints
    uploadWaypoints(generatedWaypts, 1);
    DSTATUS("Uploading Waypoints");

    // Waypoint Mission: Start
    start();

    // Cleanup before return. The mission isn't done yet, but it doesn't need any
    // more input from our side.
    return PackageManager::instance().unsubscribe(pkgIndex) >= 0;
}

std::vector<DJI::OSDK::WayPointSettings>
M210::WaypointMission::createWaypoints(int numWaypoints,
                float64_t distanceIncrement, float32_t start_alt)
{
    // Create Start Waypoint
    WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition broadcastGPosition;

    subscribeGPosition = flightController->getVehicle()->subscribe->getValue<TOPIC_GPS_FUSED>();
    start_wp.latitude  = subscribeGPosition.latitude;
    start_wp.longitude = subscribeGPosition.longitude;
    start_wp.altitude  = start_alt;
    DSTATUS("Waypoint created at (LLA): %f \t%f \t%f",
           subscribeGPosition.latitude, subscribeGPosition.longitude,
           start_alt);

    std::vector<DJI::OSDK::WayPointSettings> wpVector =
            generateWaypointsPolygon(&start_wp, distanceIncrement, numWaypoints);
    return wpVector;
}

std::vector<DJI::OSDK::WayPointSettings>
M210::WaypointMission::generateWaypointsPolygon(WayPointSettings* start_data, float64_t increment,
                         int num_wp)
{

    // Let's create a vector to store our waypoints in.
    std::vector<DJI::OSDK::WayPointSettings> wp_list;

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
void M210::WaypointMission::uploadWaypoints( vector<DJI::OSDK::WayPointSettings>& wp_list,
                int responseTimeout)
{
    for (auto &wp : wp_list) {
        DSTATUS("Waypoint created at (LLA): %f \t%f \t%f ", wp.latitude,
               wp.longitude, wp.altitude);
        ACK::WayPointIndex wpDataACK =
                flightController->getVehicle()->missionManager->wpMission->uploadIndexData(&wp,
                                                                    responseTimeout);

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
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    DSTATUS("Starting Waypoint Mission");
    return true;
}

bool M210::WaypointMission::stop() {
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->stop(1);
    if (ACK::getError(ack)) {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    DSTATUS("Stopping Waypoint Mission");
    return true;
}

bool M210::WaypointMission::pause() {
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->pause(1);
    if (ACK::getError(ack)) {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    DSTATUS("Pausing Waypoint Mission");
    return true;
}

bool M210::WaypointMission::resume() {
    ACK::ErrorCode ack = flightController->getVehicle()->missionManager->wpMission->resume(1);
    if (ACK::getError(ack)) {
        ACK::getErrorCodeMessage(ack, __func__);
        return false;
    }
    DSTATUS("Resuming Waypoint Mission");
    return true;}

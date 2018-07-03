/*! @file flight_control.hpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_FLIGHTCONTROL_HPP
#define MATRICE210_FLIGHTCONTROL_HPP

// System Includes
#include <cmath>

// DJI OSDK includes
#include <dji_status.hpp>
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

bool monitoredTakeoff(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);
bool monitoredLanding(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);

bool moveByPositionOffset(DJI::OSDK::Vehicle *vehicle, float xOffsetDesired,
                          float yOffsetDesired, float zOffsetDesired,
                          float yawDesired, float posThresholdInM = 0.2,
                          float yawThresholdInDeg = 1.0);

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
 * coordinates.
 *
 * Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(DJI::OSDK::Vehicle*             vehicle,
                              DJI::OSDK::Telemetry::Vector3f& deltaNed,
                              void* target, void* origin);

DJI::OSDK::Telemetry::Vector3f toEulerAngle(void* quaternionData);
bool startGlobalPositionBroadcast(DJI::OSDK::Vehicle* vehicle);


#endif // MATRICE210_FLIGHTCONTROL_HPP

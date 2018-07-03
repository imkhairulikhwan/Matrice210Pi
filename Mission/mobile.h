/*! @file mobile.h
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */


#ifndef MATRICE210_MOBILE_H
#define MATRICE210_MOBILE_H

#include <pthread.h>

// DJI OSDK includes
#include <dji_vehicle.hpp>

using namespace DJI;
using namespace DJI::OSDK;

// Helpers
#include <dji_linux_helpers.hpp>


// Register handlers with vehicle
bool setupMocParsing(Vehicle *vehicle,
                     LinuxSetup *linuxEnvironment);
// Main parser for incoming mobile data. This parser will decide what
// vehicle API calls to make.
void parseFromMobileCallback(Vehicle*      vehicle,
                             RecvContainer recvFrame,
                             UserData      userData);
// Mobile poll thread functions
pthread_t setupSamplePollingThread(Vehicle* vehicle);
void* mobileSamplePoll(void* vehiclePtr);

#endif //MATRICE210_MOBILE_H
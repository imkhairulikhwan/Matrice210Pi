//
// Created by jonathan.michel on 27.06.2018.
//

#ifndef MATRICE210_MOBILE_H
#define MATRICE210_MOBILE_H

using namespace DJI;
using namespace DJI::OSDK;

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

// Register handlers with vehicle
bool setupMSDKParsing(Vehicle* vehicle,
                      LinuxSetup*         linuxEnvironment);
// Main parser for incoming mobile data. This parser will decide what
// vehicle API calls to make.
void parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                             DJI::OSDK::RecvContainer recvFrame,
                             DJI::OSDK::UserData      userData);
// Mobile poll thread functions
pthread_t setupSamplePollingThread(Vehicle* vehicle);
void* mobileSamplePoll(void* vehiclePtr);

#endif //MATRICE210_MOBILE_H
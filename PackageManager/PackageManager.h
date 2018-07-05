/*! @file PackageManager.h
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_PACKAGEMANAGER_H
#define MATRICE210_PACKAGEMANAGER_H

#include <dji_vehicle.hpp>

#define MAX_NUMBER_OF_PACKAGE 5

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class PackageManager {
private:
    PackageManager();
    static PackageManager* instance;
    bool packagesStatus[MAX_NUMBER_OF_PACKAGE];
    Vehicle* vehicle = nullptr;
    int timeout{1};
    bool vehicleInstanced();
    bool validIndex(int index);
public:
    static PackageManager* getInstance();
    void setVehicle(Vehicle* vehicle);
    bool verify();
    // TODO Remove index !
    bool subscribe(int index, TopicName *topics, int numTopic, uint16_t frequency, bool enableTimestamp);
    bool unsubscribe(int index);

};

#endif //MATRICE210_PACKAGEMANAGER_H

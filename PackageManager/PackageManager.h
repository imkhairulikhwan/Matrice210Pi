/*! @file PackageManager.h
 *  @version 1.0
 *  @date Jul 05 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_PACKAGEMANAGER_H
#define MATRICE210_PACKAGEMANAGER_H

#include "pthread.h"

#include <dji_vehicle.hpp>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

class PackageManager {
public:
    const static int PACKAGE_UNAVAILABLE{-1};
private:
    PackageManager();
    static PackageManager* instance;
    Vehicle* vehicle = nullptr;
    int timeout{1};
    int packageCnt{0};          // next available package
    bool vehicleInstanced();
    bool validIndex(int index);
    // Mutex
    static pthread_mutex_t packageManager_mutex;
public:
    static PackageManager* getInstance();
    void setVehicle(Vehicle* vehicle);
    bool verify();
    int subscribe(TopicName *topics, int numTopic, uint16_t frequency, bool enableTimestamp);
    bool unsubscribe(int index);
    void clear();

    int allocatePackage();
};

#endif //MATRICE210_PACKAGEMANAGER_H

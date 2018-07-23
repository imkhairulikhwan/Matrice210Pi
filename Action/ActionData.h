/*! @file ActionData.h
 *  @version 1.0
 *  @date Jul 20 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_ACTIONDATA_H
#define MATRICE210_ACTIONDATA_H

#include <pthread.h>

#include <dji_vehicle.hpp>

using namespace DJI::OSDK;

class ActionData {
public:
    enum ActionId {
        monitoredTakeoff,
        monitoredLanding,
        moveByPosition,
        moveByPositionOffset,
        moveByVelocity,
        sendDataToMSDK,
        stopAircraft,
        emergencyStop,
        emergencyRelease
    };
private:
    char* data;
    size_t dataPosCnt;
    size_t dataSize;
    ActionId actionId;
    bool _push(char *c, size_t length);
    bool checkSize(size_t size);
    // Mutex
    static pthread_mutex_t mutex;
public:
    explicit ActionData(ActionId actionId, size_t size = 0);
    ~ActionData();
    bool push(char c);
    bool push(unsigned u);
    bool push(float f);
    bool push(Telemetry::Vector3f &v);
    bool popChar(char &c);
    bool popUnsigned(unsigned &u);
    bool popFloat(float &f);
    bool popVector3f(Telemetry::Vector3f &v);
    void setAction(ActionId actionId1) { this->actionId = actionId1; }
    ActionId getActionId() {return actionId; }
};

#endif //MATRICE210_ACTIONDATA_H
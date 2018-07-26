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

namespace M210 {
    class ActionData {
    public:
        enum ActionId {
            takeOff,
            landing,
            mission,
            sendDataToMSDK,
            stopAircraft,
            emergencyStop,
            emergencyRelease,
            watchdog
        };
    private:
        char *data;
        size_t dataPosCnt;
        size_t dataSize;
        ActionId actionId;

        bool _push(const char *c, size_t length);

        bool checkSize(size_t size) const;

        // Mutex
        static pthread_mutex_t mutex;
    public:
        explicit ActionData(ActionId actionId, size_t size = 0);

        ~ActionData();

        static void unitTest();

        bool push(char c);

        bool push(int i);

        bool push(unsigned u);

        bool push(float f);

        bool push(const Telemetry::Vector3f &v);

        bool push(const uint8_t *data, size_t length);

        bool popChar(char &c);

        bool popInt(int &i);

        bool popUnsigned(unsigned &u);

        bool popFloat(float &f);

        bool popVector3f(Telemetry::Vector3f &v);

        void setAction(ActionId actionId1) { this->actionId = actionId1; }

        ActionId getActionId() const { return actionId; }
    };
}

#endif //MATRICE210_ACTIONDATA_H
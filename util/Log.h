/*! @file Log.h
 *  @version 1.0
 *  @date Jul 27 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_LOG_H
#define MATRICE210_LOG_H

#include "dji_vehicle.hpp"

// _type_ is DJI::LOG definition (DSTATUS, DERROR or DDEBUG)
// ## before __VA_ARGS__ delete previous comma if user call macro
// without variadic elements
#define LLOG(_type_, _param_, ...) \
{ \
    _type_(_param_, ##__VA_ARGS__);   \
    M210::Log::instance().send(#_type_, _param_, ##__VA_ARGS__); \
}

#define LSTATUS(_param_, ...) LLOG(DSTATUS, _param_, ##__VA_ARGS__)
#define LERROR(_param_, ...) LLOG(DERROR, _param_, ##__VA_ARGS__)
#define LDEBUG(_param_, ...) LLOG(DDEBUG, _param_, ##__VA_ARGS__)

using namespace DJI;
using namespace DJI::OSDK;

namespace M210 {
    class FlightController;

    class Log : public Singleton<Log> {
    private:
        FlightController* flightController;
        bool isInitialized() const;
    public:
        Log();
        void setFlightController(FlightController* flightController);
        void send(const char* type, const char* format, ...);
    };
}


#endif //MATRICE210_LOG_H

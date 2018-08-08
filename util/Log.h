/*! @file Log.h
 *  @version 1.0
 *  @date Jul 27 2018
 *  @author Jonathan Michel
 *  @brief This class provides log functions. The goal is to
 *  use DJI::LOG methods to print log on console and send log
 *  to mobile application.
 *  Three types are available : LSTATUS, LERROR, LDEBUG
 *  User can use DJI::OSDK::Log class to enable/disable
 *  specified log type
 *  DSTATUS, DERROR, DDEBUG methods can always be used to
 *  log local messages on the console and not send them
 *  to mobile.
 */

#ifndef MATRICE210_LOG_H
#define MATRICE210_LOG_H

#include "dji_vehicle.hpp"

#define LOG_CHAR '%'

// _state_ is DJI::OSDK::Log::instance().getStatusLogState() or .getDebugLogState()  or .getErrorLogState()
// _type_ is DJI::LOG definition (DSTATUS, DERROR or DDEBUG)
// ## before __VA_ARGS__ delete previous comma if user call macro
// without variadic elements
#define LLOG(_state_, _type_, _param_, ...) \
{ \
    _type_(_param_, ##__VA_ARGS__);   \
    if(_state_) { \
        M210::Log::instance().send(#_type_, _param_, ##__VA_ARGS__); \
    } \
}

#define LSTATUS(_param_, ...) LLOG(STATUS, DSTATUS, _param_, ##__VA_ARGS__)
#define LERROR(_param_, ...) LLOG(ERRORLOG, DERROR, _param_, ##__VA_ARGS__)
#define LDEBUG(_param_, ...) LLOG(DEBUG, DDEBUG, _param_, ##__VA_ARGS__)

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
        /**
         * Send log message to mobile.
         * @param type Log type (DSTATUS, DERROR or DDEBUG)
         * @param format Text to be sent to mobile. Can optionally contain embedded format specifiers
         * that are replaced by the values specified in subsequent additional arguments and formatted
         * as requested.
         * @param ... Variadic additional arguments
         */
        void send(const char* type, const char* format, ...);
    };
}

#endif //MATRICE210_LOG_H
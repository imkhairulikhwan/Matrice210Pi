/*! @file Log.cpp
 *  @version 1.0
 *  @date Jul 27 2018
 *  @author Jonathan Michel
 */

#include "Log.h"

#include <cstdarg>

#include "../Aircraft/FlightController.h"

using namespace M210;

M210::Log::Log() {
    flightController = nullptr;
}

void M210::Log::setFlightController(FlightController *flightController) {
    this->flightController = flightController;
}

void M210::Log::send(const char* type, const char *format, ...) {
    if(!isInitialized())
        return;
    char buffer[100];
    buffer[0] = LOG_CHAR;
    // type = DSTATUS or DERROR or DDEBUG
    // type[1] is respectively S or E or D
    buffer[1] = type[1];
    va_list args;
    va_start(args, format);
    vsprintf(buffer+2, format, args);
    fflush(stdout);
    va_end(args);
    flightController->sendDataToMSDK(reinterpret_cast<uint8_t *>(buffer), strlen(buffer));
}

bool M210::Log::isInitialized() const {
    bool initialized = (flightController != nullptr);
    if(!initialized)
        DERROR("Please configure Log FlightController");
    return initialized;
}



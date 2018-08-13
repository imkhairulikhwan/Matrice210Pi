/*! @file main.cpp
 *  @version 2.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include <iostream>
#include <string>
#include <sstream>

#include <dji_vehicle.hpp>

#include "Aircraft/FlightController.h"
#include "Action/Action.h"
#include "Action/ActionData.h"
#include "Managers/PackageManager.h"
#include "Communication/Console.h"
#include "Communication/Mobile.h"
#include "Communication/Uart.h"
#include "Gps/GeodeticCoord.h"
#include "util/Log.h"

bool running = true;

using namespace M210;

FlightController *flightController;
Console* console;
Mobile *mobileCommunication;

/*!
 *  main
 */
int main(int argc, char** argv)
{
    // Unit test for action data class
    ActionData::unitTest();
    Action::unitTest();
    GeodeticCoord::unitTest();
    /* Todo add unit tests
     *      - Subscription
     *      - MOC
     */

    // Initialize flight controller
    flightController = new FlightController();
    flightController->setupVehicle(argc, argv);
    
    // Singleton configuration
    M210::Log::instance().setFlightController(flightController);
    M210::PackageManager::instance().setVehicle(flightController->getVehicle());
    M210::Action::instance().setFlightController(flightController);

    // Console thread
    // If program was called with 1 as argument
    if(argc == 2 && strtol(argv[1], nullptr, 10)) {
        console =  new Console(flightController);
        console->launchThread();
    }

    // Mobile-Onboard Communication
    mobileCommunication = new Mobile(flightController);
    mobileCommunication->setup();
    //*/

    // STM32 Communication thread
    Uart uart("/dev/ttyUSB0", 115200);
    uart.setFlightController(flightController);
    uart.launchRxThread();
    //*/

    while(running) {
        // Process action queue
        Action::instance().process();
        //*/
    }

    return 0;
}
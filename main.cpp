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

bool running = true;

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

    // Initialize flight controller
    flightController = new FlightController();
    flightController->setupVehicle(argc, argv);

    // Package manager
    PackageManager::instance().setVehicle(flightController->getVehicle());
    Action::instance().setFlightController(flightController);

    // Console thread
    console =  new Console(flightController);
    console->launchThread();

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
        //
        Action::instance().process();
        //*/
    }

    return 0;
}
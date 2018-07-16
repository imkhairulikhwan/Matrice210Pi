/*! @file main.cpp
 *  @version 2.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include <iostream>
#include <string>
#include <sstream>

#include <dji_vehicle.hpp>

#include "FlightController.h"
#include "PackageManager/PackageManager.h"
#include "Console/Console.h"
#include "Mobile/Mobile.h"
#include "Uart/Uart.h"

bool running = true;

FlightController *flightController;
Console* console;
Mobile *mobileCommunication;

/*!
 *  main
 */
int main(int argc, char** argv)
{
    // Initialize flight controller
    flightController = new FlightController();
    flightController->setupVehicle(argc, argv);

    // Package manager
    PackageManager::getInstance()->setVehicle(flightController->getVehicle());

    // Console thread
    console =  new Console(flightController);
    console->launchThread();

    // Mobile-Onboard Communication
    mobileCommunication = new Mobile(flightController);
    mobileCommunication->setup();

    // STM32 Communication thread
    Uart uart("/dev/ttyUSB0", 115200);
    uart.setFlightController(flightController);
    uart.launchRxThread();
    //*/

    //
    while(running) {

    }
    //*/

    return 0;
}
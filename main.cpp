/*! @file main.cpp
 *  @version 2.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include <iostream>
#include <string>
#include <sstream>

#include <dji_status.hpp>
#include <dji_vehicle.hpp>
#include <dji_control.hpp>

#include "timer.h"
#include "Console.h"
#include "FlightController.hpp"
#include "Mobile.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool running = true;

FlightController *flightController;
Console* console;
Mobile *mobileCommunication;


/*! main
 *
 */
int main(int argc, char** argv)
{
    // Initialize flight controller
    flightController = new FlightController();
    flightController->setupVehicle(argc, argv);

    // Console thread
    console =  new Console(flightController);
    console->launchThread();

    // Mobile-Onboard Communication
    mobileCommunication = new Mobile(flightController);
    mobileCommunication->setup();
    //*/

    //
    while(running) {

    }
     //*/

    return 0;
}
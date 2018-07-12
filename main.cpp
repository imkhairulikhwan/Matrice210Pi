/*! @file main.cpp
 *  @version 2.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include <iostream>
#include <string>
#include <sstream>

// TODO Remove : Used to auto-completion
#ifdef __linux__
#include <pigpio.h>
#elif _WIN32
#include <PIGPIO/pigpio.h>
#endif

#include <dji_vehicle.hpp>
#include <pigpio.h>

#include "FlightController.h"
#include "PackageManager/PackageManager.h"
#include "Console/Console.h"
#include "Mobile/Mobile.h"

bool running = true;

FlightController *flightController;
Console* console;
Mobile *mobileCommunication;

#define SLAVE_I2C_ADDR 0b1000000

/*!
 *  main
 */
int main(int argc, char** argv)
{
    int res = gpioInitialise();

    if(res == PI_INIT_FAILED) {
        DERROR("Failed to initialize PIGPIO");
        exit(1);
    } else {
        DSTATUS("PIGPIO initialization success");
    }

    auto handle = (unsigned)i2cOpen(1, SLAVE_I2C_ADDR, 0);

    if(handle < 0) {
        DERROR("Failed to open I2C : Error code = %u", handle);
        exit(1);
    } else {
        DSTATUS("I2C correctly open : handle = %u", handle);
    }

    cout << endl;

    char command = 0b00110101;
    while (running) {
        i2cWriteDevice(handle, &command, 1);
        //delay_ms(20);
        //int bf  = i2cReadByte(handle);
        cout << "Send" << endl;
        usleep(500000);
    }

    i2cClose(handle);
    gpioTerminate();


    /*/ Initialize flight controller
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
    //*/

    //
    while(running) {

    }
    //*/

    return 0;
}
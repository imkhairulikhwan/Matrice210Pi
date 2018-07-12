/*! @file Console.h
 *  @version 1.0
 *  @date Jul 04 2018
 *  @author Jonathan Michel
 */


#ifndef MATRICE210_CONSOLE_H
#define MATRICE210_CONSOLE_H

#include <pthread.h>
#include <string>
#include <sstream>

#include <dji_vehicle.hpp>

using namespace std;
using namespace DJI::OSDK;

class FlightController;

/**
 * Launch a thread to get char on console and control FlightController
 * TODO replace switch-case with queue to assure priority to emergency stop
 */
class Console {
private:
    // Thread attributes
    pthread_t consoleThreadID;
    pthread_attr_t consoleThreadAttr;
    // FlightController
    FlightController *flightController;
public:
    /**
     * Create console object
     * @param flightController FlightController to use
     */
    explicit Console(FlightController* flightController);
    /**
     * Create, set name and launch console thread
     */
    void launchThread();
private:
    /**
     * Dedicated console thread.
     * Get command typed by user and process them
     * @param param Console object is passed as parameter to
     * grant access to FlightController
     * @return -
     */
    static void* consoleThread(void* param);
    /**
     * Display available commands list
     */
    void displayMenu();
    /**
     * Display available command line
     * @param command Command char
     * @param hint Command description, max hint length = 55 to assure
     * alignment
     */
    void displayMenuLine(char command, const std::string &hint);
    /**
     * Read float number from console
     * @param hint Description displayed to user
     * @return Read number
     */
    float32_t getNumber(const std::string &hint);

};


#endif //MATRICE210_CONSOLE_H

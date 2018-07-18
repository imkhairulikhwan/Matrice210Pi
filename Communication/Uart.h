/*! @file Uart.h
 *  @version 1.0
 *  @date Jul 12 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_UART_H
#define MATRICE210_UART_H

#include <pthread.h>
#include <string>
#include <sstream>

#include <dji_vehicle.hpp>
#include <linux_serial_device.hpp>

#define DEBUG_UART_FRAME

using namespace std;
using namespace DJI::OSDK;

class FlightController;

class Uart {
private:
    LinuxSerialDevice* serialDevice;
    FlightController* flightController;
    // Thread attributes
    pthread_t uartRxThreadID;
    pthread_attr_t uartRxThreadAttr;
public:
    Uart(const char* device, uint32_t baudrate);
    ~Uart();
    void send(const uint8_t* buf, size_t len);
    void launchRxThread();
    void setFlightController(FlightController* flightController);

private:
    static void* uartRxThread(void* param);
    size_t read(uint8_t* buf, size_t len);
};


#endif //MATRICE210_UART_H

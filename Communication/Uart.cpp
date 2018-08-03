/*! @file Uart.cpp
 *  @version 1.0
 *  @date Jul 12 2018
 *  @author Jonathan Michel
 */


#include "Uart.h"

#include <iostream>
#include <string>
#include <sstream>

#include "../util/Log.h"
#include "../Aircraft/FlightController.h"
#include "../Managers/ThreadManager.h"

using namespace M210;

Uart::Uart(const char *device, uint32_t baudRate) {
    serialDevice = new LinuxSerialDevice(device, baudRate);
    serialDevice->init();
}

Uart::~Uart() {
    delete serialDevice;
}

void Uart::launchRxThread() {
    ThreadManager::start("uartRxThread",
                         &uartRxThreadID, &uartRxThreadAttr,
                         uartRxThread, (void*)this);
}

void Uart::send(const uint8_t *buf, size_t len) const {
    serialDevice->send(buf, len);
}

size_t Uart::read(uint8_t *buf, size_t len) const{
    return serialDevice->readall(buf, len);
}

void *Uart::uartRxThread(void *param) {
    LSTATUS("uartRxThread running...");
    auto uart = static_cast<Uart*>(param);
    char rxBuffer[256];
    uint8_t rxChar;
    uint8_t rxIndex = 0;

    // Current protocol implementation transmit ascii value of numbers
    // | char indicates new value
    // @ char indicates end of frame
    // frame example : 1|4|2.4513|123.4@
    // TODO Improve protocol
    // It would be better to transmit 32 bits value
    // Needs more frame formatting and CRC
    bool running = true;
    while(running) {
        // New char received
        if(uart->read(&rxChar, 1) != 0) {
            // Add to rx buffer
            rxBuffer[rxIndex] = rxChar;
            rxIndex++;
            // '@' indicate end of transmission
            if(rxChar == '@') {
                //DSTATUS("Frame received : %u", rxIndex);
                // TODO Launch in new thread ?!
                // Process data
                long data_i[20];
                float data_f[20];
                int countValue = 0;
                char tabValues[20];
                int valuesInc = 0;
                int length = rxIndex;

#ifdef DEBUG_UART_FRAME
                std::cout << std::endl;
#endif
                for(uint16_t i = 0; i < length; i++)
                {
#ifdef DEBUG_UART_FRAME
                    std::cout << rxBuffer[i];
#endif
                    // '|' char indicate end of value
                    if(rxBuffer[i] == '|')
                    {
                        tabValues[valuesInc] = ' ';
                        data_f[countValue] = strtof(tabValues, nullptr);
                        data_i[countValue] = strtol(tabValues, nullptr, 10);
                        countValue++;
                        valuesInc = 0;
                    }
                    else
                    {
                        tabValues[valuesInc] = rxBuffer[i];
                        valuesInc++;
                    }
                }
#ifdef DEBUG_UART_FRAME
                std::cout << std::endl;
#endif
                // Last value
                tabValues[valuesInc] = ' ';
                data_f[countValue] = strtof(tabValues, nullptr);
                data_i[countValue] = strtol(tabValues, nullptr, 10);

                // Process data
                switch(data_i[0]) {
                    case 0: {           // antenna value received
                        char buffer[6];
                        buffer[0] = '#';
                        buffer[1] = 'a';
                        memcpy(&buffer[2], &data_i[1], 4);
                        // Send data to MSDK : #a[4 bytes of float value]
                        uart->flightController->sendDataToMSDK(reinterpret_cast<uint8_t *>(buffer), 6);
                        //DSTATUS("Antenna value : %lu", data_i[1]);
                    }
                        break;
                    default:
                        LERROR("Unknown data received from antenna : %u", data_i[0]);
                        break;
                }

                // Reset rx buffer
                rxIndex = 0;
            }
        }
    }
    return nullptr;
}
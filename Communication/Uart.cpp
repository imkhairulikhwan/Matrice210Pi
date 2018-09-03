/*! @file Uart.cpp
 *  @version 1.0
 *  @date Jul 12 2018
 *  @author Jonathan Michel
 *  @brief Uart.h implementation
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

bool Uart::launchRxThread() {
    return ThreadManager::start("uartRxThread",
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

    // todo Improve protocol
    // It would be better to transmit 32 bits value
    // Needs more frame formatting and CRC
    // todo Fill buffer continuously and process data with a dedicated method
    // Need circular buffer implementation
    bool running = true;
    while(running) {
        // New char received
        if(uart->read(&rxChar, 1) != 0) {
            // Add data to rx buffer
            rxBuffer[rxIndex] = rxChar;
            rxIndex++;
            // '@' indicate end of transmission
            if(rxChar == END_OF_FRAME_CHAR) {
                // Process data
                long data_i[20];
                float data_f[20];
                int countValue = 0;
                char tabValues[20];
                int valuesInc = 0;
                int length = rxIndex;

#ifdef DEBUG_UART_FRAME
                //DSTATUS("Frame received : %u", length);
                std::cout << std::endl;
#endif
                for(uint16_t i = 0; i < length; i++)
                {
#ifdef DEBUG_UART_FRAME
                    std::cout << rxBuffer[i];
#endif
                    // '|' char indicate end of value
                    if(rxBuffer[i] == NEW_VALUE_CHAR)
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
                // First integer indicate which value is transmitted
                switch(data_i[0]) {
                    case 0: {           // antenna value received
                        // Send data to MSDK : #a[4 bytes of float value]
                        char buffer[6];
                        buffer[0] = '#';
                        buffer[1] = 'a';
                        memcpy(&buffer[2], &data_i[1], 4);
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
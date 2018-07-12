/*! @file Uart.cpp
 *  @version 1.0
 *  @date Jul 12 2018
 *  @author Jonathan Michel
 */

#include "Uart.h"

Uart::Uart(const char *device, uint32_t baudrate) {
    serialDevice = new LinuxSerialDevice(device, baudrate);
    serialDevice->init();
}

Uart::~Uart() {
    delete serialDevice;
}

void Uart::launchRxThread() {
    pthread_attr_init(&uartRxThreadAttr);
    pthread_attr_setdetachstate(&uartRxThreadAttr, PTHREAD_CREATE_JOINABLE);
    int ret  = pthread_create(&uartRxThreadID, nullptr, uartRxThread, (void*)this);
    string threadName = "uartRxThread";

    if (ret != 0)
        DERROR("Fail to create thread for %s !", threadName.c_str());

    ret = pthread_setname_np(uartRxThreadID, threadName.c_str());
    if (ret != 0)
        DERROR("Fail to set thread name for %s !", threadName.c_str());
}

void Uart::send(const uint8_t *buf, int len) {
    serialDevice->send(buf, len);
}

size_t Uart::read(uint8_t *buf, int len) {
    return serialDevice->readall(buf, len);
}

void *Uart::uartRxThread(void *param) {
    auto uart = (Uart*)param;
    char rxBuffer[256];
    int rxIndex = 0;

    bool running = true;
    while(running) {
        uint8_t rxChar;
        if(uart->read(&rxChar, 1) != 0) {
            rxBuffer[rxIndex] = rxChar;
            if(rxChar == '@') {
                rxIndex = 0;

            }
        }
    }
    return nullptr;
}

//
// Created by jonathan.michel on 27.06.2018.
//

#include "mobile.h"

using namespace DJI;
using namespace DJI::OSDK;

// GLOBAL: mobileDataID for keeping track of command from mobile
uint16_t mobileDataID_glob = 0;
// GLOBAL: keepLoopRunning to maintain state of when to stop checking mobile command state
bool keepLoopRunning = true;

bool setupMSDKParsing(Vehicle* vehicle, LinuxSetup* linuxEnvironment)
{
    // First, register the callback for parsing mobile data
    vehicle->moc->setFromMSDKCallback(parseFromMobileCallback, linuxEnvironment);

    // Then, setup a thread to poll the incoming data, for large functions
    pthread_t threadID = setupSamplePollingThread(vehicle);

    // User input
    std::cout << "Listening to mobile commands. Press any key to exit.\n";
    char a;
    std::cin >> a;

    // Now that we're exiting, Let's shut off the polling thread
    keepLoopRunning = false;
    void* status;
    pthread_join(threadID, &status);

    return true;
}

pthread_t setupSamplePollingThread(Vehicle* vehicle)
{
    int         ret = -1;
    std::string infoStr;

    pthread_t      threadID;
    pthread_attr_t attr;

    /* Initialize and set thread detached attribute */
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    ret     = pthread_create(&threadID, NULL, mobileSamplePoll, (void*)vehicle);
    infoStr = "mobilePoll";

    if (0 != ret)
    {
        DERROR("fail to create thread for %s!\n", infoStr.c_str());
    }

    ret = pthread_setname_np(threadID, infoStr.c_str());
    if (0 != ret)
    {
        DERROR("fail to set thread name for %s!\n", infoStr.c_str());
    }
    return threadID;
}

void parseFromMobileCallback(Vehicle* vehicle, RecvContainer recvFrame,
                        UserData userData) {
    // First, lets cast the userData to LinuxSetup*
    LinuxSetup *linuxEnvironment = (LinuxSetup *) userData;

    uint16_t formatFrameLength = OpenProtocol::PackageMin + OpenProtocol::CRCHead;
    uint16_t msgLength = recvFrame.recvInfo.len - formatFrameLength;
    uint8_t* data = recvFrame.recvData.raw_ack_array;

    if(data[0] == '#') {
        switch (data[1]) {
            case '1':			// toggle LED
                DSTATUS("MOC - Led toggled");
                break;
            default:
                DERROR("MOC - Unknown command");
                break;
        }
    } else {
        data[msgLength] = '\0';
        DSTATUS("MOC - String received : %s", recvFrame.recvData.raw_ack_array);
    }
    vehicle->moc->sendDataToMSDK(data, msgLength);
}

void *
mobileSamplePoll(void* vehiclePtr)
{
    Vehicle* vehicle = (Vehicle *)vehiclePtr;

    // Initialize variables so as to not cross case statements
    bool coreMissionStatus = false;

    // Run polling loop until we're told to stop
    while (keepLoopRunning)
    {
        /*/ Check global variable to find out if we need to execute an action
        switch (mobileDataID_glob)
        {
            case 0x3E:
                coreMissionStatus = runPositionControlSample(vehicle);
                sendAckToMobile(vehicle, 0x3E, coreMissionStatus);
                mobileDataID_glob = 0;
                break;
            default:
                break;
        }
        //*/
        usleep(500000);
    }

}
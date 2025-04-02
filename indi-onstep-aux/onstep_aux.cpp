/*
    myDCP4ESP32 
    Copyright (C) 2023 Stephen Hillier

    Based on MyFocuserPro2 Focuser
    Copyright (C) 2019 Alan Townshend

    As well as USB_Dewpoint
    Copyright (C) 2017-2023 Jarno Paananen

    And INDI Sky Quality Meter Driver
    Copyright(c) 2016 Jasem Mutlaq. All rights reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "onstep_aux.h"
#include "connectionplugins/connectiontcp.h"
#include "connectionplugins/connectionserial.h"
#include "indicom.h"

#include <cstring>
//#include <string>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <mutex>

#define USBDEWPOINT_TIMEOUT 3

// Define auto pointer to ourselves
std::unique_ptr<onstepAux> onstepaux(new onstepAux());

// Mutex for communications
std::mutex osaCommsLock;

onstepAux::onstepAux()
{
    setVersion(0, 1);
}

bool onstepAux::initProperties()
{
    DefaultDevice::initProperties();
    setDriverInterface(AUX_INTERFACE);
    addDebugControl();
    addConfigurationControl();
    setDefaultPollingPeriod(10000);
    addPollPeriodControl();

    if (osaConnection & CONNECTION_SERIAL)
    {
        serialConnection = new Connection::Serial(this);
        serialConnection->registerHandshake([&]()
        {
            return Handshake();
        });
        serialConnection->setDefaultBaudRate(Connection::Serial::B_57600);
        registerConnection(serialConnection);
    }

    if (osaConnection & CONNECTION_TCP)
    {
        tcpConnection = new Connection::TCP(this);
        tcpConnection->setDefaultHost("192.168.4.1");
        tcpConnection->setDefaultPort(3131);
        tcpConnection->registerHandshake([&]()
        {
            return Handshake();
        });

        registerConnection(tcpConnection);
    }


    return true;
}

bool onstepAux::updateProperties()
{
    DefaultDevice::updateProperties();

    if (isConnected()) {
        loadConfig(true);
        timerIndex = SetTimer(getCurrentPollingPeriod());
    } else {
        return false;
    }

    return true;
}

const char *onstepAux::getDefaultName()
{
    return (const char *)"Onstep Aux";
}

// sleep for a number of milliseconds
int onstepAux::msleep( long duration)
{
    struct timespec ts;
    int res;

    if (duration < 0)
    {
        errno = EINVAL;
        return -1;
    }

    ts.tv_sec = duration / 1000;
    ts.tv_nsec = (duration % 1000) * 1000000;

    do
    {
        res = nanosleep(&ts, &ts);
    }
    while (res && errno == EINTR);

    return res;
}


/*************************************************************
 * Called from initProperties to establish contact with device
 *************************************************************/
bool onstepAux::Handshake()
{
    bool handshake_status = false;

    if (getActiveConnection() == serialConnection) {
        PortFD = serialConnection->getPortFD();
        LOG_INFO("Non-Network based connection, detection timeouts set to 0.1 seconds");
        OsaTimeoutMicroSeconds = 100000;
        OsaTimeoutSeconds = 0;
    } else if (getActiveConnection() == tcpConnection) {
        PortFD = tcpConnection->getPortFD();
        LOG_INFO("Network based connection, detection timeouts set to 1 second");
        OsaTimeoutMicroSeconds = 0;
        OsaTimeoutSeconds = 1;
    }

    if (PortFD > 0) {
        char handshake_response[RB_MAX_LEN] = {0};
        handshake_status = getCommandSingleCharErrorOrLongResponse(PortFD, handshake_response,
                                                                                      Osa_handshake);
        if (strcmp(handshake_response, "On-Step") == 0)
        {
            LOG_DEBUG("OnStep Aux handshake established");
            handshake_status = true;
//            GetCapabilites();
//            SlowTimer.start(60000);
        }
        else {
            LOGF_DEBUG("OnStep Aux handshake error, reponse was: %s", handshake_response);
        }
    }
    else {
        LOG_ERROR("OnStep Aux can't handshake, device not connected");
    }

    return handshake_status;
}



// Reboot the Dew Controller then wait to reconnect
//bool onstepAux::rebootController()
//{
//    LOG_INFO("Rebooting Controller and Disconnecting.");
//    sendCommand(MDCP_REBOOT_CMD, nullptr);
//
//    if (!Disconnect())
//        LOG_INFO("Disconnect failed");
//    setConnected(false, IPS_IDLE);
//    updateProperties();
//    LOG_INFO("Waiting 10 seconds before attempting to reconnect.");
//    RemoveTimer(timerIndex);
//
//    int i = 1;
//    do
//    {
//        sleep(10);
//        if (!Connect())
//        {
//            i++;
//            if (i <= 5)
//                LOGF_INFO("Could not reconnect waiting 10 seconds before attempt %d of 5.", i);
//            else
//            {
//                LOGF_ERROR("Could not reconnect after %d attempts", i-1);
//                setConnected(false, IPS_OK);
//            }
//        }
//        else
//        {
//            i = 0;
//            setConnected(true, IPS_OK);
//        }
//    } while ((i != 0) && (i <= 5));
//
//    return updateProperties();
//}

bool onstepAux::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    
    if (!dev || strcmp(dev, getDeviceName()))
        return false;

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool onstepAux::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    
    if (!dev || strcmp(dev, getDeviceName()))
        return false;

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}


void onstepAux::TimerHit()
{
    if (!isConnected())
    {
        return;
    }

    // Get temperatures etc.
//    readSettings();
    timerIndex = SetTimer(getCurrentPollingPeriod());
}

/*********************************************************************
 * Send command to OCS without checking (intended non-existent) return
 * *******************************************************************/
bool onstepAux::sendOsaCommandBlind(const char *cmd)
{
    // No need to block this command as there is no response

    int error_type;
    int nbytes_write = 0;
    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);
    flushIO(PortFD);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(PortFD, TCIFLUSH);
    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK) {
        LOGF_ERROR("CHECK CONNECTION: Error sending command %s", cmd);
        waitingForResponse = false;
        return 0; //Fail if we can't write
        //return error_type;
    }
    return 1;
}

/*********************************************************************
 * Send command to OCS that expects a 0 (sucess) or 1 (failure) return
 * *******************************************************************/
bool onstepAux::sendOsaCommand(const char *cmd)
{
    blockUntilClear();

    char response[1] = {0};
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(PortFD);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(PortFD, TCIFLUSH);

    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(PortFD, response, 1, OsaTimeoutSeconds, OsaTimeoutMicroSeconds, &nbytes_read);

    tcflush(PortFD, TCIFLUSH);
    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%c>", response[0]);
    //waitingForResponse = false;
    clearBlock();

    if (nbytes_read < 1) {
        LOG_WARN("Timeout/Error on response. Check connection.");
        return false;
    }

    return (response[0] == '0'); //OCS uses 0 for success and non zero for failure, in *most* cases;
}

/************************************************************
 * Send command to OCS that expects a single character return
 * **********************************************************/
int onstepAux::getCommandSingleCharResponse(int fd, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(fd, data, 1, OsaTimeoutSeconds, OsaTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    if (error_type != TTY_OK)
        return error_type;

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) { //given this function that should always be true, as should nbytes_read always be 1
        data[nbytes_read] = '\0';
    } else {
        LOG_DEBUG("got RB_MAX_LEN bytes back (which should never happen), last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);
    //waitingForResponse = false;
    clearBlock();

    return nbytes_read;
}

/**************************************************
 * Send command to OCS that expects a double return
 * ************************************************/
int onstepAux::getCommandDoubleResponse(int fd, double *value, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_section_expanded(fd, data, '#', OsaTimeoutSeconds, OsaTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) { //If within buffer, terminate string with \0 (in case it didn't find the #)
        data[nbytes_read] = '\0'; //Indexed at 0, so this is the byte passed it
    } else {
        LOG_DEBUG("got RB_MAX_LEN bytes back, last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);
    //waitingForResponse = false;
    clearBlock();

    if (error_type != TTY_OK) {
        LOGF_DEBUG("Error %d", error_type);
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return error_type;
    }

    if (sscanf(data, "%lf", value) != 1) {
        LOG_WARN("Invalid response, check connection");
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return RES_ERR_FORMAT; //-1001, so as not to conflict with TTY_RESPONSE;
    }

    return nbytes_read;
}

/************************************************
 * Send command to OCS that expects an int return
 * **********************************************/
int onstepAux::getCommandIntResponse(int fd, int *value, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(fd, data, sizeof(char), OsaTimeoutSeconds, OsaTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) { //If within buffer, terminate string with \0 (in case it didn't find the #)
        data[nbytes_read] = '\0'; //Indexed at 0, so this is the byte passed it
    } else {
        LOG_DEBUG("got RB_MAX_LEN bytes back, last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);
    //waitingForResponse = false;
    clearBlock();

    if (error_type != TTY_OK) {
        LOGF_DEBUG("Error %d", error_type);
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return error_type;
    }
    if (sscanf(data, "%i", value) != 1) {
        LOG_WARN("Invalid response, check connection");
        LOG_DEBUG("Flushing connection");
        tcflush(fd, TCIOFLUSH);
        return RES_ERR_FORMAT; //-1001, so as not to conflict with TTY_RESPONSE;
    }

    return nbytes_read;
}

/***************************************************************************
 * Send command to OCS that expects a char[] return (could be a single char)
 * *************************************************************************/
int onstepAux::getCommandSingleCharErrorOrLongResponse(int fd, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_section_expanded(fd, data, '#', OsaTimeoutSeconds, OsaTimeoutMicroSeconds, &nbytes_read);
    tcflush(fd, TCIFLUSH);

    term = strchr(data, '#');
    if (term)
        *term = '\0';
    if (nbytes_read < RB_MAX_LEN) { //If within buffer, terminate string with \0 (in case it didn't find the #)
        data[nbytes_read] = '\0'; //Indexed at 0, so this is the byte passed it
    } else {
        LOG_DEBUG("got RB_MAX_LEN bytes back, last byte set to null and possible overflow");
        data[RB_MAX_LEN - 1] = '\0';
    }

    DEBUGF(INDI::Logger::DBG_DEBUG, "RES <%s>", data);
    //waitingForResponse = false;
    clearBlock();

    if (error_type != TTY_OK) {
        LOGF_DEBUG("Error %d", error_type);
        return error_type;
    }

    return nbytes_read;
}

/********************************************************
 * Converts an OCS char[] return of a numeric into an int
 * ******************************************************/
int onstepAux::getCommandIntFromCharResponse(int fd, char *data, int *response, const char *cmd)
{
    int errorOrFail = getCommandSingleCharErrorOrLongResponse(fd, data, cmd);
    if (errorOrFail < 1) {
        waitingForResponse = false;
        return errorOrFail;
    } else {
        int value = conversion_error;
        try {
            value = std::stoi(data);
        } catch (const std::invalid_argument&) {
            LOGF_WARN("Invalid response to %s: %s", cmd, data);
        } catch (const std::out_of_range&) {
            LOGF_WARN("Invalid response to %s: %s", cmd, data);
        }
        *response = value;
        return errorOrFail;
    }
}


/**********************
 * Flush the comms port
 * ********************/
int onstepAux::flushIO(int fd)
{
    tcflush(fd, TCIOFLUSH);
    int error_type = 0;
    int nbytes_read;
    std::unique_lock<std::mutex> guard(osaCommsLock);
    tcflush(fd, TCIOFLUSH);
    do {
        char discard_data[RB_MAX_LEN] = {0};
        error_type = tty_read_section_expanded(fd, discard_data, '#', 0, 1000, &nbytes_read);
        if (error_type >= 0) {
            LOGF_DEBUG("flushIO: Information in buffer: Bytes: %u, string: %s", nbytes_read, discard_data);
        }
        //LOGF_DEBUG("flushIO: error_type = %i", error_type);
    }
    while (error_type > 0);

    return 0;
}

int onstepAux::charToInt (char *inString)
{
    int value = conversion_error;
    try {
        value = std::stoi(inString);
    } catch (const std::invalid_argument&) {
    } catch (const std::out_of_range&) {
    }
    return value;
}

/*******************************************************
 * Block outgoing command until previous return is clear
 * *****************************************************/
void onstepAux::blockUntilClear()
{
    // Blocking wait for last command response to clear
    while (waitingForResponse) {
        usleep(((OsaTimeoutSeconds * 1000000) + OsaTimeoutMicroSeconds) / 10);
        //        usleep(OCSTimeoutMicroSeconds / 10);
    }
    // Grab the response waiting command blocker
    waitingForResponse = true;
}

/*********************************************
 * Flush port and clear command sequence block
 * *******************************************/
void onstepAux::clearBlock()
{
    waitingForResponse = false;
}


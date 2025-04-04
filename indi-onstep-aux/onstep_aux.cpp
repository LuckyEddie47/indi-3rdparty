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
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <mutex>

// Debug only - required for attaching debugger
// #include <signal.h>
// #include <unistd.h>
// Debug only end

// Custom tabs
#define MANUAL_TAB "Manual"
#define OUTPUTS_TAB "Outputs"

// Define auto pointer to ourselves
std::unique_ptr<onstepAux> onstepaux(new onstepAux());

// Mutex for communications
std::mutex osaCommsLock;

onstepAux::onstepAux()
{
    // Debug only
    // Halts the process at this point. Allows remote debugger to attach which is required
    // when launching the driver from a client eg. Ekos
    // kill(getpid(), SIGSTOP);
    // Debug only end

    setVersion(0, 1);
}

/**********************************************
 * Called from defaultDevice after construction
 **********************************************/
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

    // Output tab controls
    //--------------------
    IUFillSwitchVector(&Output1SP, Output1S, SWITCH_TOGGLE_COUNT, getDeviceName(), "OUTPUT1", "Device 1",
                       OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Output1S[ON_SWITCH], "OUTPUT1_ON", "ON", ISS_OFF);
    IUFillSwitch(&Output1S[OFF_SWITCH], "OUTPUT1_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Output_Name1TP, Output_Name1T, 1, getDeviceName(), "OUTPUT_1_NAME", "Device 1",
                     OUTPUTS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Output_Name1T[0], "DEVICE_1_NAME", "Name", "");

    IUFillSwitchVector(&Output2SP, Output2S, SWITCH_TOGGLE_COUNT, getDeviceName(), "OUTPUT2", "Device 2",
                       OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Output2S[ON_SWITCH], "OUTPUT2_ON", "ON", ISS_OFF);
    IUFillSwitch(&Output2S[OFF_SWITCH], "OUTPUT2_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Output_Name2TP, Output_Name2T, 1, getDeviceName(), "OUTPUT_2_NAME", "Device 2",
                     OUTPUTS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Output_Name2T[0], "DEVICE_2_NAME", "Name", "");

    IUFillSwitchVector(&Output3SP, Output3S, SWITCH_TOGGLE_COUNT, getDeviceName(), "OUTPUT3", "Device 3",
                       OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Output3S[ON_SWITCH], "OUTPUT3_ON", "ON", ISS_OFF);
    IUFillSwitch(&Output3S[OFF_SWITCH], "OUTPUT3_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Output_Name3TP, Output_Name3T, 1, getDeviceName(), "OUTPUT_3_NAME", "Device 3",
                     OUTPUTS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Output_Name3T[0], "DEVICE_3_NAME", "Name", "");

    IUFillSwitchVector(&Output4SP, Output4S, SWITCH_TOGGLE_COUNT, getDeviceName(), "OUTPUT4", "Device 4",
                       OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Output4S[ON_SWITCH], "OUTPUT4_ON", "ON", ISS_OFF);
    IUFillSwitch(&Output4S[OFF_SWITCH], "OUTPUT4_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Output_Name4TP, Output_Name4T, 1, getDeviceName(), "OUTPUT_4_NAME", "Device 4",
                     OUTPUTS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Output_Name4T[0], "DEVICE_4_NAME", "Name", "");

    IUFillSwitchVector(&Output5SP, Output5S, SWITCH_TOGGLE_COUNT, getDeviceName(), "OUTPUT5", "Device 5",
                       OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Output5S[ON_SWITCH], "OUTPUT5_ON", "ON", ISS_OFF);
    IUFillSwitch(&Output5S[OFF_SWITCH], "OUTPUT5_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Output_Name5TP, Output_Name5T, 1, getDeviceName(), "OUTPUT_5_NAME", "Device 5",
                     OUTPUTS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Output_Name5T[0], "DEVICE_5_NAME", "Name", "");

    IUFillSwitchVector(&Output6SP, Output6S, SWITCH_TOGGLE_COUNT, getDeviceName(), "OUTPUT6", "Device 6",
                       OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Output6S[ON_SWITCH], "OUTPUT6_ON", "ON", ISS_OFF);
    IUFillSwitch(&Output6S[OFF_SWITCH], "OUTPUT6_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Output_Name6TP, Output_Name6T, 1, getDeviceName(), "OUTPUT_6_NAME", "Device 6",
                     OUTPUTS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Output_Name6T[0], "DEVICE_6_NAME", "Name", "");

    IUFillSwitchVector(&Output7SP, Output7S, SWITCH_TOGGLE_COUNT, getDeviceName(), "OUTPUT7", "Device 7",
                       OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Output7S[ON_SWITCH], "OUTPUT7_ON", "ON", ISS_OFF);
    IUFillSwitch(&Output7S[OFF_SWITCH], "OUTPUT7_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Output_Name7TP, Output_Name7T, 1, getDeviceName(), "OUTPUT_7_NAME", "Device 7",
                     OUTPUTS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Output_Name7T[0], "DEVICE_7_NAME", "Name", "");

    IUFillSwitchVector(&Output8SP, Output8S, SWITCH_TOGGLE_COUNT, getDeviceName(), "OUTPUT8", "Device 8",
                       OUTPUTS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Output8S[ON_SWITCH], "OUTPUT8_ON", "ON", ISS_OFF);
    IUFillSwitch(&Output8S[OFF_SWITCH], "OUTPUT8_OFF", "OFF", ISS_ON);
    IUFillTextVector(&Output_Name8TP, Output_Name8T, 1, getDeviceName(), "OUTPUT_8_NAME", "Device 8",
                     OUTPUTS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Output_Name8T[0], "DEVICE_8_NAME", "Name", "");


    // Manual tab controls
    //--------------------

    // Debug only
    IUFillTextVector(&Arbitary_CommandTP, Arbitary_CommandT, 1, getDeviceName(), "ARBITARY_COMMAND", "Command",
                     MANUAL_TAB, IP_RW, 60, IPS_IDLE);
    IUFillText(&Arbitary_CommandT[0], "ARBITARY_COMMANDT", "Response:", ":IP#");
    // Debug only end

    // Standard Indi aux controls
    //---------------------------
    addAuxControls();

    return true;
}

bool onstepAux::updateProperties()
{
    DefaultDevice::updateProperties();

    if (isConnected()) {
        loadConfig(true);
        timerIndex = SetTimer(getCurrentPollingPeriod());

        if (outputs[0] > 0) {
            defineProperty(&Output1SP);
            defineProperty(&Output_Name1TP);
        }
        if (outputs[1] > 0) {
            defineProperty(&Output2SP);
            defineProperty(&Output_Name2TP);
        }
        if (outputs[2] > 0) {
            defineProperty(&Output3SP);
            defineProperty(&Output_Name3TP);
        }
        if (outputs[3] > 0) {
            defineProperty(&Output4SP);
            defineProperty(&Output_Name4TP);
        }
        if (outputs[4] > 0) {
            defineProperty(&Output5SP);
            defineProperty(&Output_Name5TP);
        }
        if (outputs[5] > 0) {
            defineProperty(&Output6SP);
            defineProperty(&Output_Name6TP);
        }
        if (outputs[6] > 0) {
            defineProperty(&Output7SP);
            defineProperty(&Output_Name7TP);
        }
        if (outputs[7] > 0) {
            defineProperty(&Output8SP);
            defineProperty(&Output_Name8TP);
        }

        // Debug only
        defineProperty(&Arbitary_CommandTP);
        // Debug only end
    } else {
        if (outputs[0] > 0) {
            deleteProperty(Output1SP.name);
            deleteProperty(Output_Name1TP.name);
        }
        if (outputs[1] > 0) {
            deleteProperty(Output2SP.name);
            deleteProperty(Output_Name2TP.name);
        }
        if (outputs[2] > 0) {
            deleteProperty(Output3SP.name);
            deleteProperty(Output_Name3TP.name);
        }
        if (outputs[3] > 0) {
            deleteProperty(Output4SP.name);
            deleteProperty(Output_Name4TP.name);
        }
        if (outputs[4] > 0) {
            deleteProperty(Output5SP.name);
            deleteProperty(Output_Name5TP.name);
        }
        if (outputs[5] > 0) {
            deleteProperty(Output6SP.name);
            deleteProperty(Output_Name6TP.name);
        }
        if (outputs[6] > 0) {
            deleteProperty(Output7SP.name);
            deleteProperty(Output_Name7TP.name);
        }
        if (outputs[7] > 0) {
            deleteProperty(Output8SP.name);
            deleteProperty(Output_Name8TP.name);
        }

        // Debug only
        deleteProperty(Arbitary_CommandTP.name);
        // Debug only end

        return false;
    }

    return true;
}

const char *onstepAux::getDefaultName()
{
    return (const char *)"Onstep Aux";
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

        // We send a handshake blind (ignore the return) as OnStepX always seems to return a unterminated 0
        // on the first command after connect, so this gets it out of the way
        sendOsaCommandBlind(Osa_handshake);
        auto response = getCommandSingleCharErrorOrLongResponse(PortFD, handshake_response, Osa_handshake);
        (void)response;
        if (strcmp(handshake_response, "On-Step") == 0)
        {
            // This is checking that the connected OnStep device does not have a mount defined
            auto response = sendOsaCommand(Osa_noMount);
            if (response) {
                LOG_DEBUG("OnStep Aux handshake established");
                handshake_status = true;
                getCapabilities();
            //            SlowTimer.start(60000);
            } else {
                LOG_ERROR("The connected OnStep device has a mount and can not be used with this driver");
            }
        } else {
            LOGF_DEBUG("OnStep Aux handshake error, reponse was: %s", handshake_response);
        }
    } else {
        LOG_ERROR("OnStep Aux can't handshake, device not connected");
    }

    return handshake_status;
}

/**************************************************
 * Discover features of the connected OnStep device
 **************************************************/
void onstepAux::getCapabilities()
{
    // Get feature definitions
    char feature_definitions_response[RB_MAX_LEN] = {0};
    int feature_definitions_error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, feature_definitions_response,
                                                                                                Osa_getFeatureDefinitions);
    // Return should be 8 features + 1 terminator bytes long
    if (feature_definitions_error_or_fail == (OUTPUT_COUNT + 1)) {
        for (int outputNo = 0; outputNo < OUTPUT_COUNT; outputNo ++) {
            char output = feature_definitions_response[outputNo];
            if (charToInt(&output) != conversion_error) {
                outputs[outputNo] = charToInt(&output);
            }
        }
        // Defined devices return a 1, undefined return 0
        // so we can sum these to check if any are defined, if not then keep tab hidden
        int outputsDisabled = 0;
        for (int outputNo = 0; outputNo < OUTPUT_COUNT; outputNo ++) {
            outputsDisabled += outputs[outputNo];
        }
        if (outputsDisabled > 0) {
            outputs_tab_enabled = true;
            LOG_INFO("OnStep Aux has feature device(s), enabling tab");
            for (int deviceNo = 1; deviceNo < OUTPUT_COUNT; deviceNo ++) {
                if (outputs[(deviceNo - 1)] == 1) {
                    char feature_definition_response[RB_MAX_LEN] = {0};
                    char get_feature_definition_command[CMD_MAX_LEN] = {0};
                    sprintf(get_feature_definition_command, "%s%i%s",
                            Osa_getFeatureNameTypePart, deviceNo, Osa_command_terminator);
                    int feature_definition_error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, feature_definition_response,
                                                                                                 get_feature_definition_command);
                    if (feature_definition_error_or_fail > 0) {
                        char *split;
                        split = strtok(feature_definition_response, ",");
                        switch(deviceNo) {
                            case 1:
                                indi_strlcpy(OUTPUT1_NAME, split, sizeof(OUTPUT1_NAME));
                                IUSaveText(&Output_Name1T[0], OUTPUT1_NAME);
                                IDSetText(&Output_Name1TP, nullptr);
                                break;
                            case 2:
                                indi_strlcpy(OUTPUT2_NAME, split, sizeof(OUTPUT2_NAME));
                                IUSaveText(&Output_Name2T[0], OUTPUT2_NAME);
                                IDSetText(&Output_Name2TP, nullptr);
                                break;
                            case 3:
                                indi_strlcpy(OUTPUT3_NAME, split, sizeof(OUTPUT3_NAME));
                                IUSaveText(&Output_Name3T[0], OUTPUT3_NAME);
                                IDSetText(&Output_Name3TP, nullptr);
                                break;
                            case 4:
                                indi_strlcpy(OUTPUT4_NAME, split, sizeof(OUTPUT4_NAME));
                                IUSaveText(&Output_Name4T[0], OUTPUT4_NAME);
                                IDSetText(&Output_Name4TP, nullptr);
                                break;
                            case 5:
                                indi_strlcpy(OUTPUT5_NAME, split, sizeof(OUTPUT5_NAME));
                                IUSaveText(&Output_Name5T[0], OUTPUT5_NAME);
                                IDSetText(&Output_Name5TP, nullptr);
                                break;
                            case 6:
                                indi_strlcpy(OUTPUT6_NAME, split, sizeof(OUTPUT6_NAME));
                                IUSaveText(&Output_Name6T[0], OUTPUT6_NAME);
                                IDSetText(&Output_Name6TP, nullptr);
                                break;
                            case 7:
                                indi_strlcpy(OUTPUT7_NAME, split, sizeof(OUTPUT7_NAME));
                                IUSaveText(&Output_Name7T[0], OUTPUT7_NAME);
                                IDSetText(&Output_Name7TP, nullptr);
                                break;
                            case 8:
                                indi_strlcpy(OUTPUT8_NAME, split, sizeof(OUTPUT8_NAME));
                                IUSaveText(&Output_Name8T[0], OUTPUT8_NAME);
                                IDSetText(&Output_Name8TP, nullptr);
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
        } else {
            LOG_INFO("OnStep Aux device does not have features(s), disabling tab");
        }
    } else if (strcmp(feature_definitions_response, "0") == 0) {
        LOG_INFO("OnStep Aux device does not have features(s), disabling tab");
    }
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


/*****************************************
 * Client has changed a text field, update
 *****************************************/
bool onstepAux::ISNewText(const char *dev,const char *name,char *texts[],char *names[],int n)
{
    // Debug only - Manual tab, Arbitary command
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {
        if (!strcmp(Arbitary_CommandTP.name, name)) {
            if (1 == n) {
                char command_response[RB_MAX_LEN] = {0};
                int command_error_or_fail  = getCommandSingleCharErrorOrLongResponse(PortFD, command_response, texts[0]);
                if (command_error_or_fail > 0) {
                    if (strcmp(command_response, "") == 0) {
                        indi_strlcpy(command_response, "No response", sizeof(command_response));
                    }
                } else {
                    char error_code[RB_MAX_LEN] = {0};
                    if (command_error_or_fail == TTY_TIME_OUT) {
                        indi_strlcpy(command_response, "No response", sizeof(command_response));
                    } else {
                        sprintf(error_code, "Error: %d", command_error_or_fail);
                        indi_strlcpy(command_response, error_code, sizeof(command_response));
                    }
                }
                        // Replace the user entered string with the OCS response
                indi_strlcpy(texts[0], command_response, RB_MAX_LEN);
                IUUpdateText(&Arbitary_CommandTP, texts, names, n);
                IDSetText(&Arbitary_CommandTP, nullptr);
                return true;
            }
        }
        return false;
    // Debug only end

    } else {
        return false;
    }
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
        clearBlock();
        return 0; //Fail if we can't write
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


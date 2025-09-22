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

#pragma once

#include <defaultdevice.h>

#define RB_MAX_LEN 64
#define CMD_MAX_LEN 32
enum ResponseErrors {RES_ERR_FORMAT = -1001};

// OnStep commands
//----------------

// Get Product (compatibility)
#define Osa_handshake ":GVP#"
// Returns: On-Step#

// Get Status
#define Osa_noMount ":GU#"
// Normally for OnStep this returns the mount status
// This driver is only for use with OnStep devices that have no mount
// For our use there is no return (unterminated 0) as the lack of mount
// disables the mount command handler. Any return indicates that the
// OnStep device has a mount and should not be used with this driver

// Get feature(s) definitions
#define Osa_getFeatureDefinitions ":GXY0#"

// Get feature(s) enabled and name
#define Osa_getFeatureNameTypePart ":GXY"

// Get feature type
#define Osa_getFeaturePart ":GXX"

// Set feature
#define Osa_setFeaturePart ":SXX"

// For dynamically assembled commands
//-----------------------------------
#define Osa_command_terminator "#"


class onstepAux : public INDI::DefaultDevice
{
    public:
        onstepAux();
        virtual ~onstepAux() = default;

        virtual const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        virtual void TimerHit() override;

        /**
         * @struct MdcpConnection
         * @brief Holds the connection mode of the device.
         */
        typedef enum
        {
            CONNECTION_NONE   = 1 << 0,
            CONNECTION_SERIAL = 1 << 1,
            CONNECTION_TCP    = 1 << 2
        } OsaConnection;

    private:
        int  timerIndex;
        bool Handshake();
        void getCapabilities();

        int conversion_error = -10000;

        Connection::Serial *serialConnection { nullptr };
        Connection::TCP *tcpConnection { nullptr };
        int PortFD { -1 };
        uint8_t osaConnection { CONNECTION_SERIAL | CONNECTION_TCP };

        // Command sequence enforcement
        bool waitingForResponse = false;

        bool sendOsaCommand(const char *cmd);
        bool sendOsaCommandBlind(const char *cmd);
        int flushIO(int fd);
        int getCommandSingleCharResponse(int fd, char *data, const char *cmd); //Reimplemented from getCommandString
        int getCommandSingleCharErrorOrLongResponse(int fd, char *data, const char *cmd); //Reimplemented from getCommandString
        int getCommandDoubleResponse(int fd, double *value, char *data,
                                     const char *cmd); //Reimplemented from getCommandString Will return a double, and raw value.
        int getCommandIntResponse(int fd, int *value, char *data, const char *cmd);
        int getCommandIntFromCharResponse(int fd, char *data, int *response, const char *cmd); //Calls getCommandSingleCharErrorOrLongResponse with conversion of return
        int charToInt(char *inString);
        void blockUntilClear();
        void clearBlock();

        long int OsaTimeoutSeconds = 0;
        long int OsaTimeoutMicroSeconds = 100000;

        enum {
            ON_SWITCH,
            OFF_SWITCH,
            SWITCH_TOGGLE_COUNT
        };

        // Outputs tab controls
        //-------------------
        bool outputs_tab_enabled = false;

        enum {
            OUTPUT1,
            OUTPUT2,
            OUTPUT3,
            OUTPUT4,
            OUTPUT5,
            OUTPUT6,
            OUTPUT7,
            OUTPUT8,
            OUTPUT_COUNT
        };

        int outputs[OUTPUT_COUNT] = {0};

        char OUTPUT1_NAME[RB_MAX_LEN];
        char OUTPUT2_NAME[RB_MAX_LEN];
        char OUTPUT3_NAME[RB_MAX_LEN];
        char OUTPUT4_NAME[RB_MAX_LEN];
        char OUTPUT5_NAME[RB_MAX_LEN];
        char OUTPUT6_NAME[RB_MAX_LEN];
        char OUTPUT7_NAME[RB_MAX_LEN];
        char OUTPUT8_NAME[RB_MAX_LEN];

        ISwitchVectorProperty Output1SP;
        ISwitch Output1S[SWITCH_TOGGLE_COUNT];
        ISwitchVectorProperty Output2SP;
        ISwitch Output2S[SWITCH_TOGGLE_COUNT];
        ISwitchVectorProperty Output3SP;
        ISwitch Output3S[SWITCH_TOGGLE_COUNT];
        ISwitchVectorProperty Output4SP;
        ISwitch Output4S[SWITCH_TOGGLE_COUNT];
        ISwitchVectorProperty Output5SP;
        ISwitch Output5S[SWITCH_TOGGLE_COUNT];
        ISwitchVectorProperty Output6SP;
        ISwitch Output6S[SWITCH_TOGGLE_COUNT];
        ISwitchVectorProperty Output7SP;
        ISwitch Output7S[SWITCH_TOGGLE_COUNT];
        ISwitchVectorProperty Output8SP;
        ISwitch Output8S[SWITCH_TOGGLE_COUNT];
        ITextVectorProperty Output_Name1TP;
        IText Output_Name1T[1] {};
        ITextVectorProperty Output_Name2TP;
        IText Output_Name2T[1] {};
        ITextVectorProperty Output_Name3TP;
        IText Output_Name3T[1] {};
        ITextVectorProperty Output_Name4TP;
        IText Output_Name4T[1] {};
        ITextVectorProperty Output_Name5TP;
        IText Output_Name5T[1] {};
        ITextVectorProperty Output_Name6TP;
        IText Output_Name6T[1] {};
        ITextVectorProperty Output_Name7TP;
        IText Output_Name7T[1] {};
        ITextVectorProperty Output_Name8TP;
        IText Output_Name8T[1] {};

        // Manual tab controls
        //--------------------

        // Debug only
        ITextVectorProperty Arbitary_CommandTP;
        IText Arbitary_CommandT[1];
        // Debug only end
        
};

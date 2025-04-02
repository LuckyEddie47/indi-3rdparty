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

#include <time.h>           // for nsleep() 
#include <errno.h>          // for nsleep() 

#define CDRIVER_VERSION_MAJOR           1
#define CDRIVER_VERSION_MINOR           0
#define RB_MAX_LEN 64
#define CMD_MAX_LEN 32
enum ResponseErrors {RES_ERR_FORMAT = -1001};

// General commands
//-----------------

// Get Product (compatibility)
#define Osa_handshake ":GVP#"
// Returns: On-Step#

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

        // Manual tab controls
        //--------------------

        // Debug only
        ITextVectorProperty Arbitary_CommandTP;
        IText Arbitary_CommandT[1];
        // Debug only end
        
};

/*
    Ed Lee (ed@thefamilee.co.uk)

    Copyright (C) 2025 Jasem Mutlaq (mutlaqja@ikarustech.com) (Contributors, above)

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

    ===========================================

    Version 1.0
    Based on the lx200_onstep INDI driver with the telescope stripped out

*/

#pragma once

//#include "indicom.h"
#include "indifocuser.h"
#include "indiweatherinterface.h"
#include "indirotatorinterface.h"
//#include "defaultdevice.h"
//#include "connectionplugins/connectiontcp.h"


#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

#define RB_MAX_LEN 64
#define CMD_MAX_LEN 32

enum ResponseErrors {RES_ERR_FORMAT = -1001};

/**********************************************************************
OnStep lexicon
Extracted from the OnStep Groups.IO Wiki
Note all commands sent and responses returned terminate with a # symbol
These are stripped from returned char* by their retrieving functions
An unterminated 0 is returned from unconfigured items
**********************************************************************/

// General commands
//-----------------

// Get Product (compatibility)
#define OS_handshake ":GVP#"
// Returns: On-Step#

// Get firmware version number
#define OS_get_firmware ":GVN#"
// Returns: firmware_string# for example 10.26g#

// Set the UTC Date and Time
// ":SC[MM/DD/YYYY,HH:MM:SS]#"
// Example: SC03/31/2023,13:22:00#
// Returns: 0# on failure, 1# on success

// Set USB Baud Rate where n is an ASCII digit (1..9) with the following interpertation
// 0=115.2K, 1=56.7K, 2=38.4K, 3=28.8K, 4=19.2K, 5=14.4K, 6=9600, 7=4800, 8=2400, 9=1200
// ":SB[n]#"
// Returns: 1# (at the current baud rate and then changes to the new rate for further communication)

// Focuser commands
//-----------------

// Get the defined focusers
#define OS_get_defined_focusers_part ":FA"
// Returns: 1-6 if defined, or 0 if undefined

// Move focuser relative
#define OS_move_focuser_rel_part ":FR"
// Returns: nothing

// Move focuser absolute
#define OS_move_focuser_abs_part ":FS"
// Returns: 1 on sucess, 0 on failure

// Stop the focuser
#define OS_stop_focuser ":FQ#"
// Returns: nothing

// Get the focuser current position
#define OS_get_focuser_position ":FG#"
// Returns: n

// Get the foxuser status
#define OS_get_focuser_status ":FT#"
// Returns: M# for Moving, S# for Stopped

// Get the focuser min position (full in)
#define OS_get_focuser_min ":FI#"
// Returns: n# (microns)

// Get the focuser max position (full out)
#define OS_get_focuser_max ":FM#"
// Returns: n# (microns)

// Get the focuser temperature
#define OS_get_focuser_temperature ":Ft#"
// Returns: n# (deg C)

// Get focuser differential temperature
#define OS_get_focuser_diff_temperature ":Fe#"
// Returns: n# (deg C)

// Get focuser temp compr coefficient
#define OS_get_focuser_temp_comp_coef ":FC#"
// Returns: n.n# (microns per deg C)

// Get focuser temp comp deadband
#define OS_get_focuser_deadband ":FD#"
// Returns: n#

// Get focuser temp comp enabled
#define OS_get_focuser_temp_comp_en ":Fc#"

// Rotator commands
//-----------------

// Get defined rotator
#define OS_get_defined_rotator ":rA#"
// Returns: 1 if defined, or 0 if undefined

// Get rotator angle
#define OS_get_rotator_angle ":rG#"
// Returns: sDD*MM#

// Get rotator min angle
#define OS_get_rotator_min ":rI#"
// Returns: n.n#

// Get rotator max angle
#define OS_get_rotator_max ":rM#"
// Returns: n.n#

// Get rotator status
#define OS_get_rotator_status ":rT#"
// Returns: [M]oving, [S]topped, [D]e-rotating, [R]everse de-rotating

// Set rotator angle (move to)
#define OS_set_rotator_angle_part ":rS"
// Returns: 1 on success, 0 on failure

// Weather commands
//-----------------

// Get the weather temperature in deg. C
#define OS_get_temperature ":GX9A#"
// Returns: +/-n.n# if supported, 0 or nan if unsupported

// Get the weather pressure in mb
#define OS_get_pressure ":GX9B#"
// Returns: +/-n.n# if supported, 0 or nan if unsupported

// Get the weather relative humidity in %
#define OS_get_humidity ":GX9C#"
// Returns: +/-n.n# if supported, 0 or nan if unsupported

// Get the weather dew point in deg. C
#define OS_get_dew_point ":GX9E#"
// Returns: +/-n.n# if supported, 0 or nan if unsupported

// Get the internal MCU temperature in deg. C
#define OS_get_MCU_temperature ":GX9F#"
// Returns: +/-n.n# if supported, 0 or nan if unsupported

// Auxiliary features commands
//---------------------------
#define OS_get_defined_features ":GXY0#"
// Returns: 00000000# 1 if defined, or 0 if undefined

// For dynamically assembled commands
//-----------------------------------
#define OS_command_terminator "#"

/*****************
OnStep lexicon end
*****************/

class OnStep_Aux : public INDI::DefaultDevice, public INDI::FocuserInterface, public INDI::RotatorInterface, public INDI::WeatherInterface
{
  public:
    OnStep_Aux();
    ~OnStep_Aux() {}


    virtual bool initProperties() override;
//    virtual void ISGetProperties(const char *dev) override;
    virtual bool updateProperties() override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev,const char *name,char *texts[],char *names[],int n) override;

  protected:
    virtual const char *getDefaultName() override;
    virtual bool Connect() override;
    virtual bool Disconnect() override;
    virtual bool saveConfigItems(FILE *fp) override;
//    virtual void Init_Outputs();

    //Command processing
    bool sendOSCommand(const char *cmd);
    bool sendOSCommandBlind(const char *cmd);
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

  private:
    bool Handshake();

    static constexpr const float minimum_OS_fw = 10.25;
    static constexpr const int conversion_error = -10000;
    long int OSTimeoutSeconds = 0;
    long int OSTimeoutMicroSeconds = 100000;

    // For Serial and TCP connections
    int PortFD {-1};
    Connection::Serial * serialConnection{nullptr};
    Connection::TCP * tcpConnection{nullptr};

    // Capability queries on connection
    void GetCapabilites();
    int hasFocusers = 0;
    bool hasRotator = false;
    bool hasWeather = false;
    bool hasFeatures = false;

    // Command sequence enforcement
    bool waitingForResponse = false;

    enum {
        ON_SWITCH,
        OFF_SWITCH,
        SWITCH_TOGGLE_COUNT
    };

    //FocuserInterface

    IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration) override;
    IPState MoveAbsFocuser (uint32_t targetTicks) override;
    IPState MoveRelFocuser (FocusDirection dir, uint32_t ticks) override;
    bool AbortFocuser () override;
    int MAX_FOCUSERS = 6;

    //RotatorInterface

    IPState MoveRotator(double angle) override;
    IPState HomeRotator() override;
    bool AbortRotator() override;
    bool SetRotatorBacklash (int32_t steps) override;
    bool SetRotatorBacklashEnabled(bool enabled) override;

    //Outputs
 //   IPState OSEnableOutput(int output);
 //   IPState OSDisableOutput(int output);
 //   bool OSGetOutputState(int output);



    int OSUpdateFocuser(); //Return = 0 good, -1 = Communication error
    int OSUpdateRotator(); //Return = 0 good, -1 = Communication error

    ITextVectorProperty ObjectInfoTP;
    IText ObjectInfoT[1] {};

    ITextVectorProperty VersionTP;
    IText VersionT[5] {};

    // OnStep Status controls
    ITextVectorProperty OnstepStatTP;
    IText OnstepStat[11] {};

    bool TMCDrivers = true; //Set to false if it doesn't detect TMC_SPI reporting. (Small delay on connection/first update)
    bool OSHighPrecision = false;

    // Focuser controls
    // Focuser 1
//    bool OSFocuser1 = false;
    ISwitchVectorProperty OSFocus1InitializeSP;
    ISwitch OSFocus1InitializeS[4];

    // Focus T° Compensation
    INumberVectorProperty FocusTemperatureNP;
    INumber FocusTemperatureN[2];

    ISwitchVectorProperty TFCCompensationSP;
    ISwitch TFCCompensationS[2];
    INumberVectorProperty TFCCoefficientNP;
    INumber TFCCoefficientN[1];
    INumberVectorProperty TFCDeadbandNP;
    INumber TFCDeadbandN[1];
    // End Focus T° Compensation

//    int OSNumFocusers = 0;
    ISwitchVectorProperty OSFocusSelectSP;
    ISwitch OSFocusSelectS[9];

    // Focuser 2
    //ISwitchVectorProperty OSFocus2SelSP;
    //ISwitch OSFocus2SelS[2];
//    bool OSFocuser2 = false;
//    ISwitchVectorProperty OSFocus2RateSP;
//    ISwitch OSFocus2RateS[4];
//
//    ISwitchVectorProperty OSFocus2MotionSP;
//    ISwitch OSFocus2MotionS[3];
//
//    INumberVectorProperty OSFocus2TargNP;
//    INumber OSFocus2TargN[1];

    //Rotator - Some handled by RotatorInterface, but that's mostly for rotation only, absolute, and... very limited.
    bool OSRotator1 = false; //Change to false after detection code
    ISwitchVectorProperty OSRotatorRateSP;
    ISwitch OSRotatorRateS[4]; //Set rate

    ISwitchVectorProperty OSRotatorDerotateSP;
    ISwitch OSRotatorDerotateS[2]; //On or Off

    int IsTracking = 0;
    uint32_t m_RememberPollingPeriod {1000};

    // Reticle +/- Buttons
    ISwitchVectorProperty ReticSP;
    ISwitch ReticS[2];

    ISwitchVectorProperty OSOutput1SP;
    ISwitch OSOutput1S[2];
    ISwitchVectorProperty OSOutput2SP;
    ISwitch OSOutput2S[2];

 //   INumber OutputPorts[PORTS_COUNT];
    INumberVectorProperty OutputPorts_NP;
    bool OSHasOutputs = true;

    char OSStat[RB_MAX_LEN];
    char OldOSStat[RB_MAX_LEN];

    // Weather support
    // NOTE: Much is handled by WeatherInterface, these controls are mainly for setting values which are not detected
    // As of right now, if there is a sensor the values will be overwritten on the next update
    bool OSCpuTemp_good =
        true; //This can fail on some processors and take the timeout before an update, so if it fails, don't check again.


    INumberVectorProperty OSSetTemperatureNP;
    INumber OSSetTemperatureN[1];
    INumberVectorProperty OSSetHumidityNP;
    INumber OSSetHumidityN[1];
    INumberVectorProperty OSSetPressureNP;
    INumber OSSetPressureN[1];
    //Not sure why this would be used, but will feed to it from site settings
    INumberVectorProperty OSSetAltitudeNP;
    INumber OSSetAltitudeN[1];


    //This is updated via other commands, as such I'm going to ignore it like some others do.
    virtual IPState updateWeather() override
    {
        return IPS_OK;
    }


    // Timer for slow updates, once per minute
 //   INDI::Timer SlowTimer;



    // Weather tab controls
    //---------------------
    bool weather_tab_enabled = false;
    int wind_speed_threshold = 0;
    int diff_temp_threshold = 0;

    enum {
        WEATHER_TEMPERATURE,
        WEATHER_PRESSURE,
        WEATHER_HUMIDITY,
        WEATHER_DEW_POINT,
        WEATHER_MEASUREMENTS_COUNT
    };

    int weather_enabled[WEATHER_MEASUREMENTS_COUNT];

    ITextVectorProperty Weather_CloudTP;
    IText Weather_CloudT[1];
    ITextVectorProperty Weather_SkyTP;
    IText Weather_SkyT[1];
    ITextVectorProperty Weather_Sky_TempTP;
    IText Weather_Sky_TempT[1];

    // Status tab controls
    //--------------------
    enum {
        STATUS_FIRMWARE,
        STATUS_ITEMS_COUNT
    };
    ITextVectorProperty Status_ItemsTP;
    IText Status_ItemsT[STATUS_ITEMS_COUNT] {};

    // Features tab controls
    //----------------------
    ISwitchVectorProperty Feature1SP;
    ISwitch Feature1S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Feature2SP;
    ISwitch Feature2S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Feature3SP;
    ISwitch Feature3S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Feature4SP;
    ISwitch Feature4S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Feature5SP;
    ISwitch Feature5S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Feature6SP;
    ISwitch Feature6S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Feature7SP;
    ISwitch Feature7S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Feature8SP;
    ISwitch Feature8S[SWITCH_TOGGLE_COUNT];
    ITextVectorProperty Feature_Name1TP;
    IText Feature_Name1T[1] {};
    ITextVectorProperty Feature_Name2TP;
    IText Feature_Name2T[1] {};
    ITextVectorProperty Feature_Name3TP;
    IText Feature_Name3T[1] {};
    ITextVectorProperty Feature_Name4TP;
    IText Feature_Name4T[1] {};
    ITextVectorProperty Feature_Name5TP;
    IText Feature_Name5T[1] {};
    ITextVectorProperty Feature_Name6TP;
    IText Feature_Name6T[1] {};
    ITextVectorProperty Feature_Name7TP;
    IText Feature_Name7T[1] {};
    ITextVectorProperty Feature_Name8TP;
    IText Feature_Name8T[1] {};

    // Debug only
    ITextVectorProperty Arbitary_CommandTP;
    IText Arbitary_CommandT[1];
    // Debug only end

};

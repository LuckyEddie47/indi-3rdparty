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

    Version 0.2
    Based on the lx200_onstep INDI driver with the telescope stripped out

*/

#pragma once

#include "defaultdevice.h"
#include "indifocuserinterface.h"
#include "indirotatorinterface.h"
#include "indiweatherinterface.h"

#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

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
#define OS_get_defined_focusers ":FA#"
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
#define OS_get_defined_rotator ":GX98#"
// Returns: R if rotate only, D if de-rotate as well, or N if undefined

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

// Set rotator angle (move to) - pass n
#define OS_set_rotator_angle_part ":rS"
// Returns: 1 on success, 0 on failure

// Get rotator backlash
#define OS_get_rotator_backlash ":rb#"
// Returns n# in steps

// Set rotator backlash - pass n
#define OS_set_rotator_backlash_part ":rb"
// Returns 0 on failure, 1 on sucess

// Set rotator move to home
#define OS_move_rotator_home ":rC#"
// Returns nothing

// Set rotator to stop
#define OS_stop_rotator ":rQ#"
// Returns nothing

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

// Auxiliary features commands
//----------------------------

// Get defined features
#define OS_get_defined_features ":GXY0#"
// Returns: 00000000# 1 if defined, or 0 if undefined

// Get feature definition - pass 1-8
#define OS_get_feature_definiton_part ":GXY"
// Returns string,n where string = feature name and n = FEATURE_TYPE enum

// Get feature state - pass 1-8
#define OS_get_feature_state_part ":GXX"
// Returns 0 or 1 for SWITCH | MOMENTARY_SWITCH | COVER_SWITCH
// 0-255 for ANALOG_OUTPUT
// 0 or 1,x.x,y.y,z.z for DEW_HEATER (enabled,zero point, span, temp - dew point)
// m,n.n,p.p,q for INTERVALOMETER (currentCount,exposure,delay,targetCount)

// Set feature state
// :SXX[n]parameters where n = feature number 1-8
// Parameters = V0 or V1 for SWITCH | MOMENTARY_SWITCH | COVER_SWITCH
// V0-V255 for ANALOG_OUTPUT
// V0 or V1 (enable) or Zn.n (zero degC) or Sn.n (span degC) for DEW_HEATER
// V0 or V1 (enable) or En.n (exposure secs) or Dn.n (delay secs) or Cn (target count) for INTERVALOMETER
#define OS_set_feature_part ":SXX"
#define OS_set_feature_enabled_part "V"
#define OS_set_feature_analog_level_part "V"
#define OS_set_dew_zero_part "Z"
#define OS_set_dew_span_part "S"
#define OS_set_intervalometer_exposure_part "E"
#define OS_set_intervalometer_delay_part "D"
#define OS_set_intervalometer_count_part "C"
// Returns 0 on failure, 1 on sucess

// USB_switcher plugin commands
//-----------------------------

// Get defined USB ports
#define OS_get_defined_USBports ":GUY0#"
// Returns: 00000000# 1 if defined, or 0 if undefined

// Get USB port names - pass 1-8
#define OS_get_USBport_name_part ":GUY"
// Returns USB port name

// Get USB port state - pass 1-8
#define OS_get_USBport_state_part ":GUX"
// Returns 0 or 1 for Off / On

// Set feature state
// :SUX[n]parameters where n = USBport number 1-8
// :SUX0parameters = switch all enambled USB ports
// Parameters = V0 or V1 for Off / On
#define OS_set_USBport_part ":SUX"
#define OS_set_USBport_enabled_part "V"
// Returns 0 on failure, 1 on sucess

// For dynamically assembled commands
//-----------------------------------
#define OS_command_terminator "#"

/*****************
OnStep lexicon end
*****************/

class OnStep_Aux : public INDI::DefaultDevice, public INDI::FocuserInterface, public INDI::RotatorInterface,
                   public INDI::WeatherInterface

{
  public:
    OnStep_Aux();
    virtual ~OnStep_Aux() = default;

    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev,const char *name,char *texts[],char *names[],int n) override;

  protected:
    virtual const char *getDefaultName() override;
    virtual bool Connect() override;
    virtual bool Disconnect() override;
    virtual bool saveConfigItems(FILE *fp) override;
    virtual IPState updateWeather() override;
    virtual void TimerHit() override;

    typedef enum {
        CONNECTION_NONE = 1 << 0,
        CONNECTION_SERIAL = 1 << 1,
        CONNECTION_TCP = 1 << 2
    } OSAConnection;

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
    void SlowTimerHit();

    /***********************************************
    * Helper function to check for enum in an array
    * ********************************************/
    template<typename EnumType, typename ArrayType, std::size_t N>
    bool findEnumInArray(const ArrayType (&array_name)[N], EnumType enumValue) {
        auto it = std::find_if(std::begin(array_name), std::end(array_name),
                               [enumValue](ArrayType value) {
                                   return value == static_cast<ArrayType>(enumValue);
                               });
        return it != std::end(array_name);
    }

    static constexpr const int conversion_error = -10000;
    long int OSTimeoutSeconds = 0;
    long int OSTimeoutMicroSeconds = 100000;

    // For Serial and TCP connections
    Connection::Serial *serialConnection { nullptr };
    Connection::TCP *tcpConnection { nullptr };
    int PortFD { -1 };
    uint8_t osConnection { CONNECTION_SERIAL | CONNECTION_TCP };

    // Capability queries on connection
    void GetCapabilites();
    bool hasFocuser = false;
    bool hasRotator = false;
    bool hasWeather = false;
    bool hasFeature = false;
    // These four are all subsets of hasFeature
    bool hasSwitch = false;
    bool hasDew = false;
    bool hasIntervalometer = false;
    bool hasOutput = false;

    bool hasUSB = false;

    // Command sequence enforcement
    bool waitingForResponse = false;

    // FocuserInterface
    IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration) override;
    IPState MoveAbsFocuser (uint32_t targetTicks) override;
    IPState MoveRelFocuser (FocusDirection dir, uint32_t ticks) override;
    bool AbortFocuser () override;
    int MAX_FOCUSERS = 6;

    // RotatorInterface
    IPState MoveRotator(double angle) override;
    IPState HomeRotator() override;
    bool AbortRotator() override;
    bool SetRotatorBacklash (int32_t steps) override;
    bool SetRotatorBacklashEnabled(bool enabled) override;

    int OSUpdateFocuser(); //Return = 0 good, -1 = Communication error
    int OSUpdateRotator(); //Return = 0 good, -1 = Communication error

    ITextVectorProperty ObjectInfoTP;
    IText ObjectInfoT[1] {};

    ITextVectorProperty VersionTP;
    IText VersionT[1] {};

    // Focuser controls
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

    ISwitchVectorProperty OSFocusSelectSP;
    ISwitch OSFocusSelectS[9];

    // Rotator - Some handled by RotatorInterface, but that's mostly for rotation only, absolute, and... very limited.
    ISwitchVectorProperty OSRotatorRateSP;
    ISwitch OSRotatorRateS[4]; //Set rate

    ISwitchVectorProperty OSRotatorDerotateSP;
    ISwitch OSRotatorDerotateS[2]; //On or Off

    int IsTracking = 0;
    uint32_t m_RememberPollingPeriod {1000};

    // Reticle +/- Buttons
    ISwitchVectorProperty ReticSP;
    ISwitch ReticS[2];

     char OSStat[RB_MAX_LEN];
    char OldOSStat[RB_MAX_LEN];

    // Weather tab controls
    //---------------------
    bool weather_tab_enabled = false;

    enum {
        WEATHER_TEMPERATURE,
        WEATHER_PRESSURE,
        WEATHER_HUMIDITY,
        WEATHER_DEW_POINT,
        WEATHER_MEASUREMENTS_COUNT
    };

    int weather_enabled[WEATHER_MEASUREMENTS_COUNT];

    // Status tab controls
    //--------------------
    enum {
        STATUS_FIRMWARE,
        STATUS_ITEMS_COUNT
    };
    ITextVectorProperty Status_ItemsTP;
    IText Status_ItemsT[STATUS_ITEMS_COUNT] {};

    // Features
    static const int max_features = 8;
    enum feature_types{
        OFF,
        SWITCH,
        ANALOG_OUTPUT,
        ANALOG_OUT,
        DEW_HEATER,
        INTERVALOMETER,
        MOMENTARY_SWITCH,
        HIDDEN_SWITCH,
        COVER_SWITCH,
        FEATURE_TYPE_COUNT
    };

    int features_enabled[max_features] = {0};
    feature_types features_type[max_features] = {OFF};
    std::string features_name[max_features];


    // Switches tab controls
    //----------------------
    enum {
        OFF_SWITCH,
        ON_SWITCH,
        SWITCH_TOGGLE_COUNT
    };

    enum {
        SWITCH_DISABLE,
        SWITCH_ENABLE,
        SWITCH_STATES_COUNT
    };

    ISwitchVectorProperty Switch1SP;
    ISwitch Switch1S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Switch2SP;
    ISwitch Switch2S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Switch3SP;
    ISwitch Switch3S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Switch4SP;
    ISwitch Switch4S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Switch5SP;
    ISwitch Switch5S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Switch6SP;
    ISwitch Switch6S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Switch7SP;
    ISwitch Switch7S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty Switch8SP;
    ISwitch Switch8S[SWITCH_TOGGLE_COUNT];
    ITextVectorProperty Switch1_nameTP;
    IText Switch1_nameT[1] {};
    ITextVectorProperty Switch2_nameTP;
    IText Switch2_nameT[1] {};
    ITextVectorProperty Switch3_nameTP;
    IText Switch3_nameT[1] {};
    ITextVectorProperty Switch4_nameTP;
    IText Switch4_nameT[1] {};
    ITextVectorProperty Switch5_nameTP;
    IText Switch5_nameT[1] {};
    ITextVectorProperty Switch6_nameTP;
    IText Switch6_nameT[1] {};
    ITextVectorProperty Switch7_nameTP;
    IText Switch7_nameT[1] {};
    ITextVectorProperty Switch8_nameTP;
    IText Switch8_nameT[1] {};

    // Dew Heaters tab controls
    //-------------------------
    ITextVectorProperty Dew1TP;
    IText Dew1_nameT[1] {};
    ISwitchVectorProperty Dew1SP;
    ISwitch Dew1_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Dew1NP;
    INumber Dew1_zeroN[1];
    INumber Dew1_spanN[1];
    ITextVectorProperty Dew1deltaTP;
    IText Dew1_deltaT[1];
    ITextVectorProperty Dew2TP;
    IText Dew2_nameT[1] {};
    ISwitchVectorProperty Dew2SP;
    ISwitch Dew2_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Dew2NP;
    INumber Dew2_zeroN[1];
    INumber Dew2_spanN[1];
    ITextVectorProperty Dew2deltaTP;
    IText Dew2_deltaT[1];
    ITextVectorProperty Dew3TP;
    IText Dew3_nameT[1] {};
    ISwitchVectorProperty Dew3SP;
    ISwitch Dew3_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Dew3NP;
    INumber Dew3_zeroN[1];
    INumber Dew3_spanN[1];
    ITextVectorProperty Dew3deltaTP;
    IText Dew3_deltaT[1];
    ITextVectorProperty Dew4TP;
    IText Dew4_nameT[1] {};
    ISwitchVectorProperty Dew4SP;
    ISwitch Dew4_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Dew4NP;
    INumber Dew4_zeroN[1];
    INumber Dew4_spanN[1];
    ITextVectorProperty Dew4deltaTP;
    IText Dew4_deltaT[1];
    ITextVectorProperty Dew5TP;
    IText Dew5_nameT[1] {};
    ISwitchVectorProperty Dew5SP;
    ISwitch Dew5_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Dew5NP;
    INumber Dew5_zeroN[1];
    INumber Dew5_spanN[1];
    ITextVectorProperty Dew5deltaTP;
    IText Dew5_deltaT[1];
    ITextVectorProperty Dew6TP;
    IText Dew6_nameT[1] {};
    ISwitchVectorProperty Dew6SP;
    ISwitch Dew6_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Dew6NP;
    INumber Dew6_zeroN[1];
    INumber Dew6_spanN[1];
    ITextVectorProperty Dew6deltaTP;
    IText Dew6_deltaT[1];
    ITextVectorProperty Dew7TP;
    IText Dew7_nameT[1] {};
    ISwitchVectorProperty Dew7SP;
    ISwitch Dew7_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Dew7NP;
    INumber Dew7_zeroN[1];
    INumber Dew7_spanN[1];
    ITextVectorProperty Dew7deltaTP;
    IText Dew7_deltaT[1];
    ITextVectorProperty Dew8TP;
    IText Dew8_nameT[1] {};
    ISwitchVectorProperty Dew8SP;
    ISwitch Dew8_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Dew8NP;
    INumber Dew8_zeroN[1];
    INumber Dew8_spanN[1];
    ITextVectorProperty Dew8deltaTP;
    IText Dew8_deltaT[1];

    // Intervalometer tab controls
    //----------------------------
    ITextVectorProperty Inter1TP;
    IText Inter1_nameT[1] {};
    ISwitchVectorProperty Inter1SP;
    ISwitch Inter1_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Inter1NP;
    INumber Inter1_expN[1];
    INumber Inter1_delayN[1];
    INumber Inter1_countN[1];
    ITextVectorProperty Inter1doneTP;
    IText Inter1_doneT[1];
    ITextVectorProperty Inter2TP;
    IText Inter2_nameT[1] {};
    ISwitchVectorProperty Inter2SP;
    ISwitch Inter2_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Inter2NP;
    INumber Inter2_expN[1];
    INumber Inter2_delayN[1];
    INumber Inter2_countN[1];
    ITextVectorProperty Inter2doneTP;
    IText Inter2_doneT[1];
    ITextVectorProperty Inter3TP;
    IText Inter3_nameT[1] {};
    ISwitchVectorProperty Inter3SP;
    ISwitch Inter3_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Inter3NP;
    INumber Inter3_expN[1];
    INumber Inter3_delayN[1];
    INumber Inter3_countN[1];
    ITextVectorProperty Inter3doneTP;
    IText Inter3_doneT[1];
    ITextVectorProperty Inter4TP;
    IText Inter4_nameT[1] {};
    ISwitchVectorProperty Inter4SP;
    ISwitch Inter4_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Inter4NP;
    INumber Inter4_expN[1];
    INumber Inter4_delayN[1];
    INumber Inter4_countN[1];
    ITextVectorProperty Inter4doneTP;
    IText Inter4_doneT[1];
    ITextVectorProperty Inter5TP;
    IText Inter5_nameT[1] {};
    ISwitchVectorProperty Inter5SP;
    ISwitch Inter5_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Inter5NP;
    INumber Inter5_expN[1];
    INumber Inter5_delayN[1];
    INumber Inter5_countN[1];
    ITextVectorProperty Inter5doneTP;
    IText Inter5_doneT[1];
    ITextVectorProperty Inter6TP;
    IText Inter6_nameT[1] {};
    ISwitchVectorProperty Inter6SP;
    ISwitch Inter6_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Inter6NP;
    INumber Inter6_expN[1];
    INumber Inter6_delayN[1];
    INumber Inter6_countN[1];
    ITextVectorProperty Inter6doneTP;
    IText Inter6_doneT[1];
    ITextVectorProperty Inter7TP;
    IText Inter7_nameT[1] {};
    ISwitchVectorProperty Inter7SP;
    ISwitch Inter7_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Inter7NP;
    INumber Inter7_expN[1];
    INumber Inter7_delayN[1];
    INumber Inter7_countN[1];
    ITextVectorProperty Inter7doneTP;
    IText Inter7_doneT[1];
    ITextVectorProperty Inter8TP;
    IText Inter8_nameT[1] {};
    ISwitchVectorProperty Inter8SP;
    ISwitch Inter8_enableS[SWITCH_TOGGLE_COUNT];
    INumberVectorProperty Inter8NP;
    INumber Inter8_expN[1];
    INumber Inter8_delayN[1];
    INumber Inter8_countN[1];
    ITextVectorProperty Inter8doneTP;
    IText Inter8_doneT[1];

    // USB tab controls
    //-----------------
    static const int max_USBports = 8;
    int USBports_enabled[max_USBports] = {0};
    std::string USBports_name[max_USBports];
    int USBportCount = 0;

    ISwitchVectorProperty USBallSP;
    ISwitch USBallS[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty USB1SP;
    ISwitch USB1S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty USB2SP;
    ISwitch USB2S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty USB3SP;
    ISwitch USB3S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty USB4SP;
    ISwitch USB4S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty USB5SP;
    ISwitch USB5S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty USB6SP;
    ISwitch USB6S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty USB7SP;
    ISwitch USB7S[SWITCH_TOGGLE_COUNT];
    ISwitchVectorProperty USB8SP;
    ISwitch USB8S[SWITCH_TOGGLE_COUNT];
    ITextVectorProperty USB1_nameTP;
    IText USB1_nameT[1] {};
    ITextVectorProperty USB2_nameTP;
    IText USB2_nameT[1] {};
    ITextVectorProperty USB3_nameTP;
    IText USB3_nameT[1] {};
    ITextVectorProperty USB4_nameTP;
    IText USB4_nameT[1] {};
    ITextVectorProperty USB5_nameTP;
    IText USB5_nameT[1] {};
    ITextVectorProperty USB6_nameTP;
    IText USB6_nameT[1] {};
    ITextVectorProperty USB7_nameTP;
    IText USB7_nameT[1] {};
    ITextVectorProperty USB8_nameTP;
    IText USB8_nameT[1] {};

    // Debug only
    // ITextVectorProperty Arbitary_CommandTP;
    // IText Arbitary_CommandT[1];
    // Debug only end

};

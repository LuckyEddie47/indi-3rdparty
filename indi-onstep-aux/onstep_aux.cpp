/*
    OnStep Aux
    A driver to support all ancillary features of OnStepX minus telescope control
    Large parts copied from the LX200_OnStep driver

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
#include "connectionplugins/connectioninterface.h"
#include "indicom.h"

#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <mutex>

// Debug only - required for attaching debugger
//  #include <signal.h>
//  #include <unistd.h>
// Debug only end

// Additional tabs
#define ROTATOR_TAB "Rotator"
#define WEATHER_TAB "Weather"
#define SWITCH_TAB "Switches"
#define DEW_HEATERS_TAB "Dew Heaters"
#define INTERVALOMETER_TAB "Intervalometers"
#define USB_TAB "USB Ports"
#define OUTPUT_TAB "Ouputs"
#define MANUAL_TAB "Manual"

// Define auto pointer to ourselves
std::unique_ptr<OnStep_Aux> OnStepAux(new OnStep_Aux());

// Mutex for communications
std::mutex osCommsLock;

OnStep_Aux::OnStep_Aux() : INDI::DefaultDevice(), FI(this),  RI(this), WI(this)//, PI(this)
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
 * Defines all Properties and Interfaces
 **********************************************/
bool OnStep_Aux::initProperties()
{
    DefaultDevice::initProperties();
    setDriverInterface(FOCUSER_INTERFACE | ROTATOR_INTERFACE | WEATHER_INTERFACE | POWER_INTERFACE | AUX_INTERFACE);

    // MAIN_CONTROL_TAB
    //-----------------
    IUFillText(&VersionT[0], "Version", "", "");
    IUFillTextVector(&VersionTP, VersionT, 1, getDeviceName(), "Firmware Info", "", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // CONNECTION_TAB
    // OPTIONS_TAB
    // Constructed form Standard Indi aux controls
    //--------------------------------------------
    addAuxControls();

    // FOCUSER_INTERFACE
    //------------------
    FI::initProperties(FOCUS_TAB);

    FocusRelPosNP[0].min = 0.;
    FocusRelPosNP[0].max = 30000.;
    FocusRelPosNP[0].value = 0;
    FocusRelPosNP[0].step = 10;
    FocusAbsPosNP[0].min = 0.;
    FocusAbsPosNP[0].max = 60000.;
    FocusAbsPosNP[0].value = 0;
    FocusAbsPosNP[0].step = 10;

    IUFillSwitch(&OSFocus1InitializeS[0], "Focus1_0", "Zero", ISS_OFF);
    IUFillSwitch(&OSFocus1InitializeS[1], "Focus1_2", "Mid", ISS_OFF);
    IUFillSwitchVector(&OSFocus1InitializeSP, OSFocus1InitializeS, 2, getDeviceName(), "Foc1Rate", "Initialize", FOCUS_TAB,
                       IP_RW, ISR_ATMOST1, 0, IPS_IDLE);
    // Focus T° Compensation
    // Property must be FOCUS_TEMPERATURE to be recognized by Ekos
    IUFillNumber(&FocusTemperatureN[0], "FOCUS_TEMPERATURE", "TFC T°", "%+2.2f", 0, 1, 0.25,
                 25);  //default value is meaningless
    IUFillNumber(&FocusTemperatureN[1], "TFC Δ T°", "TFC Δ T°", "%+2.2f", 0, 1, 0.25, 25);  //default value is meaningless
    IUFillNumberVector(&FocusTemperatureNP, FocusTemperatureN, 2, getDeviceName(), "FOCUS_TEMPERATURE", "Focuser T°",
                       FOCUS_TAB, IP_RO, 0,
                       IPS_IDLE);
    IUFillSwitch(&TFCCompensationS[0], "Off", "Compensation: OFF", ISS_OFF);
    IUFillSwitch(&TFCCompensationS[1], "On", "Compensation: ON", ISS_OFF);
    IUFillSwitchVector(&TFCCompensationSP, TFCCompensationS, 2, getDeviceName(), "Compensation T°", "Temperature Compensation",
                       FOCUS_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);

    IUFillNumber(&TFCCoefficientN[0], "TFC Coefficient", "TFC Coefficient µm/°C", "%+03.5f", -999.99999, 999.99999, 1, 100);
    IUFillNumberVector(&TFCCoefficientNP, TFCCoefficientN, 1, getDeviceName(), "TFC Coefficient", "", FOCUS_TAB, IP_RW, 0,
                       IPS_IDLE);
    IUFillNumber(&TFCDeadbandN[0], "TFC Deadband", "TFC Deadband µm", "%g", 1, 32767, 1, 5);
    IUFillNumberVector(&TFCDeadbandNP, TFCDeadbandN, 1, getDeviceName(), "TFC Deadband", "", FOCUS_TAB, IP_RW, 0, IPS_IDLE);

    // ROTATOR_INTERFACE
    //------------------
    RI::initProperties(ROTATOR_TAB);

    IUFillSwitch(&OSRotatorDerotateS[0], "Derotate_OFF", "OFF", ISS_OFF);
    IUFillSwitch(&OSRotatorDerotateS[1], "Derotate_ON", "ON", ISS_OFF);
    IUFillSwitchVector(&OSRotatorDerotateSP, OSRotatorDerotateS, 2, getDeviceName(), "Derotate_Status", "DEROTATE", ROTATOR_TAB,
                       IP_RW,
                       ISR_ATMOST1, 0, IPS_IDLE);

    // WEATHER_INTERFACE
    //------------------
    WI::initProperties(WEATHER_TAB, WEATHER_TAB);
    addParameter("WEATHER_TEMPERATURE", "Temperature (C)", -40, 50, 15);
    addParameter("WEATHER_HUMIDITY", "Humidity %", 0, 100, 15);
    addParameter("WEATHER_BAROMETER", "Pressure (hPa)", 0, 1500, 15);
    addParameter("WEATHER_DEWPOINT", "Dew Point (C)", 0, 50, 15); // From OnStep
    setCriticalParameter("WEATHER_TEMPERATURE");

    // SWITCH_TAB
    //----------------------
    IUFillTextVector(&Switch1_nameTP, Switch1_nameT, 1, getDeviceName(), "Switch_1_NAME", "Device 1",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch1_nameT[0], "DEVICE_1_NAME", "Name", "");
    IUFillSwitchVector(&Switch1SP, Switch1S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch1", "Device 1",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch1S[ON_SWITCH], "DEVICE1_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch1S[OFF_SWITCH], "DEVICE1_OFF", "OFF", ISS_ON);

    IUFillTextVector(&Switch2_nameTP, Switch2_nameT, 1, getDeviceName(), "Switch_2_NAME", "Device 2",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch2_nameT[0], "DEVICE_2_NAME", "Name", "");
    IUFillSwitchVector(&Switch2SP, Switch2S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch2", "Device 2",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch2S[ON_SWITCH], "DEVICE2_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch2S[OFF_SWITCH], "DEVICE2_OFF", "OFF", ISS_ON);

    IUFillTextVector(&Switch3_nameTP, Switch3_nameT, 1, getDeviceName(), "Switch_3_NAME", "Device 3",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch3_nameT[0], "DEVICE_3_NAME", "Name", "");
    IUFillSwitchVector(&Switch3SP, Switch3S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch3", "Device 3",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch3S[ON_SWITCH], "DEVICE3_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch3S[OFF_SWITCH], "DEVICE3_OFF", "OFF", ISS_ON);

    IUFillTextVector(&Switch4_nameTP, Switch4_nameT, 1, getDeviceName(), "Switch_4_NAME", "Device 4",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch4_nameT[0], "DEVICE_4_NAME", "Name", "");
    IUFillSwitchVector(&Switch4SP, Switch4S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch4", "Device 4",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch4S[ON_SWITCH], "DEVICE4_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch4S[OFF_SWITCH], "DEVICE4_OFF", "OFF", ISS_ON);

    IUFillTextVector(&Switch5_nameTP, Switch5_nameT, 1, getDeviceName(), "Switch_5_NAME", "Device 5",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch5_nameT[0], "DEVICE_5_NAME", "Name", "");
    IUFillSwitchVector(&Switch5SP, Switch5S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch5", "Device 5",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch5S[ON_SWITCH], "DEVICE5_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch5S[OFF_SWITCH], "DEVICE5_OFF", "OFF", ISS_ON);

    IUFillTextVector(&Switch6_nameTP, Switch6_nameT, 1, getDeviceName(), "Switch_6_NAME", "Device 6",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch6_nameT[0], "DEVICE_6_NAME", "Name", "");
    IUFillSwitchVector(&Switch6SP, Switch6S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch6", "Device 6",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch6S[ON_SWITCH], "DEVICE6_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch6S[OFF_SWITCH], "DEVICE6_OFF", "OFF", ISS_ON);

    IUFillTextVector(&Switch7_nameTP, Switch7_nameT, 1, getDeviceName(), "Switch_7_NAME", "Device 7",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch7_nameT[0], "DEVICE_7_NAME", "Name", "");
    IUFillSwitchVector(&Switch7SP, Switch7S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch7", "Device 7",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch7S[ON_SWITCH], "DEVICE7_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch7S[OFF_SWITCH], "DEVICE7_OFF", "OFF", ISS_ON);

    IUFillTextVector(&Switch8_nameTP, Switch8_nameT, 1, getDeviceName(), "Switch_8_NAME", "Device 8",
                     SWITCH_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Switch8_nameT[0], "DEVICE_8_NAME", "Name", "");
    IUFillSwitchVector(&Switch8SP, Switch8S, SWITCH_TOGGLE_COUNT, getDeviceName(), "Switch8", "Device 8",
                       SWITCH_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Switch8S[ON_SWITCH], "DEVICE8_ON", "ON", ISS_OFF);
    IUFillSwitch(&Switch8S[OFF_SWITCH], "DEVICE8_OFF", "OFF", ISS_ON);

    // DEW HEATERS TAB
    //----------------
    IUFillTextVector(&Dew1TP, Dew1_nameT, 1, getDeviceName(), "Dew_1_NAME", "Dew 1",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew1_nameT[0], "DEW_1_NAME", "Name", "");
    IUFillSwitchVector(&Dew1SP, Dew1_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable1", "Enable",
                       DEW_HEATERS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Dew1_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Dew1_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Dew1NP, Dew1_zeroN, 1, getDeviceName(), "Dew_1_SETTINGS", "Settings degC",
                       DEW_HEATERS_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Dew1_zeroN[1],"DEW1_ZERO_POINT","Zero point","%.0f", -5, 20, 0.1, 5);
    IUFillNumber(&Dew1_spanN[1],"DEW1_SPAN","Span range","%.0f", 0, 20, 0.1, 5);
    IUFillTextVector(&Dew1deltaTP, Dew1_deltaT, 1, getDeviceName(), "Dew_1_FEEDBACK", "Delta degC",
                       DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew1_deltaT[0], "DEW_1_DELTA", "Temp-dew", "");

    IUFillTextVector(&Dew2TP, Dew2_nameT, 1, getDeviceName(), "Dew_2_NAME", "Dew 2",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew2_nameT[0], "DEW_2_NAME", "Name", "");
    IUFillSwitchVector(&Dew2SP, Dew2_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable2", "Enable",
                       DEW_HEATERS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Dew2_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Dew2_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Dew2NP, Dew2_zeroN, 1, getDeviceName(), "Dew_2_SETTINGS", "Settings degC",
                       DEW_HEATERS_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Dew2_zeroN[1],"DEW2_ZERO_POINT","Zero point","%.0f", -5, 20, 0.1, 5);
    IUFillNumber(&Dew2_spanN[1],"DEW2_SPAN","Span range","%.0f", 0, 20, 0.1, 5);
    IUFillTextVector(&Dew2deltaTP, Dew2_deltaT, 1, getDeviceName(), "Dew_2_FEEDBACK", "Delta degC",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew2_deltaT[0], "DEW_2_DELTA", "Temp-dew", "");

    IUFillTextVector(&Dew3TP, Dew3_nameT, 1, getDeviceName(), "Dew_3_NAME", "Dew 3",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew3_nameT[0], "DEW_3_NAME", "Name", "");
    IUFillSwitchVector(&Dew3SP, Dew3_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable3", "Enable",
                       DEW_HEATERS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Dew3_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Dew3_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Dew3NP, Dew3_zeroN, 1, getDeviceName(), "Dew_3_SETTINGS", "Settings degC",
                       DEW_HEATERS_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Dew3_zeroN[1],"DEW3_ZERO_POINT","Zero point","%.0f", -5, 20, 0.1, 5);
    IUFillNumber(&Dew3_spanN[1],"DEW3_SPAN","Span range","%.0f", 0, 20, 0.1, 5);
    IUFillTextVector(&Dew3deltaTP, Dew3_deltaT, 1, getDeviceName(), "Dew_3_FEEDBACK", "Delta degC",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew3_deltaT[0], "DEW_3_DELTA", "Temp-dew", "");

    IUFillTextVector(&Dew4TP, Dew4_nameT, 1, getDeviceName(), "Dew_4_NAME", "Dew 4",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew4_nameT[0], "DEW_4_NAME", "Name", "");
    IUFillSwitchVector(&Dew4SP, Dew4_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable4", "Enable",
                       DEW_HEATERS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Dew4_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Dew4_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Dew4NP, Dew4_zeroN, 1, getDeviceName(), "Dew_4_SETTINGS", "Settings degC",
                       DEW_HEATERS_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Dew4_zeroN[1],"DEW4_ZERO_POINT","Zero point","%.0f", -5, 20, 0.1, 5);
    IUFillNumber(&Dew4_spanN[1],"DEW4_SPAN","Span range","%.0f", 0, 20, 0.1, 5);
    IUFillTextVector(&Dew4deltaTP, Dew4_deltaT, 1, getDeviceName(), "Dew_4_FEEDBACK", "Delta degC",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew4_deltaT[0], "DEW_4_DELTA", "Temp-dew", "");

    IUFillTextVector(&Dew5TP, Dew5_nameT, 1, getDeviceName(), "Dew_5_NAME", "Dew 5",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew5_nameT[0], "DEW_5_NAME", "Name", "");
    IUFillSwitchVector(&Dew5SP, Dew5_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable5", "Enable",
                       DEW_HEATERS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Dew5_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Dew5_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Dew5NP, Dew5_zeroN, 1, getDeviceName(), "Dew_5_SETTINGS", "Settings degC",
                       DEW_HEATERS_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Dew5_zeroN[1],"DEW5_ZERO_POINT","Zero point","%.0f", -5, 20, 0.1, 5);
    IUFillNumber(&Dew5_spanN[1],"DEW5_SPAN","Span range","%.0f", 0, 20, 0.1, 5);

    IUFillTextVector(&Dew6TP, Dew6_nameT, 1, getDeviceName(), "Dew_6_NAME", "Dew 6",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew6_nameT[0], "DEW_6_NAME", "Name", "");
    IUFillSwitchVector(&Dew6SP, Dew6_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable6", "Enable",
                       DEW_HEATERS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Dew6_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Dew6_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Dew6NP, Dew6_zeroN, 1, getDeviceName(), "Dew_6_SETTINGS", "Settings degC",
                       DEW_HEATERS_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Dew6_zeroN[1],"DEW6_ZERO_POINT","Zero point","%.0f", -5, 20, 0.1, 5);
    IUFillNumber(&Dew6_spanN[1],"DEW6_SPAN","Span range","%.0f", 0, 20, 0.1, 5);
    IUFillTextVector(&Dew6deltaTP, Dew6_deltaT, 1, getDeviceName(), "Dew_6_FEEDBACK", "Delta degC",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew6_deltaT[0], "DEW_6_DELTA", "Temp-dew", "");

    IUFillTextVector(&Dew7TP, Dew7_nameT, 1, getDeviceName(), "Dew_7_NAME", "Dew 7",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew7_nameT[0], "DEW_6_NAME", "Name", "");
    IUFillSwitchVector(&Dew7SP, Dew7_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable7", "Enable",
                       DEW_HEATERS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Dew7_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Dew7_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Dew7NP, Dew6_zeroN, 1, getDeviceName(), "Dew_7_SETTINGS", "Settings degC",
                       DEW_HEATERS_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Dew7_zeroN[1],"DEW7_ZERO_POINT","Zero point","%.0f", -5, 20, 0.1, 5);
    IUFillNumber(&Dew7_spanN[1],"DEW7_SPAN","Span range","%.0f", 0, 20, 0.1, 5);
    IUFillTextVector(&Dew7deltaTP, Dew7_deltaT, 1, getDeviceName(), "Dew_7_FEEDBACK", "Delta degC",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew7_deltaT[0], "DEW_7_DELTA", "Temp-dew", "");

    IUFillTextVector(&Dew8TP, Dew8_nameT, 1, getDeviceName(), "Dew_8_NAME", "Dew 8",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew8_nameT[0], "DEW_8_NAME", "Name", "");
    IUFillSwitchVector(&Dew8SP, Dew8_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable8", "Enable",
                       DEW_HEATERS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Dew8_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Dew8_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Dew8NP, Dew8_zeroN, 1, getDeviceName(), "Dew_8_SETTINGS", "Settings degC",
                       DEW_HEATERS_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Dew8_zeroN[1],"DEW8_ZERO_POINT","Zero point","%.0f", -5, 20, 0.1, 5);
    IUFillNumber(&Dew8_spanN[1],"DEW8_SPAN","Span range","%.0f", 0, 20, 0.1, 5);
    IUFillTextVector(&Dew8deltaTP, Dew8_deltaT, 1, getDeviceName(), "Dew_8_FEEDBACK", "Delta degC",
                     DEW_HEATERS_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Dew8_deltaT[0], "DEW_8_DELTA", "Temp-dew", "");

    // INTERVALOMETER_TAB
    //-------------------
    IUFillTextVector(&Inter1TP, Inter1_nameT, 1, getDeviceName(), "Inter_1_NAME", "Intervalometer 1",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter1_nameT[0], "INTER_1_NAME", "Name", "");
    IUFillSwitchVector(&Inter1SP, Inter1_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable1", "Enable",
                       INTERVALOMETER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Inter1_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Inter1_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Inter1NP, Inter1_expN, 1, getDeviceName(), "Inter_1_SETTINGS", "Settings secs",
                       INTERVALOMETER_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Inter1_expN[1],"INTER1_EXP","Exposure","%.0f", 0.1, 3600.0, 0.1, 30);
    IUFillNumber(&Inter1_delayN[1],"INTER1_DELAY","Delay","%.0f", 1.0, 3600.0, 1, 1);
    IUFillNumber(&Inter1_countN[1],"INTER1_COUNT","Count","%d", 1, 255, 1, 10);
    IUFillTextVector(&Inter1doneTP, Inter1_doneT, 1, getDeviceName(), "Inter_1_DONE", "Current count",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter1_doneT[0], "INTER1_DONE", "Count", "");

    IUFillTextVector(&Inter2TP, Inter2_nameT, 1, getDeviceName(), "Inter_2_NAME", "Intervalometer 2",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter2_nameT[0], "INTER_2_NAME", "Name", "");
    IUFillSwitchVector(&Inter2SP, Inter2_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable2", "Enable",
                       INTERVALOMETER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Inter2_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Inter2_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Inter2NP, Inter2_expN, 1, getDeviceName(), "Inter_2_SETTINGS", "Settings secs",
                       INTERVALOMETER_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Inter2_expN[1],"INTER2_EXP","Exposure","%.0f", 0.1, 3600.0, 0.1, 30);
    IUFillNumber(&Inter2_delayN[1],"INTER2_DELAY","Delay","%.0f", 1.0, 3600.0, 1, 1);
    IUFillNumber(&Inter2_countN[1],"INTER2_COUNT","Count","%d", 1, 255, 1, 10);
    IUFillTextVector(&Inter2doneTP, Inter2_doneT, 1, getDeviceName(), "Inter_2_DONE", "Current count",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter2_doneT[0], "INTER2_DONE", "Count", "");

    IUFillTextVector(&Inter3TP, Inter3_nameT, 1, getDeviceName(), "Inter_3_NAME", "Intervalometer 3",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter3_nameT[0], "INTER_3_NAME", "Name", "");
    IUFillSwitchVector(&Inter3SP, Inter3_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable3", "Enable",
                       INTERVALOMETER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Inter3_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Inter3_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Inter3NP, Inter3_expN, 1, getDeviceName(), "Inter_3_SETTINGS", "Settings secs",
                       INTERVALOMETER_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Inter3_expN[1],"INTER3_EXP","Exposure","%.0f", 0.1, 3600.0, 0.1, 30);
    IUFillNumber(&Inter3_delayN[1],"INTER3_DELAY","Delay","%.0f", 1.0, 3600.0, 1, 1);
    IUFillNumber(&Inter3_countN[1],"INTER3_COUNT","Count","%d", 1, 255, 1, 10);
    IUFillTextVector(&Inter3doneTP, Inter3_doneT, 1, getDeviceName(), "Inter_3_DONE", "Current count",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter3_doneT[0], "INTER3_DONE", "Count", "");

    IUFillTextVector(&Inter4TP, Inter4_nameT, 1, getDeviceName(), "Inter_4_NAME", "Intervalometer 4",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter4_nameT[0], "INTER_4_NAME", "Name", "");
    IUFillSwitchVector(&Inter4SP, Inter4_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable4", "Enable",
                       INTERVALOMETER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Inter4_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Inter4_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Inter4NP, Inter4_expN, 1, getDeviceName(), "Inter_4_SETTINGS", "Settings secs",
                       INTERVALOMETER_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Inter4_expN[1],"Inter4_EXP","Exposure","%.0f", 0.1, 3600.0, 0.1, 30);
    IUFillNumber(&Inter4_delayN[1],"Inter4_DELAY","Delay","%.0f", 1.0, 3600.0, 1, 1);
    IUFillNumber(&Inter4_countN[1],"Inter4_COUNT","Count","%d", 1, 255, 1, 10);
    IUFillTextVector(&Inter4doneTP, Inter4_doneT, 1, getDeviceName(), "Inter_4_DONE", "Current count",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter4_doneT[0], "INTER4_DONE", "Count", "");

    IUFillTextVector(&Inter5TP, Inter5_nameT, 1, getDeviceName(), "Inter_5_NAME", "Intervalometer 5",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter5_nameT[0], "INTER_5_NAME", "Name", "");
    IUFillSwitchVector(&Inter5SP, Inter5_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable5", "Enable",
                       INTERVALOMETER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Inter5_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Inter5_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Inter5NP, Inter5_expN, 1, getDeviceName(), "Inter_5_SETTINGS", "Settings secs",
                       INTERVALOMETER_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Inter5_expN[1],"Inter5_EXP","Exposure","%.0f", 0.1, 3600.0, 0.1, 30);
    IUFillNumber(&Inter5_delayN[1],"Inter5_DELAY","Delay","%.0f", 1.0, 3600.0, 1, 1);
    IUFillNumber(&Inter5_countN[1],"Inter5_COUNT","Count","%d", 1, 255, 1, 10);
    IUFillTextVector(&Inter5doneTP, Inter5_doneT, 1, getDeviceName(), "Inter_5_DONE", "Current count",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter5_doneT[0], "INTER5_DONE", "Count", "");

    IUFillTextVector(&Inter6TP, Inter6_nameT, 1, getDeviceName(), "Inter_6_NAME", "Intervalometer 6",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter6_nameT[0], "INTER_6_NAME", "Name", "");
    IUFillSwitchVector(&Inter6SP, Inter6_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable6", "Enable",
                       INTERVALOMETER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Inter6_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Inter6_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Inter6NP, Inter6_expN, 1, getDeviceName(), "Inter_6_SETTINGS", "Settings secs",
                       INTERVALOMETER_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Inter6_expN[1],"INTER6_EXP","Exposure","%.0f", 0.1, 3600.0, 0.1, 30);
    IUFillNumber(&Inter6_delayN[1],"INTER6_DELAY","Delay","%.0f", 1.0, 3600.0, 1, 1);
    IUFillNumber(&Inter6_countN[1],"INTER6_COUNT","Count","%d", 1, 255, 1, 10);
    IUFillTextVector(&Inter6doneTP, Inter6_doneT, 1, getDeviceName(), "Inter_6_DONE", "Current count",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter6_doneT[0], "INTER6_DONE", "Count", "");

    IUFillTextVector(&Inter7TP, Inter7_nameT, 1, getDeviceName(), "Inter_7_NAME", "Intervalometer 7",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter7_nameT[0], "INTER_7_NAME", "Name", "");
    IUFillSwitchVector(&Inter7SP, Inter7_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable7", "Enable",
                       INTERVALOMETER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Inter7_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Inter7_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Inter7NP, Inter7_expN, 1, getDeviceName(), "Inter_7_SETTINGS", "Settings secs",
                       INTERVALOMETER_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Inter7_expN[1],"Inter7_EXP","Exposure","%.0f", 0.1, 3600.0, 0.1, 30);
    IUFillNumber(&Inter7_delayN[1],"Inter7_DELAY","Delay","%.0f", 1.0, 3600.0, 1, 1);
    IUFillNumber(&Inter7_countN[1],"Inter7_COUNT","Count","%d", 1, 255, 1, 10);
    IUFillTextVector(&Inter7doneTP, Inter7_doneT, 1, getDeviceName(), "Inter_7_DONE", "Current count",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter7_doneT[0], "INTER7_DONE", "Count", "");

    IUFillTextVector(&Inter8TP, Inter8_nameT, 1, getDeviceName(), "Inter_8_NAME", "Intervalometer 8",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter8_nameT[0], "INTER_8_NAME", "Name", "");
    IUFillSwitchVector(&Inter8SP, Inter8_enableS, SWITCH_TOGGLE_COUNT, getDeviceName(), "Enable8", "Enable",
                       INTERVALOMETER_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&Inter8_enableS[ON_SWITCH], "ENABLE_ON", "ON", ISS_OFF);
    IUFillSwitch(&Inter8_enableS[OFF_SWITCH], "ENABLE_OFF", "OFF", ISS_ON);
    IUFillNumberVector(&Inter8NP, Inter8_expN, 1, getDeviceName(), "Inter_8_SETTINGS", "Settings secs",
                       INTERVALOMETER_TAB, IP_RW, 60, IPS_OK);
    IUFillNumber(&Inter8_expN[1],"INTER8_EXP","Exposure","%.0f", 0.1, 3600.0, 0.1, 30);
    IUFillNumber(&Inter8_delayN[1],"INTER8_DELAY","Delay","%.0f", 1.0, 3600.0, 1, 1);
    IUFillNumber(&Inter8_countN[1],"INTER8_COUNT","Count","%d", 1, 255, 1, 10);
    IUFillTextVector(&Inter8doneTP, Inter8_doneT, 1, getDeviceName(), "Inter_8_DONE", "Current count",
                     INTERVALOMETER_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&Inter8_doneT[0], "INTER8_DONE", "Count", "");

    // USB Tab
    //--------
    IUFillSwitchVector(&USBallSP, USBallS, SWITCH_TOGGLE_COUNT, getDeviceName(),  "USBall",  "USB ALL",
                       USB_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&USBallS[ON_SWITCH],  "USBALL_ON", "ON", ISS_OFF);
    IUFillSwitch(&USBallS[OFF_SWITCH],  "USBALL_OFF", "OFF", ISS_ON);

    IUFillTextVector(&USB1_nameTP, USB1_nameT, 1, getDeviceName(),  "USB_1_NAME",  "USB 1",
                     USB_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&USB1_nameT[0],  "USB_1_NAME", "Name", "");
    IUFillSwitchVector(&USB1SP, USB1S, SWITCH_TOGGLE_COUNT, getDeviceName(),  "USB1",  "USB 1",
                       USB_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&USB1S[ON_SWITCH],  "USB1_ON", "ON", ISS_OFF);
    IUFillSwitch(&USB1S[OFF_SWITCH],  "USB1_OFF", "OFF", ISS_ON);

    IUFillTextVector(&USB2_nameTP, USB2_nameT, 1, getDeviceName(),  "USB_2_NAME",  "USB 2",
                     USB_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&USB2_nameT[0],  "USB_2_NAME", "Name", "");
    IUFillSwitchVector(&USB2SP, USB2S, SWITCH_TOGGLE_COUNT, getDeviceName(),  "USB2",  "USB 2",
                       USB_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&USB2S[ON_SWITCH],  "USB2_ON", "ON", ISS_OFF);
    IUFillSwitch(&USB2S[OFF_SWITCH],  "USB2_OFF", "OFF", ISS_ON);

    IUFillTextVector(&USB3_nameTP, USB3_nameT, 1, getDeviceName(),  "USB_3_NAME",  "USB 3",
                     USB_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&USB3_nameT[0],  "USB_3_NAME", "Name", "");
    IUFillSwitchVector(&USB3SP, USB3S, SWITCH_TOGGLE_COUNT, getDeviceName(),  "USB3",  "USB 3",
                       USB_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&USB3S[ON_SWITCH],  "USB3_ON", "ON", ISS_OFF);
    IUFillSwitch(&USB3S[OFF_SWITCH],  "USB3_OFF", "OFF", ISS_ON);

    IUFillTextVector(&USB4_nameTP, USB4_nameT, 1, getDeviceName(),  "USB_4_NAME",  "USB 4",
                     USB_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&USB4_nameT[0],  "USB_4_NAME", "Name", "");
    IUFillSwitchVector(&USB4SP, USB4S, SWITCH_TOGGLE_COUNT, getDeviceName(),  "USB4",  "USB 4",
                       USB_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&USB4S[ON_SWITCH],  "USB4_ON", "ON", ISS_OFF);
    IUFillSwitch(&USB4S[OFF_SWITCH],  "USB4_OFF", "OFF", ISS_ON);

    IUFillTextVector(&USB5_nameTP, USB5_nameT, 1, getDeviceName(),  "USB_5_NAME",  "USB 5",
                     USB_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&USB5_nameT[0],  "USB_5_NAME", "Name", "");
    IUFillSwitchVector(&USB5SP, USB5S, SWITCH_TOGGLE_COUNT, getDeviceName(),  "USB5",  "USB 5",
                       USB_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&USB5S[ON_SWITCH],  "USB5_ON", "ON", ISS_OFF);
    IUFillSwitch(&USB5S[OFF_SWITCH],  "USB5_OFF", "OFF", ISS_ON);

    IUFillTextVector(&USB6_nameTP, USB6_nameT, 1, getDeviceName(),  "USB_6_NAME",  "USB 6",
                     USB_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&USB6_nameT[0],  "USB_6_NAME", "Name", "");
    IUFillSwitchVector(&USB6SP, USB6S, SWITCH_TOGGLE_COUNT, getDeviceName(),  "USB6",  "USB 6",
                       USB_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&USB6S[ON_SWITCH],  "USB6_ON", "ON", ISS_OFF);
    IUFillSwitch(&USB6S[OFF_SWITCH],  "USB6_OFF", "OFF", ISS_ON);

    IUFillTextVector(&USB7_nameTP, USB7_nameT, 1, getDeviceName(),  "USB_7_NAME",  "USB 7",
                     USB_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&USB7_nameT[0],  "USB_7_NAME", "Name", "");
    IUFillSwitchVector(&USB7SP, USB7S, SWITCH_TOGGLE_COUNT, getDeviceName(),  "USB7",  "USB 7",
                       USB_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&USB7S[ON_SWITCH],  "USB7_ON", "ON", ISS_OFF);
    IUFillSwitch(&USB7S[OFF_SWITCH],  "USB7_OFF", "OFF", ISS_ON);

    IUFillTextVector(&USB8_nameTP, USB8_nameT, 1, getDeviceName(),  "USB_8_NAME",  "USB 8",
                     USB_TAB, IP_RO, 60, IPS_OK);
    IUFillText(&USB8_nameT[0],  "USB_8_NAME", "Name", "");
    IUFillSwitchVector(&USB8SP, USB8S, SWITCH_TOGGLE_COUNT, getDeviceName(),  "USB8",  "USB 8",
                       USB_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);
    IUFillSwitch(&USB8S[ON_SWITCH],  "USB8_ON", "ON", ISS_OFF);
    IUFillSwitch(&USB8S[OFF_SWITCH],  "USB8_OFF", "OFF", ISS_ON);

    // MANUAL_TAB
    //-----------
    // Debug only
    // IUFillTextVector(&Arbitary_CommandTP, Arbitary_CommandT, 1, getDeviceName(), "ARBITARY_COMMAND", "Command",
    //                  MANUAL_TAB, IP_RW, 60, IPS_IDLE);
    // IUFillText(&Arbitary_CommandT[0], "ARBITARY_COMMANDT", "Response:", ":GVP#");
    // Debug only end

    // Connection and handshake registration
    if (osConnection & CONNECTION_SERIAL) {
        serialConnection = new Connection::Serial(this);
        serialConnection->registerHandshake([&]() { return Handshake(); });
        serialConnection->setDefaultBaudRate(Connection::Serial::B_9600);
        LOG_INFO("Non-Network based connection, detection timeouts set to 0.1 seconds");
        OSTimeoutMicroSeconds = 100000;
        OSTimeoutSeconds = 0;
        registerConnection(serialConnection);
    } else if (osConnection & CONNECTION_TCP) {
        tcpConnection = new Connection::TCP(this);
        tcpConnection->setDefaultHost("192.168.0.1");
        tcpConnection->setDefaultPort(9999);
        tcpConnection->registerHandshake([&]() { return Handshake(); });
        LOG_INFO("Network based connection, detection timeouts set to 2 seconds");
        OSTimeoutMicroSeconds = 0;
        OSTimeoutSeconds = 2;
        registerConnection(tcpConnection);
    }

    if (isConnected()) {
        loadConfig(true);
    }

    return true;
}

/******************************************************************
 * Called from connectionInterface to establish contact with device
 *****************************************************************/
bool OnStep_Aux::Handshake()
{
    if (getActiveConnection() == serialConnection) {
        PortFD = serialConnection->getPortFD();
    } else if (getActiveConnection() == tcpConnection) {
        PortFD = tcpConnection->getPortFD();
    }

    if (PortFD < 0) {
        LOG_ERROR("Failed to get valid file descriptor from connection)");
        return false;
    }

    bool handshake_status = false;
    char response[RB_MAX_LEN] = {0};
    handshake_status = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_handshake);
    if (strcmp(response, "On-Step") == 0) {
        LOG_INFO("OnStep Aux handshake established");
        handshake_status = true;
        GetCapabilites();
    } else {
        LOGF_INFO("OnStep Aux handshake error, reponse was: %s", response);
    }

    return handshake_status;
}

/************************************************************
 * Called from Handshake
 * Query connected device for capabilities and enable/disable
 * Properties and Interfaces based on the responses
 ************************************************************/
void OnStep_Aux::GetCapabilites()
{
    // Get OnStepX version
    uint16_t capabilities = getDriverInterface();
    // Get firmware version
    char response[RB_MAX_LEN] = {0};
    int error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_firmware);
    if (error_or_fail > 1) {
        IUSaveText(&VersionT[0], response);
        IDSetText(&VersionTP, nullptr);
        LOGF_DEBUG("OnStepX version: %s", response);
    } else {
        LOG_ERROR("OnStepX version not retrieved");
    }

    // Discover focuser
    memset(response, 0, RB_MAX_LEN);
    int intResponse = 0;
    error_or_fail = getCommandIntResponse(PortFD, &intResponse, response, OS_get_defined_focusers);
    if (error_or_fail > 0 && intResponse > 0) {
        hasFocuser = true;
        FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT);
        LOG_DEBUG("Focuser found, enabling Focuser Tab");
    } else {
        LOG_DEBUG("Focuser not found, disabling Focuser Tab");
        capabilities &= ~FOCUSER_INTERFACE;
    }

    // Discover rotator
    memset(response, 0, RB_MAX_LEN);
    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_defined_rotator);
    if (error_or_fail > 1 ) {
        if (response[0] == 'D' || response[0] == 'R') {
            LOG_DEBUG("Rotator found, enabling Rotator Tab");
            hasRotator = true;
            RI::SetCapability(ROTATOR_CAN_ABORT | ROTATOR_CAN_HOME | ROTATOR_HAS_BACKLASH);
        }
        if (response[0] == 'D') {
            defineProperty(&OSRotatorDerotateSP);
        }
    } else {
        LOG_DEBUG("Rotator not found, disabling Rotator Tab");
        capabilities &= ~ ROTATOR_INTERFACE;
    }

    // Discover weather sensors
    for (int measurement = 0; measurement < WEATHER_MEASUREMENTS_COUNT; measurement ++) {
        char command[CMD_MAX_LEN];
        switch(measurement) {
            case WEATHER_TEMPERATURE:
                indi_strlcpy(command, OS_get_temperature, sizeof(command));
                break;
            case WEATHER_PRESSURE:
                indi_strlcpy(command, OS_get_pressure, sizeof(command));
                break;
            case WEATHER_HUMIDITY:
                indi_strlcpy(command, OS_get_humidity, sizeof(command));
                break;
            case WEATHER_DEW_POINT:
                indi_strlcpy(command, OS_get_dew_point, sizeof(command));
                break;
            default:
                break;
        }

        memset(response, 0, RB_MAX_LEN);
        error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, command);
        if (error_or_fail > 1 &&
            strcmp(response, "N/A") != 0 &&
            strcmp(response, "nan") != 0 &&
            strcmp(response, "0") != 0) {
            weather_enabled[measurement] = 1;
            hasWeather = true;
        } else {
            weather_enabled[measurement] = 0;
        }
    }

    // Available weather measurements are now defined as = 1, unavailable as = 0
    // so we can sum these to check if any are defined, if not then keep tab disabled
    int weatherDisabled = 0;
    for (int wmeasure = 1; wmeasure < WEATHER_MEASUREMENTS_COUNT; wmeasure ++) {
        weatherDisabled += weather_enabled[wmeasure];
    }
    if (weatherDisabled > 0) {
        weather_tab_enabled = true;
        LOG_DEBUG("Weather sensor(s) found, enabling Weather Tab");
    } else {
        LOG_DEBUG("Weather sensor not found, disabling Weather Tab");
        capabilities &= ~WEATHER_INTERFACE;
    }

    // Discover features
    memset(response, 0, RB_MAX_LEN);
    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_defined_features);
    if (error_or_fail > 0) {
        int value = conversion_error;
        try {
            value = std::stoi(response);
        } catch (const std::invalid_argument&) {
            LOGF_WARN("Invalid response to %s: %s", OS_get_defined_features, response);
        } catch (const std::out_of_range&) {
            LOGF_WARN("Invalid response to %s: %s", OS_get_defined_features, response);
        }
        if (value > 0 ) {
            hasFeature = true;
            LOG_DEBUG("Auxiliary Feature(s) found, enabling Feature Tab(s)");
            std::string features = response;
            for (uint digit = 0; digit < max_features; digit++) {
                features_enabled[digit] = features[digit] - '0';
            }
            // Get feature names and types
            for (int feature = 0; feature < max_features; feature++) {
                memset(response, 0, RB_MAX_LEN);
                char cmd[CMD_MAX_LEN] = {0};
                snprintf(cmd, sizeof(cmd), "%s%d%s", OS_get_feature_definiton_part, (feature + 1), OS_command_terminator);
                error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, cmd);
                if (error_or_fail > 0) {
                    char *split;
                    split = strtok(response, ",");
                    if (strcmp(split, "N/A") != 0) {
                        features_name[feature] = split;
                    }
                    split = strtok(NULL, ",");
                    if (strcmp(split, "N/A") != 0) {
                        if (charToInt(split) != conversion_error) {
                            features_type[feature] = static_cast<feature_types>(charToInt(split));
                        }
                    }
                    switch(feature) {
                    case 0:
                        if (features_type[feature] == SWITCH || features_type[feature] == MOMENTARY_SWITCH || features_type[feature] == COVER_SWITCH) {
                            IUSaveText(&Switch1_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch1_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew1_nameT[0], features_name[feature].c_str());
                            IDSetText(&Dew1TP, nullptr);
                        } else if (features_type[feature] == INTERVALOMETER) {
                            IUSaveText(&Inter1_nameT[0], features_name[feature].c_str());
                            IDSetText(&Inter1TP, nullptr);
                        }
                        break;
                    case 1:
                        if (features_type[feature] == SWITCH || features_type[feature] == MOMENTARY_SWITCH || features_type[feature] == COVER_SWITCH) {
                            IUSaveText(&Switch2_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch2_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew2_nameT[0], features_name[feature].c_str());
                            IDSetText(&Dew2TP, nullptr);
                        } else if (features_type[feature] == INTERVALOMETER) {
                            IUSaveText(&Inter2_nameT[0], features_name[feature].c_str());
                            IDSetText(&Inter2TP, nullptr);
                        }
                        break;
                    case 2:
                        if (features_type[feature] == SWITCH || features_type[feature] == MOMENTARY_SWITCH || features_type[feature] == COVER_SWITCH) {
                            IUSaveText(&Switch3_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch3_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew3_nameT[0], features_name[feature].c_str());
                            IDSetText(&Dew3TP, nullptr);
                        } else if (features_type[feature] == INTERVALOMETER) {
                            IUSaveText(&Inter3_nameT[0], features_name[feature].c_str());
                            IDSetText(&Inter3TP, nullptr);
                        }
                        break;
                    case 3:
                        if (features_type[feature] == SWITCH || features_type[feature] == MOMENTARY_SWITCH || features_type[feature] == COVER_SWITCH) {
                            IUSaveText(&Switch4_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch4_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew4_nameT[0], features_name[feature].c_str());
                            IDSetText(&Dew4TP, nullptr);
                        } else if (features_type[feature] == INTERVALOMETER) {
                            IUSaveText(&Inter4_nameT[0], features_name[feature].c_str());
                            IDSetText(&Inter4TP, nullptr);
                        }
                        break;
                    case 4:
                        if (features_type[feature] == SWITCH || features_type[feature] == MOMENTARY_SWITCH || features_type[feature] == COVER_SWITCH) {
                            IUSaveText(&Switch5_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch5_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew5_nameT[0], features_name[feature].c_str());
                            IDSetText(&Dew5TP, nullptr);
                        } else if (features_type[feature] == INTERVALOMETER) {
                            IUSaveText(&Inter5_nameT[0], features_name[feature].c_str());
                            IDSetText(&Inter5TP, nullptr);
                        }
                        break;
                    case 5:
                        if (features_type[feature] == SWITCH || features_type[feature] == MOMENTARY_SWITCH || features_type[feature] == COVER_SWITCH) {
                            IUSaveText(&Switch6_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch6_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew6_nameT[0], features_name[feature].c_str());
                            IDSetText(&Dew6TP, nullptr);
                        } else if (features_type[feature] == INTERVALOMETER) {
                            IUSaveText(&Inter6_nameT[0], features_name[feature].c_str());
                            IDSetText(&Inter6TP, nullptr);
                        }
                        break;
                    case 6:
                        if (features_type[feature] == SWITCH || features_type[feature] == MOMENTARY_SWITCH || features_type[feature] == COVER_SWITCH) {
                            IUSaveText(&Switch7_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch7_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew7_nameT[0], features_name[feature].c_str());
                            IDSetText(&Dew7TP, nullptr);
                        } else if (features_type[feature] == INTERVALOMETER) {
                            IUSaveText(&Inter7_nameT[0], features_name[feature].c_str());
                            IDSetText(&Inter7TP, nullptr);
                        }
                        break;
                    case 7:
                        if (features_type[feature] == SWITCH || features_type[feature] == MOMENTARY_SWITCH || features_type[feature] == COVER_SWITCH) {
                            IUSaveText(&Switch8_nameT[0], features_name[feature].c_str());
                            IDSetText(&Switch8_nameTP, nullptr);
                        } else if (features_type[feature] == DEW_HEATER) {
                            IUSaveText(&Dew8_nameT[0], features_name[feature].c_str());
                            IDSetText(&Dew8TP, nullptr);
                        } else if (features_type[feature] == INTERVALOMETER) {
                            IUSaveText(&Inter8_nameT[0], features_name[feature].c_str());
                            IDSetText(&Inter8TP, nullptr);
                        }
                        break;
                    default:
                        break;
                    }
                }
            }
            if (findEnumInArray(features_type, SWITCH) ||
                findEnumInArray(features_type, MOMENTARY_SWITCH) ||
                findEnumInArray(features_type, COVER_SWITCH)) {
                hasSwitch = true;
            }
            if (findEnumInArray(features_type, DEW_HEATER)) {
                hasDew = true;
            }
            if (findEnumInArray(features_type, INTERVALOMETER)) {
                hasIntervalometer = true;
            }
            if (findEnumInArray(features_type, ANALOG_OUTPUT)) {
                hasOutput = true;
            }
        } else {
            LOG_DEBUG("Auxiliary Feature not found, disabling Feature Tab(s)");
            capabilities &= ~AUX_INTERFACE;
        }
    } else {
        LOG_DEBUG("Auxiliary Feature not found, disabling Feature Tab(s)");
        capabilities &= ~AUX_INTERFACE;
    }

    // Discover USB ports
    // Only for OnStepX-plugin USB_switcher - otherwise use Features type SWITCH
    memset(response, 0, RB_MAX_LEN);
    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_defined_USBports);
    if (error_or_fail > 0) {
        int value = conversion_error;
        try {
            value = std::stoi(response);
        } catch (const std::invalid_argument&) {
            LOGF_WARN("Invalid response to %s: %s", OS_get_defined_USBports, response);
        } catch (const std::out_of_range&) {
            LOGF_WARN("Invalid response to %s: %s", OS_get_defined_USBports, response);
        }
        if (value > 0 ) {
            hasUSB = true;
            LOG_DEBUG("USB Port(s) found, enabling USB Tab");
            std::string USBports = response;
            for (uint digit = 0; digit < max_USBports; digit++) {
                USBports_enabled[digit] = USBports[digit] - '0';
                if (USBports_enabled[digit]) {
                    USBportCount++;
                }
            }
            // Get USB port names
            for (int USBport = 0; USBport < max_USBports; USBport++) {
                memset(response, 0, RB_MAX_LEN);
                char cmd[CMD_MAX_LEN] = {0};
                if (USBports_enabled[USBport]) {
                    snprintf(cmd, sizeof(cmd), "%s%d%s", OS_get_USBport_name_part, (USBport + 1), OS_command_terminator);
                    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, cmd);
                    if (error_or_fail > 0) {
                        if (strcmp(response, "N/A") != 0) {
                            USBports_name[USBport] = response;
                        }
                    }
                    switch(USBport) {
                    case 0:
                        IUSaveText(&USB1_nameT[0], USBports_name[USBport].c_str());
                        IDSetText(&USB1_nameTP, nullptr);
                        break;
                    case 1:
                        IUSaveText(&USB2_nameT[0], USBports_name[USBport].c_str());
                        IDSetText(&USB2_nameTP, nullptr);
                        break;
                    case 2:
                        IUSaveText(&USB3_nameT[0], USBports_name[USBport].c_str());
                        IDSetText(&USB3_nameTP, nullptr);
                        break;
                    case 3:
                        IUSaveText(&USB4_nameT[0], USBports_name[USBport].c_str());
                        IDSetText(&USB4_nameTP, nullptr);
                        break;
                    case 4:
                        IUSaveText(&USB5_nameT[0], USBports_name[USBport].c_str());
                        IDSetText(&USB5_nameTP, nullptr);
                        break;
                    case 5:
                        IUSaveText(&USB6_nameT[0], USBports_name[USBport].c_str());
                        IDSetText(&USB6_nameTP, nullptr);
                        break;
                    case 6:
                        IUSaveText(&USB7_nameT[0], USBports_name[USBport].c_str());
                        IDSetText(&USB7_nameTP, nullptr);
                        break;
                    case 7:
                        IUSaveText(&USB8_nameT[0], USBports_name[USBport].c_str());
                        IDSetText(&USB8_nameTP, nullptr);
                        break;
                    default:
                        break;
                    }
                }
            }
        } else {
               LOG_WARN("No USBs found, disabling USB Tab");
            capabilities &= ~POWER_INTERFACE;
        }
    } else {
        LOG_WARN("No USBs found, disabling USB Tab");
        capabilities &= ~POWER_INTERFACE;
    }

    setDriverInterface(capabilities);
    syncDriverInfo();
}

bool OnStep_Aux::updateProperties()
{
    DefaultDevice::updateProperties();
    if (isConnected()) {
        SetTimer(getCurrentPollingPeriod());
        loadConfig(true);

        defineProperty(&VersionTP);

        if (hasFocuser) {
            defineProperty(&OSFocus1InitializeSP);
            // Focus T° Compensation
            defineProperty(&FocusTemperatureNP);
            defineProperty(&TFCCompensationSP);
            defineProperty(&TFCCoefficientNP);
            defineProperty(&TFCDeadbandNP);
            FI::updateProperties();
        }

        if (hasRotator) {
            defineProperty(&OSRotatorRateSP);
            defineProperty(&OSRotatorDerotateSP);
        }

        if (hasWeather) {
            WI::updateProperties();
        }

        if (hasFeature) {
            for (int OSfeature = 0; OSfeature < max_features; OSfeature++) {
                if (features_enabled[OSfeature] == 1) {
                    if (features_type[OSfeature] == SWITCH ||
                        features_type[OSfeature] == MOMENTARY_SWITCH ||
                        features_type[OSfeature] == COVER_SWITCH) {
                        switch (OSfeature) {
                        case 0:
                            defineProperty(&Switch1_nameTP);
                            defineProperty(&Switch1SP);
                            break;
                        case 1:
                            defineProperty(&Switch2_nameTP);
                            defineProperty(&Switch2SP);
                            break;
                        case 2:
                            defineProperty(&Switch3_nameTP);
                            defineProperty(&Switch3SP);
                            break;
                        case 3:
                            defineProperty(&Switch4_nameTP);
                            defineProperty(&Switch4SP);
                            break;
                        case 4:
                            defineProperty(&Switch5_nameTP);
                            defineProperty(&Switch5SP);
                            break;
                        case 5:
                            defineProperty(&Switch6_nameTP);
                            defineProperty(&Switch6SP);
                            break;
                        case 6:
                            defineProperty(&Switch7_nameTP);
                            defineProperty(&Switch7SP);
                            break;
                        case 7:
                            defineProperty(&Switch8_nameTP);
                            defineProperty(&Switch8SP);
                            break;
                        default:
                            break;
                        }
                    } else if (features_type[OSfeature] == DEW_HEATER) {
                        switch (OSfeature) {
                        case 0:
                            defineProperty(&Dew1TP);
                            defineProperty(&Dew1SP);
                            defineProperty(&Dew1NP);
                            defineProperty(&Dew1deltaTP);
                            break;
                        case 1:
                            defineProperty(&Dew2TP);
                            defineProperty(&Dew2SP);
                            defineProperty(&Dew2NP);
                            defineProperty(&Dew2deltaTP);
                            break;
                        case 2:
                            defineProperty(&Dew3TP);
                            defineProperty(&Dew3SP);
                            defineProperty(&Dew3NP);
                            defineProperty(&Dew3deltaTP);
                            break;
                        case 3:
                            defineProperty(&Dew4TP);
                            defineProperty(&Dew4SP);
                            defineProperty(&Dew4NP);
                            defineProperty(&Dew4deltaTP);
                            break;
                        case 4:
                            defineProperty(&Dew5TP);
                            defineProperty(&Dew5SP);
                            defineProperty(&Dew5NP);
                            defineProperty(&Dew5deltaTP);
                            break;
                        case 5:
                            defineProperty(&Dew6TP);
                            defineProperty(&Dew6SP);
                            defineProperty(&Dew6NP);
                            defineProperty(&Dew6deltaTP);
                            break;
                        case 6:
                            defineProperty(&Dew7TP);
                            defineProperty(&Dew7SP);
                            defineProperty(&Dew7NP);
                            defineProperty(&Dew7deltaTP);
                            break;
                        case 7:
                            defineProperty(&Dew8TP);
                            defineProperty(&Dew8SP);
                            defineProperty(&Dew8NP);
                            defineProperty(&Dew8deltaTP);
                            break;
                        default:
                            break;
                        }
                    } else if (features_type[OSfeature] == INTERVALOMETER) {
                        switch (OSfeature) {
                        case 0:
                            defineProperty(&Inter1TP);
                            defineProperty(&Inter1SP);
                            defineProperty(&Inter1NP);
                            defineProperty(&Inter1doneTP);
                            break;
                        case 1:
                            defineProperty(&Inter2TP);
                            defineProperty(&Inter2SP);
                            defineProperty(&Inter2NP);
                            defineProperty(&Inter2doneTP);
                            break;
                        case 2:
                            defineProperty(&Inter3TP);
                            defineProperty(&Inter3SP);
                            defineProperty(&Inter3NP);
                            defineProperty(&Inter3doneTP);
                            break;
                        case 3:
                            defineProperty(&Inter4TP);
                            defineProperty(&Inter4SP);
                            defineProperty(&Inter4NP);
                            defineProperty(&Inter4doneTP);
                            break;
                        case 4:
                            defineProperty(&Inter5TP);
                            defineProperty(&Inter5SP);
                            defineProperty(&Inter5NP);
                            defineProperty(&Inter5doneTP);
                            break;
                        case 5:
                            defineProperty(&Inter6TP);
                            defineProperty(&Inter6SP);
                            defineProperty(&Inter6NP);
                            defineProperty(&Inter6doneTP);
                            break;
                        case 6:
                            defineProperty(&Inter7TP);
                            defineProperty(&Inter7SP);
                            defineProperty(&Inter7NP);
                            defineProperty(&Inter7doneTP);
                            break;
                        case 7:
                            defineProperty(&Inter8TP);
                            defineProperty(&Inter8SP);
                            defineProperty(&Inter8NP);
                            defineProperty(&Inter8doneTP);
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
        }

        if (hasUSB) {
            defineProperty(&USBallSP);
            for (int USBport = 0; USBport < max_USBports; USBport++) {
                if (USBports_enabled[USBport] == 1) {
                    switch (USBport) {
                    case 0:
                        defineProperty(&USB1_nameTP);
                        defineProperty(&USB1SP);
                        break;
                    case 1:
                        defineProperty(&USB2_nameTP);
                        defineProperty(&USB2SP);
                        break;
                    case 2:
                        defineProperty(&USB3_nameTP);
                        defineProperty(&USB3SP);
                        break;
                    case 3:
                        defineProperty(&USB4_nameTP);
                        defineProperty(&USB4SP);
                        break;
                    case 4:
                        defineProperty(&USB5_nameTP);
                        defineProperty(&USB5SP);
                        break;
                    case 5:
                        defineProperty(&USB6_nameTP);
                        defineProperty(&USB6SP);
                        break;
                    case 6:
                        defineProperty(&USB7_nameTP);
                        defineProperty(&USB7SP);
                        break;
                    case 7:
                        defineProperty(&USB8_nameTP);
                        defineProperty(&USB8SP);
                        break;
                    default:
                        break;
                    }
                }
            }
        }

        // Debug only
        // defineProperty(&Arbitary_CommandTP);
        // Debug only end

    } else {
        deleteProperty(VersionTP.name);

        deleteProperty(OSFocus1InitializeSP.name);
        deleteProperty(FocusTemperatureNP.name);
        deleteProperty(TFCCompensationSP.name);
        deleteProperty(TFCCoefficientNP.name);
        deleteProperty(TFCDeadbandNP.name);
        deleteProperty(OSRotatorRateSP.name);
        deleteProperty(OSRotatorDerotateSP.name);

        deleteProperty(Switch1SP.name);
        deleteProperty(Switch1_nameTP.name);
        deleteProperty(Switch2SP.name);
        deleteProperty(Switch2_nameTP.name);
        deleteProperty(Switch3SP.name);
        deleteProperty(Switch3_nameTP.name);
        deleteProperty(Switch4SP.name);
        deleteProperty(Switch4_nameTP.name);
        deleteProperty(Switch5SP.name);
        deleteProperty(Switch5_nameTP.name);
        deleteProperty(Switch6SP.name);
        deleteProperty(Switch6_nameTP.name);
        deleteProperty(Switch7SP.name);
        deleteProperty(Switch7_nameTP.name);
        deleteProperty(Switch8SP.name);
        deleteProperty(Switch8_nameTP.name);

        deleteProperty(Dew1TP.name);
        deleteProperty(Dew1SP.name);
        deleteProperty(Dew1NP.name);
        deleteProperty(Dew1deltaTP.name);
        deleteProperty(Dew2TP.name);
        deleteProperty(Dew2SP.name);
        deleteProperty(Dew2NP.name);
        deleteProperty(Dew2deltaTP.name);
        deleteProperty(Dew3TP.name);
        deleteProperty(Dew3SP.name);
        deleteProperty(Dew3NP.name);
        deleteProperty(Dew3deltaTP.name);
        deleteProperty(Dew4TP.name);
        deleteProperty(Dew4SP.name);
        deleteProperty(Dew4NP.name);
        deleteProperty(Dew4deltaTP.name);
        deleteProperty(Dew5TP.name);
        deleteProperty(Dew5SP.name);
        deleteProperty(Dew5NP.name);
        deleteProperty(Dew5deltaTP.name);
        deleteProperty(Dew6TP.name);
        deleteProperty(Dew6SP.name);
        deleteProperty(Dew6NP.name);
        deleteProperty(Dew6deltaTP.name);
        deleteProperty(Dew7TP.name);
        deleteProperty(Dew7SP.name);
        deleteProperty(Dew7NP.name);
        deleteProperty(Dew7deltaTP.name);
        deleteProperty(Dew8TP.name);
        deleteProperty(Dew8SP.name);
        deleteProperty(Dew8NP.name);
        deleteProperty(Dew8deltaTP.name);

        deleteProperty(Inter1TP.name);
        deleteProperty(Inter1SP.name);
        deleteProperty(Inter1NP.name);
        deleteProperty(Inter1doneTP.name);
        deleteProperty(Inter2TP.name);
        deleteProperty(Inter2SP.name);
        deleteProperty(Inter2NP.name);
        deleteProperty(Inter2doneTP.name);
        deleteProperty(Inter3TP.name);
        deleteProperty(Inter3SP.name);
        deleteProperty(Inter3NP.name);
        deleteProperty(Inter3doneTP.name);
        deleteProperty(Inter4TP.name);
        deleteProperty(Inter4SP.name);
        deleteProperty(Inter4NP.name);
        deleteProperty(Inter4doneTP.name);
        deleteProperty(Inter5TP.name);
        deleteProperty(Inter5SP.name);
        deleteProperty(Inter5NP.name);
        deleteProperty(Inter5doneTP.name);
        deleteProperty(Inter6TP.name);
        deleteProperty(Inter6SP.name);
        deleteProperty(Inter6NP.name);
        deleteProperty(Inter6doneTP.name);
        deleteProperty(Inter7TP.name);
        deleteProperty(Inter7SP.name);
        deleteProperty(Inter7NP.name);
        deleteProperty(Inter7doneTP.name);
        deleteProperty(Inter8TP.name);
        deleteProperty(Inter8SP.name);
        deleteProperty(Inter8NP.name);
        deleteProperty(Inter8doneTP.name);

        deleteProperty(USBallSP.name);
        deleteProperty(USB1SP.name);
        deleteProperty(USB1_nameTP.name);
        deleteProperty(USB2SP.name);
        deleteProperty(USB2_nameTP.name);
        deleteProperty(USB3SP.name);
        deleteProperty(USB3_nameTP.name);
        deleteProperty(USB4SP.name);
        deleteProperty(USB4_nameTP.name);
        deleteProperty(USB5SP.name);
        deleteProperty(USB5_nameTP.name);
        deleteProperty(USB6SP.name);
        deleteProperty(USB6_nameTP.name);
        deleteProperty(USB7SP.name);
        deleteProperty(USB7_nameTP.name);
        deleteProperty(USB8SP.name);
        deleteProperty(USB8_nameTP.name);

        // Debug only
        // deleteProperty(Arbitary_CommandTP.name);
        // Debug only end
        return false;
    }
    return true;
}

bool OnStep_Aux::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {

        LOGF_DEBUG("Got an IsNewSwitch for: %s", name);
        char cmd[CMD_MAX_LEN];

        // Switch devices
        //---------------
        if (strcmp(Switch1SP.name, name) == 0) {
            IUUpdateSwitch(&Switch1SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "DEVICE1_ON") == 0) {
                    sprintf(cmd, "%s1,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Switch1SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "DEVICE1_OFF") == 0) {
                    sprintf(cmd, "%s1,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Switch1SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Switch1SP, nullptr);
            return false;
        } else if (strcmp(Switch2SP.name, name) == 0) {
            IUUpdateSwitch(&Switch2SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "DEVICE2_ON") == 0) {
                    sprintf(cmd, "%s2,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Switch2SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "DEVICE2_OFF") == 0) {
                    sprintf(cmd, "%s2,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Switch2SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Switch2SP, nullptr);
            return false;
        } else if (strcmp(Switch3SP.name, name) == 0) {
            IUUpdateSwitch(&Switch3SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "DEVICE3_ON") == 0) {
                    sprintf(cmd, "%s3,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Switch3SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "DEVICE3_OFF") == 0) {
                    sprintf(cmd, "%s3,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Switch3SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Switch3SP, nullptr);
            return false;
        } else if (strcmp(Switch4SP.name, name) == 0) {
            IUUpdateSwitch(&Switch4SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "DEVICE4_ON") == 0) {
                    sprintf(cmd, "%s4,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Switch4SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "DEVICE4_OFF") == 0) {
                    sprintf(cmd, "%s4,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Switch4SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Switch4SP, nullptr);
            return false;
        } else if (strcmp(Switch5SP.name, name) == 0) {
            IUUpdateSwitch(&Switch5SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "DEVICE5_ON") == 0) {
                    sprintf(cmd, "%s5,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Switch5SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "DEVICE5_OFF") == 0) {
                    sprintf(cmd, "%s5,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Switch5SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Switch5SP, nullptr);
            return false;
        } else if (strcmp(Switch6SP.name, name) == 0) {
            IUUpdateSwitch(&Switch6SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "DEVICE6_ON") == 0) {
                    sprintf(cmd, "%s6,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Switch6SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "DEVICE6_OFF") == 0) {
                    sprintf(cmd, "%s6,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Switch6SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Switch6SP, nullptr);
            return false;
        } else if (strcmp(Switch7SP.name, name) == 0) {
            IUUpdateSwitch(&Switch7SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "DEVICE7_ON") == 0) {
                    sprintf(cmd, "%s7,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Switch7SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "DEVICE7_OFF") == 0) {
                    sprintf(cmd, "%s7,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Switch7SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Switch7SP, nullptr);
            return false;
        } else if (strcmp(Switch8SP.name, name) == 0) {
            IUUpdateSwitch(&Switch8SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "DEVICE8_ON") == 0) {
                    sprintf(cmd, "%s8,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Switch8SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "DEVICE8_OFF") == 0) {
                    sprintf(cmd, "%s8,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Switch8SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Switch8SP, nullptr);
            return false;
        }

        // Dew heaters
        //------------
        if (strcmp(Dew1SP.name, name) == 0) {
            IUUpdateSwitch(&Dew1SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s1,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Dew1SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s1,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Dew1SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Dew1SP, nullptr);
            return false;
        } else if (strcmp(Dew2SP.name, name) == 0) {
            IUUpdateSwitch(&Dew2SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s2,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Dew2SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s2,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Dew2SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Dew2SP, nullptr);
            return false;
        } else if (strcmp(Dew3SP.name, name) == 0) {
            IUUpdateSwitch(&Dew3SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s3,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Dew3SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s3,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Dew3SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Dew4SP.name, name) == 0) {
            IUUpdateSwitch(&Dew4SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s4,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Dew4SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s4,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Dew4SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Dew5SP.name, name) == 0) {
            IUUpdateSwitch(&Dew5SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s5,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Dew5SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s5,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Dew5SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Dew6SP.name, name) == 0) {
            IUUpdateSwitch(&Dew6SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s6,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Dew6SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s6,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Dew6SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Dew7SP.name, name) == 0) {
            IUUpdateSwitch(&Dew7SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s7,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Dew7SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s7,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Dew7SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Dew8SP.name, name) == 0) {
            IUUpdateSwitch(&Dew8SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s8,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Dew8SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s8,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Dew8SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        }

        // Intervalometers
        //----------------
        if (strcmp(Inter1SP.name, name) == 0) {
            IUUpdateSwitch(&Inter1SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s1,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Inter1SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s1,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Inter1SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Inter1SP, nullptr);
            return false;
        } else if (strcmp(Inter2SP.name, name) == 0) {
            IUUpdateSwitch(&Inter2SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s2,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Inter2SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s2,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Inter2SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&Inter2SP, nullptr);
            return false;
        } else if (strcmp(Inter3SP.name, name) == 0) {
            IUUpdateSwitch(&Inter3SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s3,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Inter3SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s3,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Inter3SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Inter4SP.name, name) == 0) {
            IUUpdateSwitch(&Inter4SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s4,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Inter4SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s4,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Inter4SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Inter5SP.name, name) == 0) {
            IUUpdateSwitch(&Inter5SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s5,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Inter5SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s5,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Inter5SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Inter6SP.name, name) == 0) {
            IUUpdateSwitch(&Inter6SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s6,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Inter6SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s6,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Inter6SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Inter7SP.name, name) == 0) {
            IUUpdateSwitch(&Inter7SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s7,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Inter7SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s7,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Inter7SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        } else if (strcmp(Inter8SP.name, name) == 0) {
            IUUpdateSwitch(&Inter8SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "ENABLE_ON") == 0) {
                    sprintf(cmd, "%s8,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&Inter8SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "ENABLE_OFF") == 0) {
                    sprintf(cmd, "%s8,%s%d%s", OS_set_feature_part, OS_set_feature_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&Inter8SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
        }

        // USB Ports
        //----------
        if (strcmp(USB1SP.name, name) == 0) {
            IUUpdateSwitch(&USB1SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "USB1_ON") == 0) {
                    sprintf(cmd, "%s1,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&USB1SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "USB1_OFF") == 0) {
                    sprintf(cmd, "%s1,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&USB1SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&USB1SP, nullptr);
            return false;
        } else if (strcmp(USB2SP.name, name) == 0) {
            IUUpdateSwitch(&USB2SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "USB2_ON") == 0) {
                    sprintf(cmd, "%s2,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&USB2SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "USB2_OFF") == 0) {
                    sprintf(cmd, "%s2,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&USB2SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&USB2SP, nullptr);
            return false;
        } else if (strcmp(USB3SP.name, name) == 0) {
            IUUpdateSwitch(&USB3SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "USB3_ON") == 0) {
                    sprintf(cmd, "%s3,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&USB3SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "USB3_OFF") == 0) {
                    sprintf(cmd, "%s3,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&USB3SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&USB3SP, nullptr);
            return false;
        } else if (strcmp(USB4SP.name, name) == 0) {
            IUUpdateSwitch(&USB4SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "USB4_ON") == 0) {
                    sprintf(cmd, "%s4,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&USB4SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "USB4_OFF") == 0) {
                    sprintf(cmd, "%s4,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&USB4SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&USB4SP, nullptr);
            return false;
        } else if (strcmp(USB5SP.name, name) == 0) {
            IUUpdateSwitch(&USB5SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "USB5_ON") == 0) {
                    sprintf(cmd, "%s5,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&USB5SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "USB5_OFF") == 0) {
                    sprintf(cmd, "%s5,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&USB5SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&USB5SP, nullptr);
            return false;
        } else if (strcmp(USB6SP.name, name) == 0) {
            IUUpdateSwitch(&USB6SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "USB6_ON") == 0) {
                    sprintf(cmd, "%s6,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&USB6SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "USB6_OFF") == 0) {
                    sprintf(cmd, "%s6,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&USB6SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&USB6SP, nullptr);
            return false;
        } else if (strcmp(USB7SP.name, name) == 0) {
            IUUpdateSwitch(&USB7SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "USB7_ON") == 0) {
                    sprintf(cmd, "%s7,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&USB7SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "USB7_OFF") == 0) {
                    sprintf(cmd, "%s7,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&USB7SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&USB7SP, nullptr);
            return false;
        } else if (strcmp(USB8SP.name, name) == 0) {
            IUUpdateSwitch(&USB8SP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "USB8_ON") == 0) {
                    sprintf(cmd, "%s8,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&USB8SP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "USB8_OFF") == 0) {
                    sprintf(cmd, "%s8,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&USB8SP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&USB8SP, nullptr);
            return false;
        }  else if (strcmp(USBallSP.name, name) == 0) {
            IUUpdateSwitch(&USBallSP, states, names, n);
            for (int i = 0; i < n; i++) {
                if (strcmp(names[i], "USBALL_ON") == 0) {
                    sprintf(cmd, "%s0,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_ENABLE, OS_command_terminator);
                    IDSetSwitch(&USBallSP, nullptr);
                    return sendOSCommand(cmd);
                } else if (strcmp(names[i], "USBALL_OFF") == 0) {
                    sprintf(cmd, "%s0,%s%d%s", OS_set_USBport_part, OS_set_USBport_enabled_part, SWITCH_DISABLE, OS_command_terminator);
                    IDSetSwitch(&USBallSP, nullptr);
                    return sendOSCommand(cmd);
                }
            }
            IDSetSwitch(&USBallSP, nullptr);
            return false;
        }

        // Process Focus-related switches via FocusInterface
        if (strstr(name, "FOCUS"))
            return FI::processSwitch(dev, name, states, names, n);

        // Process Rotator-related switches via RotatorInterface
        if (strstr(name, "ROTATOR"))
            return RI::processSwitch(dev, name, states, names, n);

        return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
    } else {
        return false;
    }
}

bool OnStep_Aux::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (!dev || strcmp(dev, getDeviceName()))
        return false;

    // Focuser
    //--------
    if (!strcmp(name, TFCCoefficientNP.name)) {
        // :FC[sn.n]# Set focuser temperature compensation coefficient in µ/°C
        if (abs(values[0]) < 1000) {    //Range is -999.999 .. + 999.999
            char cmd[CMD_MAX_LEN] = {0};
            snprintf(cmd, 15, "%s%+3.5f%s", OS_get_focuser_temp_comp_coef, values[0], OS_command_terminator);
            sendOSCommandBlind(cmd);
            TFCCoefficientNP.s = IPS_OK;
            IDSetNumber(&TFCCoefficientNP, "TFC Coefficient set to %+3.5f", values[0]);
        } else {
            TFCCoefficientNP.s = IPS_ALERT;
            IDSetNumber(&TFCCoefficientNP, "Setting TFC Coefficient Failed");
        }
        return true;
    }

    if (!strcmp(name, TFCDeadbandNP.name))
    {
        // :FD[n]#    Set focuser temperature compensation deadband amount (in steps or microns)
        if ((values[0] >= 1) && (values[0] <= 32768)) {  //Range is 1 .. 32767
            char cmd[CMD_MAX_LEN] = {0};
            snprintf(cmd, 15, "%s%d%s", OS_get_focuser_deadband, (int)values[0], OS_command_terminator);
            sendOSCommandBlind(cmd);
            TFCDeadbandNP.s = IPS_OK;
            IDSetNumber(&TFCDeadbandNP, "TFC Deadbandset to %d", (int)values[0]);
        } else {
            TFCDeadbandNP.s = IPS_ALERT;
            IDSetNumber(&TFCDeadbandNP, "Setting TFC Deadband Failed");
        }
        return true;
    }

    // Dew Heaters
    //------------
    if (!strcmp(name, Dew1NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "DEW1_ZERO_POINT")) {
                sprintf(cmd, "%s1,%s%f%s", OS_set_feature_part, OS_set_dew_zero_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "DEW1_SPAN")) {
                sprintf(cmd, "%s1,%s%f%s", OS_set_feature_part, OS_set_dew_span_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Dew1NP.s = IPS_OK;
    } else if (!strcmp(name, Dew2NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "DEW2_ZERO_POINT")) {
                sprintf(cmd, "%s2,%s%f%s", OS_set_feature_part, OS_set_dew_zero_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "DEW2_SPAN")) {
                sprintf(cmd, "%s2,%s%f%s", OS_set_feature_part, OS_set_dew_span_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Dew2NP.s = IPS_OK;
    } else if (!strcmp(name, Dew3NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "DEW3_ZERO_POINT")) {
                sprintf(cmd, "%s3,%s%f%s", OS_set_feature_part, OS_set_dew_zero_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "DEW3_SPAN")) {
                sprintf(cmd, "%s3,%s%f%s", OS_set_feature_part, OS_set_dew_span_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Dew3NP.s = IPS_OK;
    } else if (!strcmp(name, Dew4NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "DEW4_ZERO_POINT")) {
                sprintf(cmd, "%s4,%s%f%s", OS_set_feature_part, OS_set_dew_zero_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "DEW4_SPAN")) {
                sprintf(cmd, "%s4,%s%f%s", OS_set_feature_part, OS_set_dew_span_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Dew4NP.s = IPS_OK;
    } else if (!strcmp(name, Dew5NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "DEW5_ZERO_POINT")) {
                sprintf(cmd, "%s5,%s%f%s", OS_set_feature_part, OS_set_dew_zero_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "DEW5_SPAN")) {
                sprintf(cmd, "%s5,%s%f%s", OS_set_feature_part, OS_set_dew_span_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Dew1NP.s = IPS_OK;
    } else if (!strcmp(name, Dew6NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "DEW6_ZERO_POINT")) {
                sprintf(cmd, "%s6,%s%f%s", OS_set_feature_part, OS_set_dew_zero_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "DEW6_SPAN")) {
                sprintf(cmd, "%s6,%s%f%s", OS_set_feature_part, OS_set_dew_span_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Dew6NP.s = IPS_OK;
    } else if (!strcmp(name, Dew7NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "DEW7_ZERO_POINT")) {
                sprintf(cmd, "%s7,%s%f%s", OS_set_feature_part, OS_set_dew_zero_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "DEW7_SPAN")) {
                sprintf(cmd, "%s7,%s%f%s", OS_set_feature_part, OS_set_dew_span_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Dew7NP.s = IPS_OK;
    } else if (!strcmp(name, Dew8NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "DEW8_ZERO_POINT")) {
                sprintf(cmd, "%s8,%s%f%s", OS_set_feature_part, OS_set_dew_zero_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "DEW8_SPAN")) {
                sprintf(cmd, "%s8,%s%f%s", OS_set_feature_part, OS_set_dew_span_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Dew8NP.s = IPS_OK;
    }

    // Intervolometers
    //----------------
    if (!strcmp(name, Inter1NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "INTER1_EXP")) {
                sprintf(cmd, "%s1,%s%f%s", OS_set_feature_part, OS_set_intervalometer_exposure_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER1_DELAY")) {
                sprintf(cmd, "%s1,%s%f%s", OS_set_feature_part, OS_set_intervalometer_delay_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER1_COUNT")) {
                sprintf(cmd, "%s1,%s%f%s", OS_set_feature_part, OS_set_intervalometer_count_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Inter1NP.s = IPS_OK;
    } else if (!strcmp(name, Inter2NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "INTER2_EXP")) {
                sprintf(cmd, "%s2,%s%f%s", OS_set_feature_part, OS_set_intervalometer_exposure_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER2_DELAY")) {
                sprintf(cmd, "%s2,%s%f%s", OS_set_feature_part, OS_set_intervalometer_delay_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER2_COUNT")) {
                sprintf(cmd, "%s2,%s%f%s", OS_set_feature_part, OS_set_intervalometer_count_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Inter2NP.s = IPS_OK;
    } else if (!strcmp(name, Inter3NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "INTER3_EXP")) {
                sprintf(cmd, "%s3,%s%f%s", OS_set_feature_part, OS_set_intervalometer_exposure_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER3_DELAY")) {
                sprintf(cmd, "%s3,%s%f%s", OS_set_feature_part, OS_set_intervalometer_delay_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER3_COUNT")) {
                sprintf(cmd, "%s3,%s%f%s", OS_set_feature_part, OS_set_intervalometer_count_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Inter3NP.s = IPS_OK;
    } else if (!strcmp(name, Inter4NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "INTER4_EXP")) {
                sprintf(cmd, "%s4,%s%f%s", OS_set_feature_part, OS_set_intervalometer_exposure_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER4_DELAY")) {
                sprintf(cmd, "%s4,%s%f%s", OS_set_feature_part, OS_set_intervalometer_delay_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER4_COUNT")) {
                sprintf(cmd, "%s4,%s%f%s", OS_set_feature_part, OS_set_intervalometer_count_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Inter4NP.s = IPS_OK;
    } else if (!strcmp(name, Inter5NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "INTER5_EXP")) {
                sprintf(cmd, "%s5,%s%f%s", OS_set_feature_part, OS_set_intervalometer_exposure_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER5_DELAY")) {
                sprintf(cmd, "%s5,%s%f%s", OS_set_feature_part, OS_set_intervalometer_delay_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER5_COUNT")) {
                sprintf(cmd, "%s5,%s%f%s", OS_set_feature_part, OS_set_intervalometer_count_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Inter5NP.s = IPS_OK;
    } else if (!strcmp(name, Inter6NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "INTER6_EXP")) {
                sprintf(cmd, "%s6,%s%f%s", OS_set_feature_part, OS_set_intervalometer_exposure_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER6_DELAY")) {
                sprintf(cmd, "%s6,%s%f%s", OS_set_feature_part, OS_set_intervalometer_delay_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER6_COUNT")) {
                sprintf(cmd, "%s6,%s%f%s", OS_set_feature_part, OS_set_intervalometer_count_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Inter6NP.s = IPS_OK;
    } else if (!strcmp(name, Inter7NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "INTER7_EXP")) {
                sprintf(cmd, "%s7,%s%f%s", OS_set_feature_part, OS_set_intervalometer_exposure_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER7_DELAY")) {
                sprintf(cmd, "%s7,%s%f%s", OS_set_feature_part, OS_set_intervalometer_delay_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER7_COUNT")) {
                sprintf(cmd, "%s7,%s%f%s", OS_set_feature_part, OS_set_intervalometer_count_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Inter7NP.s = IPS_OK;
    } else if (!strcmp(name, Inter8NP.name)) {
        char cmd[CMD_MAX_LEN] = {0};
        for (int i = 0; i < n; i++) {
            if (!strcmp(names[i], "INTER8_EXP")) {
                sprintf(cmd, "%s8,%s%f%s", OS_set_feature_part, OS_set_intervalometer_exposure_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER8_DELAY")) {
                sprintf(cmd, "%s8,%s%f%s", OS_set_feature_part, OS_set_intervalometer_delay_part, values[0], OS_command_terminator);
            } else if (!strcmp(names[i], "INTER8_COUNT")) {
                sprintf(cmd, "%s8,%s%f%s", OS_set_feature_part, OS_set_intervalometer_count_part, values[0], OS_command_terminator);
            }
        }
        sendOSCommandBlind(cmd);
        Inter8NP.s = IPS_OK;
    }

    // Process Focuser-related switches via FocusInterface
    if (strstr(name, "FOCUS_"))
        return FI::processNumber(dev, name, values, names, n);

    // Process Rotator-related switches via RotatorInterface
    if (strstr(name, "ROTATOR"))
        return RI::processNumber(dev, name, values, names, n);

    // ProcessWeatherr-related switches via WeatherInterface
    if (strstr(name, "WEATHER_")) {
        return WI::processNumber(dev, name, values, names, n);
    }

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}


/*****************************************
 * Client has changed a text field, update
 *****************************************/
bool OnStep_Aux::ISNewText(const char *dev,const char *name,char *texts[],char *names[],int n)
{
    // Debug only - Manual tab, Arbitary command
    // if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {
    //     if (!strcmp(Arbitary_CommandTP.name, name)) {
    //         if (1 == n) {
    //             char response[RB_MAX_LEN] = {0};
    //             int error_or_fail  = getCommandSingleCharErrorOrLongResponse(PortFD, response, texts[0]);
    //             if (error_or_fail > 0) {
    //                 if (strcmp(response, "") == 0) {
    //                     indi_strlcpy(response, "No response", sizeof(response));
    //                 }
    //             } else {
    //                 char error_code[RB_MAX_LEN] = {0};
    //                 if (error_or_fail == TTY_TIME_OUT) {
    //                     indi_strlcpy(response, "No response", sizeof(response));
    //                 } else {
    //                     sprintf(error_code, "Error: %d", error_or_fail);
    //                     indi_strlcpy(response, error_code, sizeof(response));
    //                 }
    //             }
    //             // Replace the user entered string with the OCS response
    //             indi_strlcpy(texts[0], response, RB_MAX_LEN);
    //             IUUpdateText(&Arbitary_CommandTP, texts, names, n);
    //             IDSetText(&Arbitary_CommandTP, nullptr);
    //             return true;
    //         }
    //     }
    //     return false;
    //
    // } else {
    //    return false;
    // }
    // Debug only end

    // Comment out next two lines if uncommenting the above Debug only block
    INDI_UNUSED(dev); INDI_UNUSED(name); INDI_UNUSED(texts); INDI_UNUSED(names); INDI_UNUSED(n);
    return false;
}

/*******************
 * Focuser functions
 ******************/

IPState OnStep_Aux::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    INDI_UNUSED(speed);
    double output;
    char cmd[32];
    output = duration;
    if (dir == FOCUS_INWARD) output = 0 - output;
    snprintf(cmd, sizeof(cmd), "%s%5f%s",OS_move_focuser_rel_part, output, OS_command_terminator);
    sendOSCommandBlind(cmd);
    return IPS_BUSY; // Normal case, should be set to normal by update.
}

IPState OnStep_Aux::MoveAbsFocuser (uint32_t targetTicks)
{
    if (FocusAbsPosNP[0].getMax() >= int(targetTicks) && FocusAbsPosNP[0].getMin() <= int(targetTicks)) {
        char cmd[32];
        char response[RB_MAX_LEN] = {0};
        snprintf(cmd, sizeof(cmd), "%s%06d%s", OS_move_focuser_abs_part, int(targetTicks), OS_command_terminator);
        sendOSCommandBlind(cmd);
        return IPS_BUSY; // Normal case, should be set to normal by update.
    } else {
        LOG_INFO("Unable to move focuser, out of range");
        return IPS_ALERT;
    }
}

IPState OnStep_Aux::MoveRelFocuser (FocusDirection dir, uint32_t ticks)
{
    int output;
    char cmd[32];
    output = ticks;
    if (dir == FOCUS_INWARD) output = 0 - ticks;
    snprintf(cmd, sizeof(cmd), "%s%04d%s", OS_move_focuser_rel_part, output, OS_command_terminator);
    sendOSCommandBlind(cmd);
    return IPS_BUSY; // Normal case, should be set to normal by update.
}

bool OnStep_Aux::AbortFocuser ()
{
    char cmd[CMD_MAX_LEN] = {0};
    strncpy(cmd, OS_stop_focuser, sizeof(cmd));
    return sendOSCommandBlind(cmd);
}

int OnStep_Aux::OSUpdateFocuser()
{
    char value[RB_MAX_LEN] = {0};
    int value_int;
    int error_or_fail = getCommandIntResponse(PortFD, &value_int, value, OS_get_focuser_position);
    if (error_or_fail > 1) {
        FocusAbsPosNP[0].setValue( value_int);
        FocusAbsPosNP.apply();
        LOGF_DEBUG("Current focuser: %d, %f", value_int, FocusAbsPosNP[0].getValue());
    }
    char valueStatus[RB_MAX_LEN] = {0};
    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, valueStatus, OS_get_focuser_status);
    if (error_or_fail > 0 ) {
        if (valueStatus[0] == 'S') {
            FocusRelPosNP.setState(IPS_OK);
            FocusRelPosNP.apply();
            FocusAbsPosNP.setState(IPS_OK);
            FocusAbsPosNP.apply();
        } else if (valueStatus[0] == 'M') {
            FocusRelPosNP.setState(IPS_BUSY);
            FocusRelPosNP.apply();
            FocusAbsPosNP.setState(IPS_BUSY);
            FocusAbsPosNP.apply();
        } else {
            LOG_WARN("Communication :FT# error, check connection.");
            //INVALID REPLY
            FocusRelPosNP.setState(IPS_ALERT);
            FocusRelPosNP.apply();
            FocusAbsPosNP.setState(IPS_ALERT);
            FocusAbsPosNP.apply();
        }
    } else {
        //INVALID REPLY
        LOG_WARN("Communication :FT# error, check connection.");
        FocusRelPosNP.setState(IPS_ALERT);
        FocusRelPosNP.apply();
        FocusAbsPosNP.setState(IPS_ALERT);
        FocusAbsPosNP.apply();
    }
    char focus_max[RB_MAX_LEN] = {0};
    int focus_max_int;
    int fm_error = getCommandIntResponse(PortFD, &focus_max_int, focus_max, OS_get_focuser_max);
    if (fm_error > 0) {
        FocusAbsPosNP[0].setMax(focus_max_int);
        FocusAbsPosNP.updateMinMax();
        FocusAbsPosNP.apply();
        LOGF_DEBUG("focus_max: %s, %i, fm_nbchar: %i", focus_max, focus_max_int, fm_error);
    } else {
        LOG_WARN("Communication :FM# error, check connection.");
        LOGF_WARN("focus_max: %s, %u, fm_error: %i", focus_max, focus_max[0], fm_error);
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_min[RB_MAX_LEN] = {0};
    int focus_min_int ;
    int fi_error = getCommandIntResponse(PortFD, &focus_min_int, focus_min, OS_get_focuser_min);
    if (fi_error > 0) {
        FocusAbsPosNP[0].setMin( focus_min_int);
        FocusAbsPosNP.updateMinMax();
        FocusAbsPosNP.apply();
        LOGF_DEBUG("focus_min: %s, %i fi_nbchar: %i", focus_min, focus_min_int, fi_error);
    } else {
        LOG_WARN("Communication :FI# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_T[RB_MAX_LEN] = {0};
    double focus_T_double ;
    int ft_error = getCommandDoubleResponse(PortFD, &focus_T_double, focus_T, OS_get_focuser_temperature);
    if (ft_error > 0) {
        FocusTemperatureN[0].value = atof(focus_T);
        IDSetNumber(&FocusTemperatureNP, nullptr);
        LOGF_DEBUG("focus T°: %s, focus_T_double %i ft_nbcar: %i", focus_T, focus_T_double, ft_error);
    } else {
        LOG_WARN("Communication :Ft# error, check connection.");
        LOGF_DEBUG("focus T°: %s, focus_T_double %i ft_nbcar: %i", focus_T, focus_T_double, ft_error);
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_TD[RB_MAX_LEN] = {0};
    int focus_TD_int ;
    int fe_error = getCommandIntResponse(PortFD, &focus_TD_int, focus_TD, OS_get_focuser_diff_temperature);
    if (fe_error > 0) {
        FocusTemperatureN[1].value =  atof(focus_TD);
        IDSetNumber(&FocusTemperatureNP, nullptr);
        LOGF_DEBUG("focus Differential T°: %s, %i fi_nbchar: %i", focus_TD, focus_TD_int, fe_error);
    } else {
        LOG_WARN("Communication :Fe# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_Coeficient[RB_MAX_LEN] = {0};
    int focus_Coefficient_int ;
    int fC_error = getCommandIntResponse(PortFD, &focus_Coefficient_int, focus_Coeficient, OS_get_focuser_temp_comp_coef);
    if (fC_error > 0) {
        TFCCoefficientN[0].value =  atof(focus_Coeficient);
        IDSetNumber(&TFCCoefficientNP, nullptr);
        LOGF_DEBUG("TFC Coefficient: %s, %i fC_nbchar: %i", focus_Coeficient, focus_Coefficient_int, fC_error);
    } else {
        LOG_WARN("Communication :FC# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char focus_Deadband[RB_MAX_LEN] = {0};
    int focus_Deadband_int ;
    int fD_error = getCommandIntResponse(PortFD, &focus_Deadband_int, focus_Deadband, OS_get_focuser_deadband);
    if (fD_error > 0) {
        TFCDeadbandN[0].value =  focus_Deadband_int;
        IDSetNumber(&TFCDeadbandNP, nullptr);
        LOGF_DEBUG("TFC Deadband: %s, %i fD_nbchar: %i", focus_Deadband, focus_Deadband_int, fD_error);
    } else {
        LOG_WARN("Communication :FD# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    char response[RB_MAX_LEN];
    int res = getCommandSingleCharResponse(PortFD, response, OS_get_focuser_temp_comp_en);
    if (res > 0) {
        if (strcmp(response, "0")) {
            TFCCompensationSP.s = IPS_OK;
            TFCCompensationS[0].s = ISS_OFF;
            TFCCompensationS[1].s = ISS_ON;
        } else if (strcmp(response, "1")) {
            TFCCompensationSP.s = IPS_OK;
            TFCCompensationS[0].s = ISS_ON;
            TFCCompensationS[1].s = ISS_OFF;
        }
        IDSetSwitch(&TFCCompensationSP, nullptr);
        LOGF_DEBUG("TFC Enable: fc_nbchar:%d Fc_response: %s", res, response);
    } else {
        //LOGF_DEBUG("TFC Enable1: fc_error:%i Fc_response: %s", res, response);
        LOG_WARN("Communication :Fc# error, check connection.");
        flushIO(PortFD); //Unlikely to do anything, but just in case.
    }
    FI::updateProperties();
    LOGF_DEBUG("After update properties: FocusAbsPosN min: %f max: %f", FocusAbsPosNP[0].getMin(), FocusAbsPosNP[0].getMax());

    return 0;
}

/********************
 * Rotator functions
 *******************/

int OnStep_Aux::OSUpdateRotator()
{
    char response[RB_MAX_LEN];
    double double_value;
    if(hasRotator) {
        int error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_rotator_angle);
        if (error_or_fail == 1 && response[0] == '0') { //1 char return, response 0 = no Rotator
            LOG_INFO("Detected Response that Rotator is not present, disabling further checks");
            hasRotator = false;
            return 0; //Return 0, as this is not a communication error
        }
        if (error_or_fail < 1) {  //This does not necessarily mean
            LOG_WARN("Error talking to rotator, might be timeout (especially on network)");
            return -1;
        }
        if (f_scansexa(response, &double_value)) { // 0 = good, thus this is the bad
            GotoRotatorNP.setState(IPS_ALERT);
            GotoRotatorNP.apply();
            return -1;
        }
        GotoRotatorNP[0].setValue(double_value);
        double min_rotator, max_rotator;
        bool changed_minmax = false;
        memset(response, 0, RB_MAX_LEN);
        error_or_fail = getCommandDoubleResponse(PortFD, &min_rotator, response, OS_get_rotator_min);
        if (error_or_fail > 1) {
            changed_minmax = true;
            GotoRotatorNP[0].setMin(min_rotator);
        }
        memset(response, 0, RB_MAX_LEN);
        error_or_fail = getCommandDoubleResponse(PortFD, &max_rotator, response, OS_get_rotator_max);
        if (error_or_fail > 1) {
            changed_minmax = true;
            GotoRotatorNP[0].setMax(max_rotator);
        }
        if (changed_minmax) {
            GotoRotatorNP.updateMinMax();
            GotoRotatorNP.apply();
        }
        //GotoRotatorN
        memset(response, 0, RB_MAX_LEN);
        error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, OS_get_rotator_status);
        if (error_or_fail > 1) {
            if (response[0] == 'S') { /*Stopped normal on EQ mounts */
                GotoRotatorNP.setState(IPS_OK);
                GotoRotatorNP.apply();
            } else if (response[0] == 'M') { /* Moving, including de-rotation */
                GotoRotatorNP.setState(IPS_BUSY);
                GotoRotatorNP.apply();
            } else {
                //INVALID REPLY
                GotoRotatorNP.setState(IPS_ALERT);
                GotoRotatorNP.apply();
            }
        }
        memset(response, 0, RB_MAX_LEN);
        int backlash_value;
        error_or_fail = getCommandIntResponse(PortFD, &backlash_value, response, OS_get_rotator_backlash);
        if (error_or_fail > 1) {
            RotatorBacklashNP[0].setValue(backlash_value);
            RotatorBacklashNP.setState(IPS_OK);
            RotatorBacklashNP.apply();
        }
    }
    return 0;
}

IPState OnStep_Aux::MoveRotator(double angle)
{
    char cmd[CMD_MAX_LEN] = {0};
    char response[RB_MAX_LEN] = {0};
    int d, m, s;
    getSexComponents(angle, &d, &m, &s);

    snprintf(cmd, sizeof(cmd), "%s%.03d:%02d:%02d%s", OS_set_rotator_angle_part, d, m, s, OS_command_terminator);
    LOGF_INFO("Move Rotator: %s", cmd);

    int OS_moveRotator_error_or_fail = getCommandSingleCharResponse(PortFD, response, cmd);

    if (OS_moveRotator_error_or_fail > 1) {
        return IPS_BUSY;
    } else {
        return IPS_ALERT;
    }

    return IPS_BUSY;
}

IPState OnStep_Aux::HomeRotator()
{
    //Not entirely sure if this means attempt to use limit switches and home, or goto home
    //Assuming MOVE to Home
    LOG_INFO("Moving Rotator to Home");
    sendOSCommandBlind(OS_move_rotator_home);
    return IPS_BUSY;
}

bool OnStep_Aux::AbortRotator()
{
    LOG_INFO("Aborting Rotation, de-rotation in same state");
    sendOSCommandBlind(OS_stop_rotator); //Does NOT abort de-rotator
    return true;
}

bool OnStep_Aux::SetRotatorBacklash(int32_t steps)
{
    char cmd[CMD_MAX_LEN] = {0};
    snprintf(cmd, sizeof(cmd), "%s%d%s", OS_set_rotator_backlash_part, steps, OS_command_terminator);
    if(sendOSCommand(cmd)) {
        return true;
    }
    return false;
}

bool OnStep_Aux::SetRotatorBacklashEnabled(bool enabled)
{
    //Nothing required here.
    INDI_UNUSED(enabled);
    return true;
    //     As it's always enabled, which would mean setting it like SetRotatorBacklash to 0, and losing any saved values. So for now, leave it as is (always enabled)
}

/***********************************************************
** Client is asking us to establish connection to the device
************************************************************/
bool OnStep_Aux::Connect()
{
    if (!INDI::DefaultDevice::Connect()) {
        LOG_ERROR("Parent Connect() failed");
        return false;
    }

    // Start polling timer (e.g., every 1000ms)
    SetTimer(getCurrentPollingPeriod());

    return true;
}

/***********************************************************
** Client is asking us to terminate connection to the device
************************************************************/
bool OnStep_Aux::Disconnect()
{
    bool status = INDI::DefaultDevice::Disconnect();
    return status;
}

/****************************
* Poll properties for updates
****************************/
void OnStep_Aux::TimerHit()
{
    char cmd[CMD_MAX_LEN] = {0};
    char response[RB_MAX_LEN] = {0};
    int intResponse = 0;
    int error_or_fail = 0;
    if (hasFeature) {
        for (int feature = 0; feature < max_features; feature++) {
            if (features_enabled[feature]) {
                switch (features_type[feature]) {
                case SWITCH:
                case MOMENTARY_SWITCH:
                case COVER_SWITCH:
                    snprintf(cmd, sizeof(cmd), "%s%d%s", OS_get_feature_state_part, (feature + 1), OS_command_terminator);
                    error_or_fail = getCommandIntFromCharResponse(PortFD, response, &intResponse, cmd);
                    if (error_or_fail > 0 ) {
                        if (intResponse == 0) {
                            switch (feature) {
                            case 0:
                                Switch1S[OFF_SWITCH].s = ISS_ON;
                                Switch1S[ON_SWITCH].s = ISS_OFF;
                                Switch1SP.s = IPS_OK;
                                IDSetSwitch(&Switch1SP, nullptr);
                                break;
                            case 1:
                                Switch2S[OFF_SWITCH].s = ISS_ON;
                                Switch2S[ON_SWITCH].s = ISS_OFF;
                                Switch2SP.s = IPS_OK;
                                IDSetSwitch(&Switch2SP, nullptr);
                                break;
                            case 2:
                                Switch3S[OFF_SWITCH].s = ISS_ON;
                                Switch3S[ON_SWITCH].s = ISS_OFF;
                                Switch3SP.s = IPS_OK;
                                IDSetSwitch(&Switch3SP, nullptr);
                                break;
                            case 3:
                                Switch4S[OFF_SWITCH].s = ISS_ON;
                                Switch4S[ON_SWITCH].s = ISS_OFF;
                                Switch4SP.s = IPS_OK;
                                IDSetSwitch(&Switch4SP, nullptr);
                                break;
                            case 4:
                                Switch5S[OFF_SWITCH].s = ISS_ON;
                                Switch5S[ON_SWITCH].s = ISS_OFF;
                                Switch5SP.s = IPS_OK;
                                IDSetSwitch(&Switch5SP, nullptr);
                                break;
                            case 5:
                                Switch6S[OFF_SWITCH].s = ISS_ON;
                                Switch6S[ON_SWITCH].s = ISS_OFF;
                                Switch6SP.s = IPS_OK;
                                IDSetSwitch(&Switch6SP, nullptr);
                                break;
                            case 6:
                                Switch7S[OFF_SWITCH].s = ISS_ON;
                                Switch7S[ON_SWITCH].s = ISS_OFF;
                                Switch7SP.s = IPS_OK;
                                IDSetSwitch(&Switch7SP, nullptr);
                                break;
                            case 7:
                                Switch8S[OFF_SWITCH].s = ISS_ON;
                                Switch8S[ON_SWITCH].s = ISS_OFF;
                                Switch8SP.s = IPS_OK;
                                IDSetSwitch(&Switch8SP, nullptr);
                                break;
                            default:
                                break;
                            }
                        } else if (intResponse == 1) {
                            switch (feature) {
                            case 0:
                                Switch1S[OFF_SWITCH].s = ISS_OFF;
                                Switch1S[ON_SWITCH].s = ISS_ON;
                                Switch1SP.s = IPS_OK;
                                IDSetSwitch(&Switch1SP, nullptr);
                                break;
                            case 1:
                                Switch2S[OFF_SWITCH].s = ISS_OFF;
                                Switch2S[ON_SWITCH].s = ISS_ON;
                                Switch2SP.s = IPS_OK;
                                IDSetSwitch(&Switch2SP, nullptr);
                                break;
                            case 2:
                                Switch3S[OFF_SWITCH].s = ISS_OFF;
                                Switch3S[ON_SWITCH].s = ISS_ON;
                                Switch3SP.s = IPS_OK;
                                IDSetSwitch(&Switch3SP, nullptr);
                                break;
                            case 3:
                                Switch4S[OFF_SWITCH].s = ISS_OFF;
                                Switch4S[ON_SWITCH].s = ISS_ON;
                                Switch4SP.s = IPS_OK;
                                IDSetSwitch(&Switch4SP, nullptr);
                                break;
                            case 4:
                                Switch5S[OFF_SWITCH].s = ISS_OFF;
                                Switch5S[ON_SWITCH].s = ISS_ON;
                                Switch5SP.s = IPS_OK;
                                IDSetSwitch(&Switch5SP, nullptr);
                                break;
                            case 5:
                                Switch6S[OFF_SWITCH].s = ISS_OFF;
                                Switch6S[ON_SWITCH].s = ISS_ON;
                                Switch6SP.s = IPS_OK;
                                IDSetSwitch(&Switch6SP, nullptr);
                                break;
                            case 6:
                                Switch7S[OFF_SWITCH].s = ISS_OFF;
                                Switch7S[ON_SWITCH].s = ISS_ON;
                                Switch7SP.s = IPS_OK;
                                IDSetSwitch(&Switch7SP, nullptr);
                                break;
                            case 7:
                                Switch8S[OFF_SWITCH].s = ISS_OFF;
                                Switch8S[ON_SWITCH].s = ISS_ON;
                                Switch8SP.s = IPS_OK;
                                IDSetSwitch(&Switch8SP, nullptr);
                                break;
                            default:
                                break;
                            }
                        } else {
                            LOGF_ERROR("Invalid response to get bool feature status: %d", intResponse);
                        }
                    }
                    break;
                case DEW_HEATER:
                    snprintf(cmd, sizeof(cmd), "%s%d%s", OS_get_feature_state_part, (feature + 1), OS_command_terminator);
                    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, cmd);
                    if (error_or_fail > 0 ) {
                        bool dew_enabled = false;
                        float dew_zero = 0.0;
                        float dew_span = 0.0;
                        char dew_delta[RB_MAX_LEN] = {0};
                        bool valid = false;

                        // First returned part is enabled 0 or 1
                        char *split;
                        split = strtok(response, ",");
                        int value = conversion_error;
                        try {
                            value = std::stoi(split);
                        } catch (const std::invalid_argument&) {
                            LOGF_WARN("Invalid response to %s: %s", cmd, response);
                        } catch (const std::out_of_range&) {
                            LOGF_WARN("Invalid response to %s: %s", cmd, response);
                        }
                        if (value < 0 || value > 1) {
                            LOGF_WARN("Invalid response to %s: %s", cmd, response);
                        } else {
                            dew_enabled = static_cast<bool>(value);
                            valid = true;
                        }

                        // Second returned part is zero point in degC
                        if (valid) {
                            split = strtok(response, ",");
                            float fValue = static_cast<float>(conversion_error);
                            try {
                                fValue = std::stof(split);
                            } catch (const std::invalid_argument&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            } catch (const std::out_of_range&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            }
                            if (fValue < -5.0 || fValue > 20.0) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                                valid = false;
                            } else {
                                dew_zero = fValue;
                            }
                        }

                        // Third returned part is span range in degC
                        if (valid) {
                            split = strtok(response, ",");
                            float fValue = static_cast<float>(conversion_error);
                            try {
                                fValue = std::stof(split);
                            } catch (const std::invalid_argument&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            } catch (const std::out_of_range&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            }
                            if (fValue < -5.0 || fValue > 20.0) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                                valid = false;
                            } else {
                                dew_span = fValue;
                            }
                        }

                        // Forth returned part is temperature - dew point in degC
                        if (valid) {
                            split = strtok(response, ",");
                            float fValue = static_cast<float>(conversion_error);
                            try {
                                fValue = std::stof(split);
                            } catch (const std::invalid_argument&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            } catch (const std::out_of_range&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            }
                            if (fValue < -5.0 || fValue > 20.0) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                                valid = false;
                            } else {
                                indi_strlcpy(dew_delta, split, sizeof(dew_delta));
                            }
                        }

                        if (valid) {
                            switch (feature) {
                            case 0:
                                if (dew_enabled) {
                                    Dew1_enableS[OFF_SWITCH].s = ISS_ON;
                                    Dew1_enableS[ON_SWITCH].s = ISS_OFF;
                                } else {
                                    Dew1_enableS[OFF_SWITCH].s = ISS_OFF;
                                    Dew1_enableS[ON_SWITCH].s = ISS_ON;
                                }
                                Dew1SP.s = IPS_OK;
                                IDSetSwitch(&Dew1SP, nullptr);
                                Dew1_zeroN[0].value = dew_zero;
                                Dew1_spanN[0].value = dew_span;
                                IDSetNumber(&Dew1NP, nullptr);
                                Dew1_deltaT[0].text = dew_delta;
                                IDSetText(&Dew1deltaTP, nullptr);
                                break;
                            case 1:
                                if (dew_enabled) {
                                    Dew2_enableS[OFF_SWITCH].s = ISS_ON;
                                    Dew2_enableS[ON_SWITCH].s = ISS_OFF;
                                } else {
                                    Dew1_enableS[OFF_SWITCH].s = ISS_OFF;
                                    Dew1_enableS[ON_SWITCH].s = ISS_ON;
                                }
                                Dew2SP.s = IPS_OK;
                                IDSetSwitch(&Dew2SP, nullptr);
                                Dew2_zeroN[0].value = dew_zero;
                                Dew2_spanN[0].value = dew_span;
                                IDSetNumber(&Dew2NP, nullptr);
                                Dew2_deltaT[0].text = dew_delta;
                                IDSetText(&Dew2deltaTP, nullptr);
                                break;
                            case 2:
                                if (dew_enabled) {
                                    Dew3_enableS[OFF_SWITCH].s = ISS_ON;
                                    Dew3_enableS[ON_SWITCH].s = ISS_OFF;
                                } else {
                                    Dew3_enableS[OFF_SWITCH].s = ISS_OFF;
                                    Dew3_enableS[ON_SWITCH].s = ISS_ON;
                                }
                                Dew3SP.s = IPS_OK;
                                IDSetSwitch(&Dew3SP, nullptr);
                                Dew3_zeroN[0].value = dew_zero;
                                Dew3_spanN[0].value = dew_span;
                                IDSetNumber(&Dew3NP, nullptr);
                                Dew3_deltaT[0].text = dew_delta;
                                IDSetText(&Dew3deltaTP, nullptr);
                                break;
                            case 3:
                                if (dew_enabled) {
                                    Dew4_enableS[OFF_SWITCH].s = ISS_ON;
                                    Dew4_enableS[ON_SWITCH].s = ISS_OFF;
                                } else {
                                    Dew4_enableS[OFF_SWITCH].s = ISS_OFF;
                                    Dew4_enableS[ON_SWITCH].s = ISS_ON;
                                }
                                Dew4SP.s = IPS_OK;
                                IDSetSwitch(&Dew4SP, nullptr);
                                Dew4_zeroN[0].value = dew_zero;
                                Dew4_spanN[0].value = dew_span;
                                IDSetNumber(&Dew4NP, nullptr);
                                Dew4_deltaT[0].text = dew_delta;
                                IDSetText(&Dew4deltaTP, nullptr);
                                break;
                            case 4:
                                if (dew_enabled) {
                                    Dew5_enableS[OFF_SWITCH].s = ISS_ON;
                                    Dew5_enableS[ON_SWITCH].s = ISS_OFF;
                                } else {
                                    Dew5_enableS[OFF_SWITCH].s = ISS_OFF;
                                    Dew5_enableS[ON_SWITCH].s = ISS_ON;
                                }
                                Dew5SP.s = IPS_OK;
                                IDSetSwitch(&Dew5SP, nullptr);
                                Dew5_zeroN[0].value = dew_zero;
                                Dew5_spanN[0].value = dew_span;
                                IDSetNumber(&Dew5NP, nullptr);
                                Dew5_deltaT[0].text = dew_delta;
                                IDSetText(&Dew5deltaTP, nullptr);
                                break;
                            case 5:
                                if (dew_enabled) {
                                    Dew6_enableS[OFF_SWITCH].s = ISS_ON;
                                    Dew6_enableS[ON_SWITCH].s = ISS_OFF;
                                } else {
                                    Dew6_enableS[OFF_SWITCH].s = ISS_OFF;
                                    Dew6_enableS[ON_SWITCH].s = ISS_ON;
                                }
                                Dew6SP.s = IPS_OK;
                                IDSetSwitch(&Dew6SP, nullptr);
                                Dew6_zeroN[0].value = dew_zero;
                                Dew6_spanN[0].value = dew_span;
                                IDSetNumber(&Dew6NP, nullptr);
                                Dew6_deltaT[0].text = dew_delta;
                                IDSetText(&Dew6deltaTP, nullptr);
                                break;
                            case 6:
                                if (dew_enabled) {
                                    Dew7_enableS[OFF_SWITCH].s = ISS_ON;
                                    Dew7_enableS[ON_SWITCH].s = ISS_OFF;
                                } else {
                                    Dew7_enableS[OFF_SWITCH].s = ISS_OFF;
                                    Dew7_enableS[ON_SWITCH].s = ISS_ON;
                                }
                                Dew7SP.s = IPS_OK;
                                IDSetSwitch(&Dew7SP, nullptr);
                                Dew7_zeroN[0].value = dew_zero;
                                Dew7_spanN[0].value = dew_span;
                                IDSetNumber(&Dew7NP, nullptr);
                                Dew7_deltaT[0].text = dew_delta;
                                IDSetText(&Dew7deltaTP, nullptr);
                                break;
                            case 7:
                                if (dew_enabled) {
                                    Dew8_enableS[OFF_SWITCH].s = ISS_ON;
                                    Dew8_enableS[ON_SWITCH].s = ISS_OFF;
                                } else {
                                    Dew8_enableS[OFF_SWITCH].s = ISS_OFF;
                                    Dew8_enableS[ON_SWITCH].s = ISS_ON;
                                }
                                Dew8SP.s = IPS_OK;
                                IDSetSwitch(&Dew8SP, nullptr);
                                Dew8_zeroN[0].value = dew_zero;
                                Dew8_spanN[0].value = dew_span;
                                IDSetNumber(&Dew8NP, nullptr);
                                Dew8_deltaT[0].text = dew_delta;
                                IDSetText(&Dew8deltaTP, nullptr);
                                break;
                            default:
                                break;
                            }
                        };
                    }
                    break;
                case INTERVALOMETER:
                    snprintf(cmd, sizeof(cmd), "%s%d%s", OS_get_feature_state_part, (feature + 1), OS_command_terminator);
                    error_or_fail = getCommandSingleCharErrorOrLongResponse(PortFD, response, cmd);
                    if (error_or_fail > 0 ) {
                        float inter_exp = 0.0;
                        float inter_delay = 0.0;
                        int inter_count = 0;
                        char inter_done[RB_MAX_LEN] = {0};
                        bool valid = false;

                        // First returned part is count of completed exposures
                        char *split;
                        split = strtok(response, ",");
                        int value = conversion_error;
                        try {
                            value = std::stoi(split);
                        } catch (const std::invalid_argument&) {
                            LOGF_WARN("Invalid response to %s: %s", cmd, response);
                        } catch (const std::out_of_range&) {
                            LOGF_WARN("Invalid response to %s: %s", cmd, response);
                        }
                        if (value < 0 || value > 1) {
                            LOGF_WARN("Invalid response to %s: %s", cmd, response);
                        } else {
                            indi_strlcpy(inter_done, split, sizeof(inter_done));
                            valid = true;
                        }

                        // Second returned part is exposure time in secs
                        if (valid) {
                            split = strtok(response, ",");
                            float fValue = static_cast<float>(conversion_error);
                            try {
                                fValue = std::stof(split);
                            } catch (const std::invalid_argument&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            } catch (const std::out_of_range&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            }
                            if (fValue < -5.0 || fValue > 20.0) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                                valid = false;
                            } else {
                                inter_exp = fValue;
                            }
                        }

                        // Third returned part is delay time in secs
                        if (valid) {
                            split = strtok(response, ",");
                            float fValue = static_cast<float>(conversion_error);
                            try {
                                fValue = std::stof(split);
                            } catch (const std::invalid_argument&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            } catch (const std::out_of_range&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            }
                            if (fValue < -5.0 || fValue > 20.0) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                                valid = false;
                            } else {
                                inter_delay = fValue;
                            }
                        }

                        // Forth returned part set count
                        if (valid) {
                            split = strtok(response, ",");
                            float fValue = static_cast<float>(conversion_error);
                            try {
                                fValue = std::stof(split);
                            } catch (const std::invalid_argument&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            } catch (const std::out_of_range&) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                            }
                            if (fValue < -5.0 || fValue > 20.0) {
                                LOGF_WARN("Invalid response to %s: %s", cmd, response);
                                valid = false;
                            } else {
                                inter_count = fValue;
                            }
                        }

                        if (valid) {
                            switch (feature) {
                            case 0:
                                Inter1_expN[0].value = inter_exp;
                                Inter1_delayN[0].value = inter_delay;
                                Inter1_countN[0].value = inter_count;
                                IDSetNumber(&Inter1NP, nullptr);
                                Inter1_doneT[0].text = inter_done;
                                IDSetText(&Inter1doneTP, nullptr);
                                break;
                            case 1:
                                Inter2_expN[0].value = inter_exp;
                                Inter2_delayN[0].value = inter_delay;
                                Inter2_countN[0].value = inter_count;
                                IDSetNumber(&Inter2NP, nullptr);
                                Inter2_doneT[0].text = inter_done;
                                IDSetText(&Inter2doneTP, nullptr);
                                break;
                            case 2:
                                Inter3_expN[0].value = inter_exp;
                                Inter3_delayN[0].value = inter_delay;
                                Inter3_countN[0].value = inter_count;
                                IDSetNumber(&Inter3NP, nullptr);
                                Inter3_doneT[0].text = inter_done;
                                IDSetText(&Inter3doneTP, nullptr);
                                break;
                            case 3:
                                Inter4_expN[0].value = inter_exp;
                                Inter4_delayN[0].value = inter_delay;
                                Inter4_countN[0].value = inter_count;
                                IDSetNumber(&Inter4NP, nullptr);
                                Inter4_doneT[0].text = inter_done;
                                IDSetText(&Inter4doneTP, nullptr);
                                break;
                            case 4:
                                Inter5_expN[0].value = inter_exp;
                                Inter5_delayN[0].value = inter_delay;
                                Inter5_countN[0].value = inter_count;
                                IDSetNumber(&Inter5NP, nullptr);
                                Inter5_doneT[0].text = inter_done;
                                IDSetText(&Inter5doneTP, nullptr);
                                break;
                            case 5:
                                Inter6_expN[0].value = inter_exp;
                                Inter6_delayN[0].value = inter_delay;
                                Inter6_countN[0].value = inter_count;
                                IDSetNumber(&Inter6NP, nullptr);
                                Inter6_doneT[0].text = inter_done;
                                IDSetText(&Inter6doneTP, nullptr);
                                break;
                            case 6:
                                Inter7_expN[0].value = inter_exp;
                                Inter7_delayN[0].value = inter_delay;
                                Inter7_countN[0].value = inter_count;
                                IDSetNumber(&Inter7NP, nullptr);
                                Inter7_doneT[0].text = inter_done;
                                IDSetText(&Inter7doneTP, nullptr);
                                break;
                            case 7:
                                Inter8_expN[0].value = inter_exp;
                                Inter8_delayN[0].value = inter_delay;
                                Inter8_countN[0].value = inter_count;
                                IDSetNumber(&Inter8NP, nullptr);
                                Inter8_doneT[0].text = inter_done;
                                IDSetText(&Inter8doneTP, nullptr);
                                break;
                            default:
                                break;
                            }
                        }
                    }
                default:
                    break;
                }
            }
        }
    }
    if (hasUSB) {
        for (int USBport = 0; USBport < USBportCount; USBport++) {
            if (USBports_enabled[USBport]) {
                memset(response, 0, RB_MAX_LEN);
                memset(cmd, 0, CMD_MAX_LEN);
                int intResponse = 0;
                int error_or_fail = 0;
                snprintf(cmd, sizeof(cmd), "%s%d%s", OS_get_USBport_state_part, (USBport + 1), OS_command_terminator);
                error_or_fail = getCommandIntFromCharResponse(PortFD, response, &intResponse, cmd);
                if (error_or_fail > 0) {
                    if (intResponse == 0) {
                        switch (USBport) {
                        case 0:
                            USB1S[OFF_SWITCH].s = ISS_ON;
                            USB1S[ON_SWITCH].s = ISS_OFF;
                            USB1SP.s = IPS_OK;
                            IDSetSwitch(&USB1SP, nullptr);
                            break;
                        case 1:
                            USB2S[OFF_SWITCH].s = ISS_ON;
                            USB2S[ON_SWITCH].s = ISS_OFF;
                            USB2SP.s = IPS_OK;
                            IDSetSwitch(&USB2SP, nullptr);
                            break;
                        case 2:
                            USB3S[OFF_SWITCH].s = ISS_ON;
                            USB3S[ON_SWITCH].s = ISS_OFF;
                            USB3SP.s = IPS_OK;
                            IDSetSwitch(&USB3SP, nullptr);
                            break;
                        case 3:
                            USB4S[OFF_SWITCH].s = ISS_ON;
                            USB4S[ON_SWITCH].s = ISS_OFF;
                            USB4SP.s = IPS_OK;
                            IDSetSwitch(&USB4SP, nullptr);
                            break;
                        case 4:
                            USB5S[OFF_SWITCH].s = ISS_ON;
                            USB5S[ON_SWITCH].s = ISS_OFF;
                            USB5SP.s = IPS_OK;
                            IDSetSwitch(&USB5SP, nullptr);
                            break;
                        case 5:
                            USB6S[OFF_SWITCH].s = ISS_ON;
                            USB6S[ON_SWITCH].s = ISS_OFF;
                            USB6SP.s = IPS_OK;
                            IDSetSwitch(&USB6SP, nullptr);
                            break;
                        case 6:
                            USB7S[OFF_SWITCH].s = ISS_ON;
                            USB7S[ON_SWITCH].s = ISS_OFF;
                            USB7SP.s = IPS_OK;
                            IDSetSwitch(&USB7SP, nullptr);
                            break;
                        case 7:
                            USB8S[OFF_SWITCH].s = ISS_ON;
                            USB8S[ON_SWITCH].s = ISS_OFF;
                            USB8SP.s = IPS_OK;
                            IDSetSwitch(&USB8SP, nullptr);
                            break;
                        default:
                            break;
                        }
                    } else if (intResponse == 1) {
                        switch (USBport) {
                        case 0:
                            USB1S[OFF_SWITCH].s = ISS_OFF;
                            USB1S[ON_SWITCH].s = ISS_ON;
                            USB1SP.s = IPS_OK;
                            IDSetSwitch(&USB1SP, nullptr);
                            break;
                        case 1:
                            USB2S[OFF_SWITCH].s = ISS_OFF;
                            USB2S[ON_SWITCH].s = ISS_ON;
                            USB2SP.s = IPS_OK;
                            IDSetSwitch(&USB2SP, nullptr);
                            break;
                        case 2:
                            USB3S[OFF_SWITCH].s = ISS_OFF;
                            USB3S[ON_SWITCH].s = ISS_ON;
                            USB3SP.s = IPS_OK;
                            IDSetSwitch(&USB3SP, nullptr);
                            break;
                        case 3:
                            USB4S[OFF_SWITCH].s = ISS_OFF;
                            USB4S[ON_SWITCH].s = ISS_ON;
                            USB4SP.s = IPS_OK;
                            IDSetSwitch(&USB4SP, nullptr);
                            break;
                        case 4:
                            USB5S[OFF_SWITCH].s = ISS_OFF;
                            USB5S[ON_SWITCH].s = ISS_ON;
                            USB5SP.s = IPS_OK;
                            IDSetSwitch(&USB5SP, nullptr);
                            break;
                        case 5:
                            USB6S[OFF_SWITCH].s = ISS_OFF;
                            USB6S[ON_SWITCH].s = ISS_ON;
                            USB6SP.s = IPS_OK;
                            IDSetSwitch(&USB6SP, nullptr);
                            break;
                        case 6:
                            USB7S[OFF_SWITCH].s = ISS_OFF;
                            USB7S[ON_SWITCH].s = ISS_ON;
                            USB7SP.s = IPS_OK;
                            IDSetSwitch(&USB7SP, nullptr);
                            break;
                        case 7:
                            USB8S[OFF_SWITCH].s = ISS_OFF;
                            USB8S[ON_SWITCH].s = ISS_ON;
                            USB8SP.s = IPS_OK;
                            IDSetSwitch(&USB8SP, nullptr);
                            break;
                        default:
                            break;
                        }
                    } else {
                        LOGF_ERROR("Invalid response to get USB status: %d", intResponse);
                    }
                }
            }
        } // End USB port switches

        // Check if ALL defined USB ports are On/Off and use to set USBall switches
        int USBportsOn = 0;
        for (int USBport = 0; USBport < max_USBports; USBport++) {
            if (USBports_enabled[USBport] == 1) {
                memset(response, 0, RB_MAX_LEN);
                memset(cmd, 0, CMD_MAX_LEN);
                int intResponse = 0;
                int error_or_fail = 0;
                snprintf(cmd, sizeof(cmd), "%s%d%s", OS_get_USBport_state_part, (USBport + 1), OS_command_terminator);
                error_or_fail = getCommandIntFromCharResponse(PortFD, response, &intResponse, cmd);
                if (error_or_fail > 0) {
                    if (intResponse == 1) {
                        USBportsOn++;
                    }
                }
            }
        }
        if (USBportsOn == 0) {
            USBallS[OFF_SWITCH].s = ISS_ON;
            USBallS[ON_SWITCH].s = ISS_OFF;
            USBallSP.s = IPS_OK;
            IDSetSwitch(&USBallSP, nullptr);
        } else if (USBportsOn == USBportCount) {
            USBallS[OFF_SWITCH].s = ISS_OFF;
            USBallS[ON_SWITCH].s = ISS_ON;
            USBallSP.s = IPS_OK;
            IDSetSwitch(&USBallSP, nullptr);
        } else {
            USBallS[OFF_SWITCH].s = ISS_OFF;
            USBallS[ON_SWITCH].s = ISS_OFF;
            USBallSP.s = IPS_OK;
            IDSetSwitch(&USBallSP, nullptr);
        }
        // End USBall
    }
    // Timer loop control
    if (isConnected()) {
        SetTimer(getCurrentPollingPeriod());
    } else {
        return; //  No need to reset timer if we are not connected anymore
    }
}

const char *OnStep_Aux::getDefaultName()
{
    return "OnStep Aux";
}

/*****************************************************************
* Poll Weather properties for updates - period set by Weather poll
******************************************************************/
IPState OnStep_Aux::updateWeather() {
    if (hasWeather) {

        LOG_DEBUG("Weather update called");

        for (int measurement = 0; measurement < WEATHER_MEASUREMENTS_COUNT; measurement ++) {
            if (weather_enabled[measurement] == 1) {
                char measurement_reponse[RB_MAX_LEN];
                char measurement_command[CMD_MAX_LEN];

                LOGF_DEBUG("In weather measurements loop, %u", measurement);

                switch (measurement) {
                case WEATHER_TEMPERATURE:
                    indi_strlcpy(measurement_command, OS_get_temperature, sizeof(measurement_command));
                    break;
                case WEATHER_PRESSURE:
                    indi_strlcpy(measurement_command, OS_get_pressure, sizeof(measurement_command));
                    break;
                case WEATHER_HUMIDITY:
                    indi_strlcpy(measurement_command, OS_get_humidity, sizeof(measurement_command));
                    break;
                case WEATHER_DEW_POINT:
                    indi_strlcpy(measurement_command, OS_get_dew_point, sizeof(measurement_command));
                    break;
                default:
                    break;
                }

                double value = conversion_error;
                int measurement_error_or_fail = getCommandDoubleResponse(PortFD, &value, measurement_reponse,
                                                                         measurement_command);
                if ((measurement_error_or_fail >= 0) && (value != conversion_error) &&
                    (weather_enabled[measurement] == 1)) {
                    switch(measurement) {
                    case WEATHER_TEMPERATURE:
                        setParameterValue("WEATHER_TEMPERATURE", value);
                        break;
                    case WEATHER_PRESSURE:
                        setParameterValue("WEATHER_PRESSURE", value);
                        break;
                    case WEATHER_HUMIDITY:
                        setParameterValue("WEATHER_HUMIDITY", value);
                        break;
                    case WEATHER_DEW_POINT:
                        setParameterValue("WEATHER_DEWPOINT", value);
                        break;
                    default:
                        break;
                    }
                }
            }
        }

        if (WI::syncCriticalParameters()) {
            LOG_DEBUG("SyncCriticalParameters = true");
        } else {
            LOG_DEBUG("SyncCriticalParameters = false");
        }
    }

    return IPS_OK;
}
bool OnStep_Aux::saveConfigItems(FILE *fp)
{
    DefaultDevice::saveConfigItems(fp);
    FI::saveConfigItems(fp);
    WI::saveConfigItems(fp);
    RI::saveConfigItems(fp);
    return true;
}

/*********************************************************************
 * Send command to OCS without checking (intended non-existent) return
 * *******************************************************************/
bool OnStep_Aux::sendOSCommandBlind(const char *cmd)
{
    // No need to block this command as there is no response

    int error_type;
    int nbytes_write = 0;
    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);
    flushIO(PortFD);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osCommsLock);
    tcflush(PortFD, TCIFLUSH);
    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK) {
        INDI_UNUSED(error_type);
        LOGF_ERROR("CHECK CONNECTION: Error sending command %s", cmd);
        clearBlock();
        return 0; //Fail if we can't write
    }
    return 1;
}

/*********************************************************************
 * Send command to OCS that expects a 0 (sucess) or 1 (failure) return
 * *******************************************************************/
bool OnStep_Aux::sendOSCommand(const char *cmd)
{
    blockUntilClear();

    char response[1] = {0};
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(PortFD);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osCommsLock);
    tcflush(PortFD, TCIFLUSH);

    if ((error_type = tty_write_string(PortFD, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(PortFD, response, 1, OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);
    INDI_UNUSED(error_type);

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
int OnStep_Aux::getCommandSingleCharResponse(int fd, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osCommsLock);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(fd, data, 1, OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);
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
int OnStep_Aux::getCommandDoubleResponse(int fd, double *value, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_section_expanded(fd, data, '#', OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);
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
int OnStep_Aux::getCommandIntResponse(int fd, int *value, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_expanded(fd, data, sizeof(char), OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);
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
int OnStep_Aux::getCommandSingleCharErrorOrLongResponse(int fd, char *data, const char *cmd)
{
    blockUntilClear();

    char *term;
    int error_type;
    int nbytes_write = 0, nbytes_read = 0;

    DEBUGF(INDI::Logger::DBG_DEBUG, "CMD <%s>", cmd);

    flushIO(fd);
    /* Add mutex */
    std::unique_lock<std::mutex> guard(osCommsLock);
    tcflush(fd, TCIFLUSH);

    if ((error_type = tty_write_string(fd, cmd, &nbytes_write)) != TTY_OK)
        return error_type;

    error_type = tty_read_section_expanded(fd, data, '#', OSTimeoutSeconds, OSTimeoutMicroSeconds, &nbytes_read);
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
int OnStep_Aux::getCommandIntFromCharResponse(int fd, char *data, int *response, const char *cmd)
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
int OnStep_Aux::flushIO(int fd)
{
    tcflush(fd, TCIOFLUSH);
    int error_type = 0;
    int nbytes_read;
    std::unique_lock<std::mutex> guard(osCommsLock);
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

int OnStep_Aux::charToInt (char *inString)
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
void OnStep_Aux::blockUntilClear()
{
    // Blocking wait for last command response to clear
    while (waitingForResponse) {
        usleep(((OSTimeoutSeconds * 1000000) + OSTimeoutMicroSeconds) / 10);
        //        usleep(OCSTimeoutMicroSeconds / 10);
    }
    // Grab the response waiting command blocker
    waitingForResponse = true;
}

/*********************************************
 * Flush port and clear command sequence block
 * *******************************************/
void OnStep_Aux::clearBlock()
{
    waitingForResponse = false;
}

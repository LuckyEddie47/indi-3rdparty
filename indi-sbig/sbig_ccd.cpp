/*
    Driver type: SBIG CCD Camera INDI Driver

    Copyright (C) 2022 Tobias Illenseer (tillense AT astrophysik DOT uni-kiel DOT de)
    Copyright (C) 2013-2018 Jasem Mutlaq (mutlaqja AT ikarustech DOT com)
    Copyright (C) 2017 Peter Polakovic (peter DOT polakovic AT cloudmakers DOT eu)
    Copyright (C) 2005-2006 Jan Soldan (jsoldan AT asu DOT cas DOT cz)

    Acknowledgement:
    Matt Longmire 	(matto AT sbig DOT com)

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation; either version 2.1 of the License, or
    (at your option) any later version.

    This library is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
    License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this library; if not, write to the Free Software Foundation,
    Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

    2016-01-07: Added ETH connection (by Simon Holmbo)
    2016-01-07: Changed Device port from text to switch (JM)
    2017-06-22: Bugfixes and code cleanup (PP)
    2018-12-01: Added switch to ignore shutter errors which can affect some cameras (JM)

 */

#include "sbig_ccd.h"

#include <eventloop.h>

#include <math.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <memory>
#include <deque>

#ifdef __APPLE__
#include <sys/stat.h>
#include "ezusb.h"
#endif

#define TEMPERATURE_POLL_MS 5000 /* Temperature Polling time (ms) */
#define MAX_RESOLUTION      4096 /* Maximum resolution for secondary chip */
#define MAX_DEVICES         20   /* Max device cameraCount */
#define MAX_THREAD_RETRIES  3
#define MAX_THREAD_WAIT     300000

static class Loader
{
        std::deque<std::unique_ptr<SBIGCCD>> cameras;
    public:
        Loader()
        {
            cameras.push_back(std::unique_ptr<SBIGCCD>(new SBIGCCD()));
        }
} loader;

//==========================================================================

void SBIGCCD::loadFirmwareOnOSXifNeeded()
{
    // Upload firmware in case of MacOS
#ifdef __APPLE__

    //SBIG Universal Driver Check
    const std::string name = "/System/Library/Extensions/SBIGUSBEDriver.kext";
    struct stat buffer;
    if (stat (name.c_str(), &buffer) == 0)
    {
        LOG_DEBUG("SBIG Universal Driver Detected");
    }
    else
    {
        LOGF_WARN("Failed to Detect SBIG Universal Driver, please install this before running the INDI SBIG driver!", __FUNCTION__);
    }

    int rc = 0;
    int i = 0;
    int cnt = 0;

    libusb_device **list = nullptr;
    struct libusb_device_descriptor desc;
    std::string bus_name, device_name;

    if ((rc = libusb_init(nullptr)))
    {
        LOGF_WARN("Failed to start libusb", __FUNCTION__, libusb_error_name(rc));
    }

    //libusb_set_debug(nullptr, verbose);  maybe?

    cnt = libusb_get_device_list(nullptr, &list);
    if(cnt < 0)
        LOGF_WARN("Failed to get device list", __FUNCTION__, libusb_error_name(rc));
    handle = nullptr;
    for (i = 0; i < cnt; ++i)
    {
        if (!libusb_get_device_descriptor(list[i], &desc))
        {
            int sbigCameraTypeFound = 0;
            // SBIG ST-7/8/9/10/2K cameras
            if ((desc.idVendor == 0x0d97) && (desc.idProduct == 0x0001))
                sbigCameraTypeFound = 1;
            //Need the code here to detect ST-4K Camera, since it has the same Vendor and Product ID as above
            // SBIG ST-L cameras
            if ((desc.idVendor == 0x0d97) && (desc.idProduct == 0x0002))
                sbigCameraTypeFound = 3;
            // SBIG ST-402/1603/3200/8300 cameras
            if ((desc.idVendor == 0x0d97) && (desc.idProduct == 0x0003))
                sbigCameraTypeFound = 4;

            if(sbigCameraTypeFound != 0)
            {
                libusb_open(list[i], &handle);
                if (handle)
                {
                    libusb_kernel_driver_active(handle, 0);
                    libusb_claim_interface(handle, 0);
                    char driverSupportPath[MAXRBUF];
                    //On OS X, Prefer embedded App location if it exists
                    if (getenv("INDIPREFIX") != nullptr)
                        snprintf(driverSupportPath, MAXRBUF, "%s/Contents/Resources", getenv("INDIPREFIX"));
                    else
                        strncpy(driverSupportPath, "/usr/local/lib/indi", MAXRBUF);
                    int status = 0;
                    if(sbigCameraTypeFound == 1) // SBIG ST-7/8/9/10/2K cameras
                    {
                        strncat(driverSupportPath, "/DriverSupport/sbig/sbigucam.hex", MAXRBUF);
                        status = ezusb_load_ram(handle, driverSupportPath, FX_TYPE_FX1, IMG_TYPE_HEX, 0);
                    }
                    //Note that we NEED to add the code here to load sbigpcam.hex to ST-4K

                    if(sbigCameraTypeFound == 3) // SBIG ST-L cameras
                    {
                        strncat(driverSupportPath, "/DriverSupport/sbig/sbiglcam.hex", MAXRBUF);
                        status = ezusb_load_ram(handle, driverSupportPath, FX_TYPE_FX1, IMG_TYPE_HEX, 0);
                    }
                    if(sbigCameraTypeFound == 4) // SBIG ST-402/1603/3200/8300 cameras
                    {
                        strncat(driverSupportPath, "/DriverSupport/sbig/sbigfcam.hex", MAXRBUF);
                        status = ezusb_load_ram(handle, driverSupportPath, FX_TYPE_FX2, IMG_TYPE_HEX, 0);
                    }
                    if (status == 0 )
                        LOGF_DEBUG("Failed to load firmware", __FUNCTION__);
                    libusb_close(handle);
                }
            }
        }
    }
    libusb_free_device_list(list, 0);
    list = nullptr;
#endif
}

int SBIGCCD::OpenDriver()
{
    loadFirmwareOnOSXifNeeded();

    GetDriverHandleResults gdhr;
    SetDriverHandleParams sdhp;
    int res = ::SBIGUnivDrvCommand(CC_OPEN_DRIVER, nullptr, nullptr);
    if (res == CE_NO_ERROR)
    {
        LOGF_DEBUG("%s: CC_OPEN_DRIVER successful", __FUNCTION__);
        res = ::SBIGUnivDrvCommand(CC_GET_DRIVER_HANDLE, nullptr, &gdhr);
    }
    else if (res == CE_DRIVER_NOT_CLOSED)
    {
        LOGF_WARN("%s: CC_OPEN_DRIVER -> (%s)", __FUNCTION__, GetErrorString(res));
        // The driver is already open which we interpret as having been
        // opened by another instance of the class so get the driver to
        // allocate a new handle and then record it.
        sdhp.handle = INVALID_HANDLE_VALUE;
        res         = ::SBIGUnivDrvCommand(CC_SET_DRIVER_HANDLE, &sdhp, nullptr);
        if (res == CE_NO_ERROR)
        {
            res = ::SBIGUnivDrvCommand(CC_OPEN_DRIVER, nullptr, nullptr);
            if (res == CE_NO_ERROR)
            {
                res = ::SBIGUnivDrvCommand(CC_GET_DRIVER_HANDLE, nullptr, &gdhr);
            }
        }
    }
    if (res == CE_NO_ERROR)
    {
        SetDriverHandle(gdhr.handle);
    }
    else
    {
        LOGF_ERROR("%s: CC_OPEN_DRIVER -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

//==========================================================================

int SBIGCCD::CloseDriver()
{
    int res = ::SBIGUnivDrvCommand(CC_CLOSE_DRIVER, nullptr, nullptr);
    if (res == CE_NO_ERROR)
    {
        LOGF_DEBUG("%s: CC_CLOSE_DRIVER successful", __FUNCTION__);
        SetDriverHandle();
    }
    else
    {
        LOGF_ERROR("%s: CC_CLOSE_DRIVER -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

//==========================================================================

int SBIGCCD::OpenDevice(uint32_t devType)
{
    int res = CE_NO_ERROR;
    if (isSimulation())
    {
        return res;
    }
    OpenDeviceParams odp;
    if (IsDeviceOpen()) // Check if device already opened
        return (CE_NO_ERROR);
    odp.deviceType = devType; // Try to open new device
    if (devType == DEV_ETH)
    {
        unsigned long ip = htonl(inet_addr(IpTP.tp->text));
        if (ip == INADDR_NONE)
            return (CE_BAD_PARAMETER);
        odp.ipAddress = ip;
    }
    res = SBIGUnivDrvCommand(CC_OPEN_DEVICE, &odp, nullptr);
    if (res == CE_NO_ERROR)
    {
        //SetDeviceName(devType);
        SetFileDescriptor(true);
    }
    else
    {
        LOGF_ERROR("%s: CC_OPEN_DEVICE %d -> (%s)", __FUNCTION__, devType, GetErrorString(res));
    }
    return res;
}

//==========================================================================

int SBIGCCD::CloseDevice()
{
    int res = CE_NO_ERROR;
    if (isSimulation())
    {
        return res;
    }
    if (IsDeviceOpen())
    {
        if ((res = SBIGUnivDrvCommand(CC_CLOSE_DEVICE, nullptr, nullptr)) == CE_NO_ERROR)
        {
            SetFileDescriptor(); // set value to -1
            SetCameraType();     // set value to NO_CAMERA
        }
    }
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_CLOSE_DEVICE -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

//==========================================================================

SBIGCCD::SBIGCCD() : FilterInterface(this)
{
    InitVars();
    int res = OpenDriver();
    if (res != CE_NO_ERROR)
        LOGF_DEBUG("%s: Error (%s)", __FUNCTION__, GetErrorString(res));
    // TBD: For now let's set name to default name. In the future, we need to to support multiple devices per one driver
    if (*getDeviceName() == '\0')
        strncpy(name, getDefaultName(), MAXINDINAME);
    else
        strncpy(name, getDeviceName(), MAXINDINAME);

    setVersion(SBIG_VERSION_MAJOR, SBIG_VERSION_MINOR);
}

//==========================================================================

/*SBIGCCD::SBIGCCD(const char* devName) {
 int res = CE_NO_ERROR;
 InitVars();
 if ((res = OpenDriver()) == CE_NO_ERROR)
    OpenDevice(devName);
 if (res != CE_NO_ERROR)
   LOGF_DEBUG("%s: Error (%s)", __FUNCTION__, GetErrorString(res));
 }*/

//==========================================================================

SBIGCCD::~SBIGCCD()
{
    CloseDevice();
    CloseDriver();
}

//==========================================================================

const char *SBIGCCD::getDefaultName()
{
    return "SBIG CCD";
}

bool SBIGCCD::initProperties()
{
    INDI::CCD::initProperties();

    addSimulationControl();

    // CCD PRODUCT
    IUFillText(&ProductInfoT[0], "NAME", "Name", "");
    IUFillText(&ProductInfoT[1], "ID", "ID", "");
    IUFillTextVector(&ProductInfoTP, ProductInfoT, 2, getDeviceName(), "CCD_PRODUCT", "Product", MAIN_CONTROL_TAB,
                     IP_RO, 0, IPS_IDLE);

    // IP Address
    IUFillText(&IpT[0], "IP", "IP Address", "192.168.0.100");
    IUFillTextVector(&IpTP, IpT, 1, getDeviceName(), "IP_ADDRESS", "IP Address", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    // CCD DEVICE PORT
    IUFillSwitch(&PortS[0], "Ethernet", "Ethernet", ISS_OFF);
    SBIGPortMap[0] = DEV_ETH;
    PortS[0].aux   = &SBIGPortMap[0];
    IUFillSwitch(&PortS[1], "USB 1", "USB 1", ISS_ON);
    SBIGPortMap[1] = DEV_USB1;
    PortS[1].aux   = &SBIGPortMap[1];
    IUFillSwitch(&PortS[2], "USB 2", "USB 2", ISS_OFF);
    SBIGPortMap[2] = DEV_USB2;
    PortS[2].aux   = &SBIGPortMap[2];
    IUFillSwitch(&PortS[3], "USB 3", "USB 3", ISS_OFF);
    SBIGPortMap[3] = DEV_USB3;
    PortS[3].aux   = &SBIGPortMap[3];
    IUFillSwitch(&PortS[4], "USB 4", "USB 4", ISS_OFF);
    SBIGPortMap[4] = DEV_USB4;
    PortS[4].aux   = &SBIGPortMap[4];
    IUFillSwitch(&PortS[5], "LPT 1", "LPT 1", ISS_OFF);
    SBIGPortMap[5] = DEV_LPT1;
    PortS[5].aux   = &SBIGPortMap[5];
    IUFillSwitch(&PortS[6], "LPT 2", "LPT 2", ISS_OFF);
    SBIGPortMap[6] = DEV_LPT2;
    PortS[6].aux   = &SBIGPortMap[6];
    IUFillSwitch(&PortS[7], "LPT 3", "LPT 3", ISS_OFF);
    SBIGPortMap[7] = DEV_LPT3;
    PortS[7].aux   = &SBIGPortMap[7];
    IUFillSwitchVector(&PortSP, PortS, 8, getDeviceName(), "DEVICE_PORT_TYPE", "Port", MAIN_CONTROL_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_IDLE);

    // CCD FAN STATE
    IUFillSwitch(&FanStateS[0], "FAN_ON", "On", ISS_ON);
    IUFillSwitch(&FanStateS[1], "FAN_OFF", "Off", ISS_OFF);
    IUFillSwitchVector(&FanStateSP, FanStateS, 2, getDeviceName(), "CCD_FAN", "Fan", MAIN_CONTROL_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_OK);

    // CCD Cooler Switch
    IUFillSwitch(&CoolerS[0], "COOLER_ON", "On", ISS_OFF);
    IUFillSwitch(&CoolerS[1], "COOLER_OFF", "Off", ISS_ON);
    IUFillSwitchVector(&CoolerSP, CoolerS, 2, getDeviceName(), "CCD_COOLER", "Cooler", MAIN_CONTROL_TAB, IP_RW,
                       ISR_1OFMANY, 0, IPS_OK);

    // CCD COOLER:
    IUFillNumber(&CoolerN[0], "CCD_COOLER_VALUE", "[%]", "%.1f", 0, 0, 0, 0);
    IUFillNumberVector(&CoolerNP, CoolerN, 1, getDeviceName(), "CCD_COOLER_POWER", "Cooler %", MAIN_CONTROL_TAB, IP_RO,
                       0, IPS_IDLE);

    // Ignore errors
    IUFillSwitch(&IgnoreErrorsS[0], "SHUTTER_ERRORS", "Shutter Errors", ISS_OFF);
    IUFillSwitchVector(&IgnoreErrorsSP, IgnoreErrorsS, 1, getDeviceName(), "CCD_IGNORE_ERRORS", "Ignore", OPTIONS_TAB, IP_RW,
                       ISR_NOFMANY, 0, IPS_OK);

    // CFW PRODUCT
    IUFillText(&FilterProdcutT[0], "NAME", "Name", "");
    IUFillText(&FilterProdcutT[1], "ID", "ID", "");
    IUFillTextVector(&FilterProdcutTP, FilterProdcutT, 2, getDeviceName(), "CFW_PRODUCT", "Product", FILTER_TAB, IP_RO,
                     0, IPS_IDLE);

    // CFW_MODEL
    IUFillSwitch(&FilterTypeS[0], "CFW1", "CFW-2", ISS_OFF);
    FilterTypeS[0].aux = &SBIGFilterMap[0];
    SBIGFilterMap[0]   = CFWSEL_CFW2;
    IUFillSwitch(&FilterTypeS[1], "CFW2", "CFW-5", ISS_OFF);
    FilterTypeS[1].aux = &SBIGFilterMap[1];
    SBIGFilterMap[1]   = CFWSEL_CFW5;
    IUFillSwitch(&FilterTypeS[2], "CFW3", "CFW-6A", ISS_OFF);
    FilterTypeS[2].aux = &SBIGFilterMap[2];
    SBIGFilterMap[2]   = CFWSEL_CFW6A;
    IUFillSwitch(&FilterTypeS[3], "CFW4", "CFW-8", ISS_OFF);
    FilterTypeS[3].aux = &SBIGFilterMap[3];
    SBIGFilterMap[3]   = CFWSEL_CFW8;
    IUFillSwitch(&FilterTypeS[4], "CFW5", "CFW-402", ISS_OFF);
    FilterTypeS[4].aux = &SBIGFilterMap[4];
    SBIGFilterMap[4]   = CFWSEL_CFW402;
    IUFillSwitch(&FilterTypeS[5], "CFW6", "CFW-10", ISS_OFF);
    FilterTypeS[5].aux = &SBIGFilterMap[5];
    SBIGFilterMap[5]   = CFWSEL_CFW10;
    IUFillSwitch(&FilterTypeS[6], "CFW7", "CFW-10 SA", ISS_OFF);
    FilterTypeS[6].aux = &SBIGFilterMap[6];
    SBIGFilterMap[6]   = CFWSEL_CFW10_SERIAL;
    IUFillSwitch(&FilterTypeS[7], "CFW8", "CFW-L", ISS_OFF);
    FilterTypeS[7].aux = &SBIGFilterMap[7];
    SBIGFilterMap[7]   = CFWSEL_CFWL;
    IUFillSwitch(&FilterTypeS[8], "CFW9", "CFW-9", ISS_OFF);
    FilterTypeS[8].aux = &SBIGFilterMap[8];
    SBIGFilterMap[8]   = CFWSEL_CFW9;
    IUFillSwitch(&FilterTypeS[9], "CFW10", "CFW-8LG", ISS_OFF);
    FilterTypeS[9].aux = &SBIGFilterMap[9];
    SBIGFilterMap[9]   = CFWSEL_CFWL8G;
    IUFillSwitch(&FilterTypeS[10], "CFW11", "CFW-1603", ISS_OFF);
    FilterTypeS[10].aux = &SBIGFilterMap[10];
    SBIGFilterMap[10]   = CFWSEL_CFW1603;
    IUFillSwitch(&FilterTypeS[11], "CFW12", "CFW-FW5-STX", ISS_OFF);
    FilterTypeS[11].aux = &SBIGFilterMap[11];
    SBIGFilterMap[11]   = CFWSEL_FW5_STX;
    IUFillSwitch(&FilterTypeS[12], "CFW13", "CFW-FW5-8300", ISS_OFF);
    FilterTypeS[12].aux = &SBIGFilterMap[12];
    SBIGFilterMap[12]   = CFWSEL_FW5_8300;
    IUFillSwitch(&FilterTypeS[13], "CFW14", "CFW-FW8-8300", ISS_OFF);
    FilterTypeS[13].aux = &SBIGFilterMap[13];
    SBIGFilterMap[13]   = CFWSEL_FW8_8300;
    IUFillSwitch(&FilterTypeS[14], "CFW15", "CFW-FW7-STX", ISS_OFF);
    FilterTypeS[14].aux = &SBIGFilterMap[14];
    SBIGFilterMap[14]   = CFWSEL_FW7_STX;
    IUFillSwitch(&FilterTypeS[15], "CFW16", "CFW-FW8-STT", ISS_OFF);
    FilterTypeS[15].aux = &SBIGFilterMap[15];
    SBIGFilterMap[15]   = CFWSEL_FW8_STT;
#ifdef USE_CFW_AUTO
    IUFillSwitch(&FilterTypeS[16], "CFW17", "CFW-Auto", ISS_OFF);
    FilterTypeS[16].aux = &SBIGFilterMap[16];
    SBIGFilterMap[16]   = CFWSEL_AUTO;
#endif
    IUFillSwitchVector(&FilterTypeSP, FilterTypeS, MAX_CFW_TYPES, getDeviceName(), "CFW_TYPE", "Type", FILTER_TAB,
                       IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    // CFW CONNECTION
    IUFillSwitch(&FilterConnectionS[0], "CONNECT", "Connect", ISS_OFF);
    IUFillSwitch(&FilterConnectionS[1], "DISCONNECT", "Disconnect", ISS_ON);
    IUFillSwitchVector(&FilterConnectionSP, FilterConnectionS, 2, getDeviceName(), "CFW_CONNECTION", "Connect",
                       FILTER_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    /////////////////////////////////////////////////////////////////////////////
    /// Adaptive Optics
    /////////////////////////////////////////////////////////////////////////////
    IUFillNumber(&AONSN[AO_NORTH], "AO_N", "North (steps)", "%.f", 0, 2048, 100, 0);
    IUFillNumber(&AONSN[AO_SOUTH], "AO_S", "South (steps)", "%.f", 0, 2048, 100, 0);
    IUFillNumberVector(&AONSNP, AONSN, 2, getDeviceName(), "AO_NS", "AO Tilt North/South", GUIDE_CONTROL_TAB, IP_RW, 60,
                       IPS_IDLE);

    IUFillNumber(&AOWEN[AO_EAST], "AO_E", "East (steps)", "%.f", 0, 2048, 100, 0);
    IUFillNumber(&AOWEN[AO_WEST], "AO_W", "West (steps)", "%.f", 0, 2048, 100, 0);
    IUFillNumberVector(&AOWENP, AOWEN, 2, getDeviceName(), "AO_WE", "AO Tilt East/West", GUIDE_CONTROL_TAB, IP_RW, 60,
                       IPS_IDLE);

    IUFillSwitch(&CenterS[0], "CENTER", "Center", ISS_OFF);
    IUFillSwitchVector(&CenterSP, CenterS, 1, getDeviceName(), "AO_CENTER", "AO Center", GUIDE_CONTROL_TAB, IP_RW,
                       ISR_1OFMANY, 60, IPS_IDLE);


    BayerTP[2].setText("BGGR");

    INDI::FilterInterface::initProperties(FILTER_TAB);

    FilterSlotNP[0].setMin(1);
    FilterSlotNP[0].setMax(MAX_CFW_TYPES);

    // Set minimum exposure speed to 0.001 seconds
    PrimaryCCD.setMinMaxStep("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", 0.001, 3600, 1, false);

    setDriverInterface(getDriverInterface() | FILTER_INTERFACE | AO_INTERFACE);

    return true;
}

void SBIGCCD::ISGetProperties(const char *dev)
{
    INDI::CCD::ISGetProperties(dev);

    defineProperty(&PortSP);

    loadConfig(true, "DEVICE_PORT_TYPE");
    loadConfig(true, "IP_ADDRESS");

    addAuxControls();
}

bool SBIGCCD::updateProperties()
{
    // Set format first if connected.
    if (isConnected())
    {
        // N.B. AFAIK, there is no way to switch image formats.
        CaptureFormatSP.resize(0);
        CaptureFormat format;
        if (m_isColor)
        {
            format = {"INDI_RAW", "RAW", 16, true};
        }
        else
        {
            format = {"INDI_MONO", "Mono", 16, true};
        }
        addCaptureFormat(format);
    }

    INDI::CCD::updateProperties();
    if (isConnected())
    {
        defineProperty(&ProductInfoTP);
        if (IsFanControlAvailable())
        {
            defineProperty(&FanStateSP);
        }
        if (HasCooler())
        {
            defineProperty(&CoolerSP);
            defineProperty(&CoolerNP);
        }
        defineProperty(&IgnoreErrorsSP);
        if (m_hasFilterWheel)
        {
            defineProperty(&FilterConnectionSP);
            defineProperty(&FilterTypeSP);
        }

        // AO Properties
        if (m_hasAO)
        {
            defineProperty(&AONSNP);
            defineProperty(&AOWENP);
            defineProperty(&CenterSP);
        }

        setupParams();

        if (m_hasFilterWheel) // If filter type already selected (from config file), then try to connect to CFW
        {
            loadConfig(true, "CFW_TYPE");
            ISwitch *p = IUFindOnSwitch(&FilterTypeSP);
            if (p != nullptr && FilterConnectionS[0].s == ISS_OFF)
            {
                LOG_DEBUG("Filter type is already selected and filter is not connected. Will "
                          "attempt to connect to filter now...");
                CFWConnect();
            }
        }
        m_TimerID = SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(ProductInfoTP.name);
        if (IsFanControlAvailable())
        {
            deleteProperty(FanStateSP.name);
        }
        if (HasCooler())
        {
            deleteProperty(CoolerSP.name);
            deleteProperty(CoolerNP.name);
        }
        deleteProperty(IgnoreErrorsSP.name);

        if (m_hasAO)
        {
            deleteProperty(AONSNP.name);
            deleteProperty(AOWENP.name);
            deleteProperty(CenterSP.name);
        }

        if (m_hasFilterWheel)
        {
            deleteProperty(FilterConnectionSP.name);
            deleteProperty(FilterTypeSP.name);
            deleteProperty(FilterProdcutTP.name);
            deleteProperty(FilterNameTP);
        }
        rmTimer(m_TimerID);
    }
    return true;
}

bool SBIGCCD::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // IP Address
        if (strcmp(name, IpTP.name) == 0)
        {
            unsigned long ip = htonl(inet_addr(texts[0]));
            if (ip == INADDR_NONE)
            {
                LOGF_ERROR("Invalid ip address %s.", texts[0]);
                IpTP.s = IPS_ALERT;
                IDSetText(&IpTP, nullptr);
                return false;
            }
            IpTP.s = IPS_OK;
            IUUpdateText(&IpTP, texts, names, n);
            IDSetText(&IpTP, nullptr);
            return true;
        }
        // Filter Name
        else if (strcmp(name, FilterNameTP.getName()) == 0)
        {
            INDI::FilterInterface::processText(dev, name, texts, names, n);
            return true;
        }
    }
    return INDI::CCD::ISNewText(dev, name, texts, names, n);
}

bool SBIGCCD::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Device Port
        if (strcmp(name, PortSP.name) == 0)
        {
            IUUpdateSwitch(&PortSP, states, names, n);
            if (*(static_cast<uint32_t *>(IUFindOnSwitch(&PortSP)->aux)) == DEV_ETH)
            {
                defineProperty(&IpTP);
            }
            else
            {
                deleteProperty(IpTP.name);
            }
            PortSP.s = IPS_OK;
            IDSetSwitch(&PortSP, nullptr);
            return true;
        }
        // Fan Status
        else if (strcmp(name, FanStateSP.name) == 0)
        {
            IUUpdateSwitch(&FanStateSP, states, names, n);
            MiscellaneousControlParams mcp;
            mcp.fanEnable      = FanStateS[0].s == ISS_ON;
            mcp.shutterCommand = SC_LEAVE_SHUTTER;
            mcp.ledState       = LED_OFF;
            if (MiscellaneousControl(&mcp) == CE_NO_ERROR)
            {
                FanStateSP.s = IPS_OK;
                IDSetSwitch(&FanStateSP, mcp.fanEnable == 1 ? "Fan turned On." : "Fan turned Off.");
                return true;
            }
            FanStateSP.s = IPS_ALERT;
            LOG_ERROR("Failed to control fan.");
            IDSetSwitch(&FanStateSP, nullptr);
            return false;
        }
        // Filter Type
        else if (strcmp(name, FilterTypeSP.name) == 0) // CFW TYPE
        {
            IUResetSwitch(&FilterTypeSP);
            IUUpdateSwitch(&FilterTypeSP, states, names, n);
            FilterTypeSP.s = IPS_OK;
            IDSetSwitch(&FilterTypeSP, nullptr);
            return true;
        }
        // Cooler control
        else if (strcmp(name, CoolerSP.name) == 0)
        {
            IUUpdateSwitch(&CoolerSP, states, names, n);
            bool coolerON = CoolerS[0].s == ISS_ON;
            if (SetTemperatureRegulation(TemperatureNP[0].getValue(), coolerON) == CE_NO_ERROR)
            {
                CoolerSP.s = coolerON ? IPS_OK : IPS_IDLE;
                LOGF_INFO("Cooler turned %s.", coolerON ? "On" : "Off");
                IDSetSwitch(&CoolerSP, nullptr);

                return true;
            }

            CoolerSP.s = IPS_ALERT;
            LOG_ERROR("Failed to control cooler.");
            IDSetSwitch(&CoolerSP, nullptr);
            return false;
        }
        // AO Center
        else if (!strcmp(name, CenterSP.name))
        {
            CenterSP.s = (AoCenter() == CE_NO_ERROR) ? IPS_OK : IPS_ALERT;
            if (CenterSP.s == IPS_OK)
            {
                m_AOParams.xDeflection = m_AOParams.yDeflection = 0;
                AONSN[AO_NORTH].value = AONSN[AO_SOUTH].value = 0;
                AONSNP.s = IPS_IDLE;
                AOWEN[AO_EAST].value = AOWEN[AO_WEST].value = 0;
                AOWENP.s = IPS_IDLE;
                IDSetNumber(&AONSNP, nullptr);
                IDSetNumber(&AOWENP, nullptr);

                LOG_INFO("Adaptive Optics are centered.");
            }
            else
                LOG_ERROR("Failed to center adaptive optics.");

            IDSetSwitch(&CenterSP, nullptr);
            return true;
        }
        // Ignore errors
        else if (!strcmp(name, IgnoreErrorsSP.name))
        {
            IUUpdateSwitch(&IgnoreErrorsSP, states, names, n);
            IgnoreErrorsSP.s = IPS_OK;
            IDSetSwitch(&IgnoreErrorsSP, nullptr);
            saveConfig(true);
            return true;
        }
        // Filter connection
        else if (!strcmp(name, FilterConnectionSP.name))
        {
            IUUpdateSwitch(&FilterConnectionSP, states, names, n);
            FilterConnectionSP.s = IPS_BUSY;
            if (FilterConnectionS[0].s == ISS_ON)
            {
                ISwitch *p = IUFindOnSwitch(&FilterTypeSP);
                if (p == nullptr)
                {
                    FilterConnectionSP.s = IPS_ALERT;
                    IUResetSwitch(&FilterConnectionSP);
                    FilterConnectionSP.sp[1].s = ISS_ON;
                    IDSetSwitch(&FilterConnectionSP, "Please select CFW type before connecting");
                    return false;
                }
                CFWConnect();
            }
            else
            {
                CFWDisconnect();
            }
            return true;
        }
    }
    return INDI::CCD::ISNewSwitch(dev, name, states, names, n);
}

bool SBIGCCD::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Filter Slot Handling
        if (INDI::FilterInterface::processNumber(dev, name, values, names, n))
            return true;

        // NS Adaptive Optics
        if (!strcmp(name, AONSNP.name))
        {
            IUUpdateNumber(&AONSNP, values, names, n);
            uint16_t deflection = 0;

            // Check if N > 0 then if it is
            // reset S to zero regardless.
            if (AONSN[AO_NORTH].value > 0)
            {
                AONSN[AO_SOUTH].value = 0;
                deflection = std::min(4095.0, 2048 + AONSN[AO_NORTH].value);
                LOGF_DEBUG("AO North: %.f --> yDeflection: %d", AONSN[AO_NORTH].value, deflection);
            }
            // Same as above but reversed.
            else
            {
                AONSN[AO_NORTH].value = 0;
                deflection = std::max(0.0, 2048 - AONSN[AO_SOUTH].value);
                LOGF_DEBUG("AO South: %.f --> yDeflection: %d", AONSN[AO_SOUTH].value, deflection);
            }

            // Just change yDeflection
            m_AOParams.yDeflection = deflection;
            AONSNP.s = (AoTipTilt() == CE_NO_ERROR) ? IPS_OK : IPS_ALERT;
            IDSetNumber(&AONSNP, nullptr);
            return true;
        }
        // WE Adaptive Optiocs
        else if (!strcmp(name, AOWENP.name))
        {
            IUUpdateNumber(&AOWENP, values, names, n);
            uint16_t deflection = 0;

            // Check if E > 0 then if it is
            // reset W to zero regardless.
            if (AOWEN[AO_EAST].value > 0)
            {
                AOWEN[AO_WEST].value = 0;
                deflection = std::min(4095.0, 2048 + AOWEN[AO_EAST].value);
                LOGF_DEBUG("AO East: %.f --> xDeflection: %d", AOWEN[AO_EAST].value, deflection);
            }
            // Same as above but reversed.
            else
            {
                AOWEN[AO_EAST].value = 0;
                deflection = std::max(0.0, 2048 - AOWEN[AO_WEST].value);
                LOGF_DEBUG("AO West: %.f --> xDeflection: %d", AOWEN[AO_WEST].value, deflection);
            }

            // Just change xDeflection
            m_AOParams.xDeflection = deflection;
            AOWENP.s = (AoTipTilt() == CE_NO_ERROR) ? IPS_OK : IPS_ALERT;
            IDSetNumber(&AOWENP, nullptr);
            return true;
        }
    }
    return INDI::CCD::ISNewNumber(dev, name, values, names, n);
}

bool SBIGCCD::Connect()
{
    int numModes = -1;
    int maxBinX  = 1;
    int maxBinY  = 1;
    int res = CE_NO_ERROR;

    loadFirmwareOnOSXifNeeded();

    if (isConnected())
    {
        LOG_DEBUG("CCD device already connected");
        return true;
    }
    m_hasGuideHead     = false;
    m_hasFilterWheel   = false;
    uint32_t devType = *(static_cast<uint32_t *>(IUFindOnSwitch(&PortSP)->aux));
    char *port       = IUFindOnSwitch(&PortSP)->label;
    if (OpenDevice(devType) != CE_NO_ERROR)
    {
        LOGF_ERROR("Failed to open CCD at port %s", port);
        return false;
    }

    if (EstablishLink() != CE_NO_ERROR)
    {
        LOGF_ERROR("Failed to connect CCD at port %s", port);
        return false;
    }

    LOGF_INFO("CCD is connected at port %s", port);

    if (GetExtendedCCDInfo() != CE_NO_ERROR)
    {
        LOG_ERROR("Failed to get extended CCD info.");
        return false;
    }

    uint32_t cap = CCD_CAN_ABORT | CCD_CAN_BIN | CCD_CAN_SUBFRAME | CCD_HAS_SHUTTER | CCD_HAS_ST4_PORT;
    if (m_hasGuideHead)
        cap |= CCD_HAS_GUIDE_HEAD;
    if (m_isColor)
        cap |= CCD_HAS_BAYER;
    if (GetCameraType() != STI_CAMERA)
    {
        cap |= CCD_HAS_COOLER;
        IEAddTimer(TEMPERATURE_POLL_MS, SBIGCCD::updateTemperatureHelper, this);
    }

    res = getReadoutModes(&PrimaryCCD, numModes, maxBinX, maxBinY);
    if (res != CE_NO_ERROR || numModes < CCD_BIN_1x1_I || numModes > CCD_BIN_NxN_I)
    {
        LOG_ERROR("Failed to determine number of supported readout modes for primary CCD");
        return false;
    }
    PrimaryCCD.setMinMaxStep("CCD_BINNING", "HOR_BIN", 1, maxBinX, 1, false);
    PrimaryCCD.setMinMaxStep("CCD_BINNING", "VER_BIN", 1, maxBinY, 1, false);

    if (m_hasGuideHead)
    {
        res = getReadoutModes(&GuideCCD, numModes, maxBinX, maxBinY);
        if (res != CE_NO_ERROR || numModes < CCD_BIN_1x1_I || numModes > CCD_BIN_NxN_I)
        {
            LOG_ERROR("Failed to determine number of supported readout modes for primary CCD");
            return false;
        }
        GuideCCD.setMinMaxStep("CCD_BINNING", "HOR_BIN", 1, maxBinX, 1, false);
        GuideCCD.setMinMaxStep("CCD_BINNING", "VER_BIN", 1, maxBinY, 1, false);
    }

    SetCCDCapability(cap);

    m_hasAO = AoCenter() == CE_NO_ERROR;

    return true;
}

bool SBIGCCD::Disconnect()
{
    if (!isConnected())
        return true;
    m_useExternalTrackingCCD = false;
    m_hasGuideHead           = false;
#ifdef ASYNC_READOUT
    pthread_mutex_lock(&condMutex);
    grabPredicate   = GRAB_PRIMARY_CCD;
    terminateThread = true;
    pthread_cond_signal(&cv);
    pthread_mutex_unlock(&condMutex);
#endif
    if (FilterConnectionS[0].s == ISS_ON)
        CFWDisconnect();
    if (CloseDevice() == CE_NO_ERROR)
    {
        LOG_INFO("CCD is disconnected");
        return true;
    }
    else
    {
        LOG_ERROR("Failed to disconnect CCD");
    }
    return false;
}

bool SBIGCCD::setupParams()
{
    LOG_DEBUG("Retrieving CCD Parameters...");
    float x_pixel_size, y_pixel_size;
    int bit_depth = 16;
    int x_1, y_1, x_2, y_2;
    int wCcd = 0, hCcd = 0, binning = 0;
    double wPixel = 0, hPixel = 0;

    if (getBinningMode(&PrimaryCCD, binning) != CE_NO_ERROR)
    {
        LOG_ERROR("Failed to get primary camera binning mode");
        return false;
    }
    if (getCCDSizeInfo(CCD_IMAGING, binning, wCcd, hCcd, wPixel, hPixel) != CE_NO_ERROR)
    {
        LOG_ERROR("Failed to get primary camera size info");
        return false;
    }

    x_pixel_size = wPixel;
    y_pixel_size = hPixel;
    x_1 = y_1 = 0;
    x_2       = wCcd;
    y_2       = hCcd;
    SetCCDParams(x_2 - x_1, y_2 - y_1, bit_depth, x_pixel_size, y_pixel_size);

    if (HasGuideHead())
    {
        if (getBinningMode(&GuideCCD, binning) != CE_NO_ERROR)
        {
            LOG_ERROR("Failed to get guide head binning mode");
            return false;
        }
        if (getCCDSizeInfo(m_useExternalTrackingCCD ? CCD_EXT_TRACKING : CCD_TRACKING, binning, wCcd, hCcd, wPixel,
                           hPixel) != CE_NO_ERROR)
        {
            LOG_DEBUG("Invalid external tracking camera results, trying regular tracking");
            if (getCCDSizeInfo(CCD_TRACKING, binning, wCcd, hCcd, wPixel, hPixel) != CE_NO_ERROR)
            {
                LOG_ERROR("Failed to get external tracking camera size info");
                return false;
            }
            m_useExternalTrackingCCD = false;
        }

        x_pixel_size = wPixel;
        y_pixel_size = hPixel;
        x_1 = y_1 = 0;
        x_2       = wCcd;
        y_2       = hCcd;
        SetGuiderParams(x_2 - x_1, y_2 - y_1, bit_depth, x_pixel_size, y_pixel_size);
    }

    int nbuf = PrimaryCCD.getXRes() * PrimaryCCD.getYRes() * PrimaryCCD.getBPP() / 8 + 512;
    PrimaryCCD.setFrameBufferSize(nbuf);
    if (PrimaryCCD.getFrameBuffer() == nullptr)
    {
        LOG_ERROR("Failed to allocate memory for primary camera buffer");
        return false;
    }
    LOGF_DEBUG("Created primary camera buffer %d bytes.", nbuf);

    if (HasGuideHead())
    {
        nbuf = GuideCCD.getXRes() * GuideCCD.getYRes() * GuideCCD.getBPP() / 8 + 512;
        GuideCCD.setFrameBufferSize(nbuf);
        if (GuideCCD.getFrameBuffer() == nullptr)
        {
            LOG_ERROR("Failed to allocate memory for guide head buffer");
            return false;
        }
        LOGF_DEBUG("Created guide head buffer %d bytes.", nbuf);
    }

    if (HasCooler())
    {
        bool regulationEnabled = false;
        double temp, setPoint, power;
        QueryTemperatureStatus(regulationEnabled, temp, setPoint, power);
        CoolerS[0].s = regulationEnabled ? ISS_ON : ISS_OFF;
        CoolerS[1].s = regulationEnabled ? ISS_OFF : ISS_ON;
        IDSetSwitch(&CoolerSP, nullptr);
        CoolerN[0].value = power * 100;
        IDSetNumber(&CoolerNP, nullptr);
        TemperatureNP[0].setMin(MIN_CCD_TEMP);
        TemperatureNP[0].setMax(MAX_CCD_TEMP);
        TemperatureNP.updateMinMax();
    }

    IUSaveText(&ProductInfoT[0], GetCameraName());
    IUSaveText(&ProductInfoT[1], GetCameraID());
    ProductInfoTP.s = IPS_OK;
    IDSetText(&ProductInfoTP, nullptr);
    return true;
}

int SBIGCCD::SetTemperature(double temperature)
{
    if (fabs(temperature - TemperatureNP[0].getValue()) < 0.1)
        return 1;
    if (SetTemperatureRegulation(temperature) == CE_NO_ERROR)
    {
        // Set property to busy and poll in ISPoll for CCD temp
        TemperatureRequest = temperature;
        LOGF_INFO("Temperature set to %+.1fC", temperature);
        if (CoolerS[0].s != ISS_ON)
        {
            CoolerS[0].s = ISS_ON;
            CoolerS[1].s = ISS_OFF;
            CoolerSP.s   = IPS_BUSY;
            IDSetSwitch(&CoolerSP, nullptr);
        }
        return 0;
    }
    LOG_ERROR("Failed to set temperature");
    return -1;
}

int SBIGCCD::StartExposure(INDI::CCDChip *targetChip, double duration)
{
    int res, binning, shutter;

    if ((res = getShutterMode(targetChip, shutter)) != CE_NO_ERROR)
    {
        return res;
    }
    if ((res = getBinningMode(targetChip, binning)) != CE_NO_ERROR)
    {
        return res;
    }

    INDI::CCDChip::CCD_FRAME frameType;
    getFrameType(targetChip, &frameType);

    uint32_t expTime = floor(duration * 100.0 + 0.5);
    if (frameType == INDI::CCDChip::BIAS_FRAME)
    {
        // Flat frame = zero seconds
        expTime = 0;
    }

    uint16_t left   = targetChip->getSubX();
    uint16_t top    = targetChip->getSubY();
    uint16_t width  = targetChip->getSubW() / targetChip->getBinX();
    uint16_t height = targetChip->getSubH() / targetChip->getBinY();

    int ccd;
    if (targetChip == &PrimaryCCD)
    {
        ccd = CCD_IMAGING;
    }
    else
    {
        ccd = m_useExternalTrackingCCD ? CCD_EXT_TRACKING : CCD_TRACKING;
    }

    StartExposureParams2 sep;
    sep.ccd          = ccd;
    sep.abgState     = ABG_LOW7;
    sep.openShutter  = shutter;
    sep.exposureTime = expTime;
    sep.readoutMode  = binning;
    sep.left         = left;
    sep.top          = top;
    sep.width        = width;
    sep.height       = height;

    LOGF_DEBUG(
        "Exposure params for CCD (%d) openShutter(%d), exposureTime (%ld), binning (%d), left (%d), top (%d), w(%d), "
        "h(%d)",
        sep.ccd, sep.openShutter, sep.exposureTime, sep.readoutMode, sep.left, sep.top, sep.width, sep.height);

    for (int i = 0; i < MAX_THREAD_RETRIES; i++)
    {
        std::unique_lock<std::mutex> guard(sbigLock);
        res = StartExposure(&sep);
        guard.unlock();
        if (res == CE_NO_ERROR)
        {
            targetChip->setExposureDuration(duration);
            break;
        }
        usleep(MAX_THREAD_WAIT);
    }

    if (res != CE_NO_ERROR)
    {
        return res;
    }

    if (frameType == INDI::CCDChip::LIGHT_FRAME)
    {
        LOG_DEBUG("Light Frame exposure in progress...");
    }
    else if (frameType == INDI::CCDChip::DARK_FRAME)
    {
        LOG_DEBUG("Dark Frame exposure in progress...");
    }
    else if (frameType == INDI::CCDChip::FLAT_FRAME)
    {
        LOG_DEBUG("Flat Frame exposure in progress...");
    }
    else if (frameType == INDI::CCDChip::BIAS_FRAME)
    {
        LOG_DEBUG("Bias Frame exposure in progress...");
    }
    return res;
}

bool SBIGCCD::StartExposure(float duration)
{
    ExposureRequest = duration;

    if (duration >= 3)
        LOGF_INFO("Taking %.2fs exposure on main camera...", ExposureRequest);

    int res = StartExposure(&PrimaryCCD, duration);
    if (res != CE_NO_ERROR)
    {
        LOG_DEBUG("Failed to start exposure on main camera");
        return false;
    }

    ExpStart = std::chrono::system_clock::now();
    InExposure = true;
    return true;
}

bool SBIGCCD::StartGuideExposure(float duration)
{
    GuideExposureRequest = duration;

    if (duration >= 3)
        LOGF_INFO("Taking %.2fs exposure on guide head...", GuideExposureRequest);

    int res = StartExposure(&GuideCCD, duration);
    if (res != CE_NO_ERROR)
    {
        LOG_DEBUG("Failed to start exposure on guide head");
        return false;
    }

    GuideExpStart = std::chrono::system_clock::now();
    InGuideExposure = true;
    return true;
}

int SBIGCCD::AbortExposure(INDI::CCDChip *targetChip)
{
    int ccd;
    if (targetChip == &PrimaryCCD)
    {
        ccd = CCD_IMAGING;
    }
    else
    {
        ccd = m_useExternalTrackingCCD ? CCD_EXT_TRACKING : CCD_TRACKING;
    }
    EndExposureParams eep;
    eep.ccd = ccd;
    std::unique_lock<std::mutex> guard(sbigLock);
    int res = EndExposure(&eep);
    guard.unlock();
    return res;
}

bool SBIGCCD::AbortExposure()
{
    int res = CE_NO_ERROR;
    LOG_DEBUG("Aborting primary camera exposure...");
    for (int i = 0; i < MAX_THREAD_RETRIES; i++)
    {
        res = AbortExposure(&PrimaryCCD);
        if (res == CE_NO_ERROR)
        {
            break;
        }
        usleep(MAX_THREAD_WAIT);
    }
    if (res != CE_NO_ERROR)
    {
        LOG_ERROR("Failed to abort primary camera exposure");
        return false;
    }
    InExposure = false;
    LOG_DEBUG("Primary camera exposure aborted");
    return true;
}

bool SBIGCCD::AbortGuideExposure()
{
    int res = CE_NO_ERROR;
    LOG_DEBUG("Aborting guide head exposure...");
    for (int i = 0; i < MAX_THREAD_RETRIES; i++)
    {
        res = AbortExposure(&GuideCCD);
        if (res == CE_NO_ERROR)
        {
            break;
        }
        usleep(MAX_THREAD_WAIT);
    }
    if (res != CE_NO_ERROR)
    {
        LOG_ERROR("Failed to abort guide head exposure");
        return false;
    }
    InExposure = false;
    LOG_DEBUG("Guide head exposure aborted");
    return true;
}

bool SBIGCCD::UpdateCCDFrameType(INDI::CCDChip::CCD_FRAME fType)
{
    INDI::CCDChip::CCD_FRAME imageFrameType = PrimaryCCD.getFrameType();
    if (fType != imageFrameType)
    {
        PrimaryCCD.setFrameType(fType);
    }
    return true;
}

bool SBIGCCD::updateFrameProperties(INDI::CCDChip *targetChip)
{
    int wCcd, hCcd, binning;
    double wPixel, hPixel;

    LOG_DEBUG("Updating frame properties ...");
    int res = getBinningMode(targetChip, binning);
    if (res != CE_NO_ERROR)
    {
        return false;
    }
    if (targetChip == &PrimaryCCD)
    {
        res = getCCDSizeInfo(CCD_IMAGING, binning, wCcd, hCcd, wPixel, hPixel);
    }
    else
    {
        res = getCCDSizeInfo(m_useExternalTrackingCCD ? CCD_EXT_TRACKING : CCD_TRACKING, binning, wCcd, hCcd, wPixel,
                             hPixel);
    }
    if (res == CE_NO_ERROR)
    {
        // SBIG returns binned width and height, which is OK, but we used unbinned width and height across all drivers to be consistent.
        wCcd *= targetChip->getBinX();
        hCcd *= targetChip->getBinY();
        targetChip->setResolution(wCcd, hCcd);
        if (targetChip == &PrimaryCCD)
        {
            UpdateCCDFrame(0, 0, wCcd, hCcd);
        }
        else
        {
            UpdateGuiderFrame(0, 0, wCcd, hCcd);
        }
        return true;
    }
    LOG_DEBUG("Failed to update frame properties");
    return false;
}

bool SBIGCCD::UpdateCCDFrame(int x, int y, int w, int h)
{
    LOGF_DEBUG("The final main camera image area is (%ld, %ld), (%ld, %ld)", x, y, w, h);
    PrimaryCCD.setFrame(x, y, w, h);
    int nbuf = (w * h * PrimaryCCD.getBPP() / 8) + 512;
    PrimaryCCD.setFrameBufferSize(nbuf);
    LOGF_DEBUG("Created primary camera buffer %d bytes", nbuf);
    return true;
}

bool SBIGCCD::UpdateGuiderFrame(int x, int y, int w, int h)
{
    LOGF_DEBUG("The final guide head image area is (%ld, %ld), (%ld, %ld)", x, y, w, h);
    GuideCCD.setFrame(x, y, w, h);
    int nbuf = (w * h * GuideCCD.getBPP() / 8) + 512;
    GuideCCD.setFrameBufferSize(nbuf);
    LOGF_DEBUG("Created guide head buffer %d bytes", nbuf);
    return true;
}

bool SBIGCCD::UpdateCCDBin(int binx, int biny)
{
    // only basic sanity checks; if the camera really supports the requested binning
    // mode is checked in getBinningMode
    if (binx > 255 || biny > 255)
    {
        LOG_ERROR("Failed to update main camera binning mode, binning should be at least < 256");
        return false;
    }
    if (binx > 3 && biny != binx)
    {
        LOG_WARN("Forcing y-binning = x-binning while updating main camera binning mode");
        biny = binx;
    }
    PrimaryCCD.setBin(binx, biny);
    return updateFrameProperties(&PrimaryCCD);
}

bool SBIGCCD::UpdateGuiderBin(int binx, int biny)
{
    if (binx != biny)
    {
        LOG_WARN("Forcing y-binning = x-binning while updating guide head binning mode");
        biny = binx;
    }
    if (binx < 1 || binx > 3)
    {
        LOG_ERROR("Failed to update guide head binning mode, use 1x1, 2x2 or 3x3");
        return false;
    }
    GuideCCD.setBin(binx, biny);
    return updateFrameProperties(&GuideCCD);
}

void SBIGCCD::NSGuideHelper(void *context)
{
    static_cast<SBIGCCD *>(context)->NSGuideCallback();
}

void SBIGCCD::WEGuideHelper(void *context)
{
    static_cast<SBIGCCD *>(context)->WEGuideCallback();
}

void SBIGCCD::NSGuideCallback()
{
    rp.tYMinus = rp.tYPlus = 0;
    ActivateRelay(&rp);
}

void SBIGCCD::WEGuideCallback()
{
    rp.tXMinus = rp.tXPlus = 0;
    rp.tXPlus = 0;
    ActivateRelay(&rp);
}

IPState SBIGCCD::GuideNorth(uint32_t ms)
{
    rmTimer(m_NSTimerID);
    rp.tYMinus = rp.tYPlus = 0;
    rp.tYMinus = ms / 10.0;

    m_NSTimerID = IEAddTimer(ms, &SBIGCCD::NSGuideHelper, this);

    return (ActivateRelay(&rp) == CE_NO_ERROR ? IPS_BUSY : IPS_ALERT);
}

IPState SBIGCCD::GuideSouth(uint32_t ms)
{
    rmTimer(m_NSTimerID);
    rp.tYMinus = rp.tYPlus = 0;
    rp.tYPlus = ms / 10.0;

    m_NSTimerID = IEAddTimer(ms, &SBIGCCD::NSGuideHelper, this);

    return (ActivateRelay(&rp) == CE_NO_ERROR ? IPS_BUSY : IPS_ALERT);
}

IPState SBIGCCD::GuideEast(uint32_t ms)
{
    rmTimer(m_WETimerID);
    rp.tXMinus = rp.tXPlus = 0;
    rp.tXPlus = ms / 10.0;

    m_WETimerID = IEAddTimer(ms, &SBIGCCD::WEGuideHelper, this);

    return (ActivateRelay(&rp) == CE_NO_ERROR ? IPS_BUSY : IPS_ALERT);
}

IPState SBIGCCD::GuideWest(uint32_t ms)
{
    rmTimer(m_WETimerID);
    rp.tXMinus = rp.tXPlus = 0;
    rp.tXMinus = ms / 10.0;

    m_WETimerID = IEAddTimer(ms, &SBIGCCD::WEGuideHelper, this);

    return (ActivateRelay(&rp) == CE_NO_ERROR ? IPS_BUSY : IPS_ALERT);
}

#ifdef ASYNC_READOUT
void *SBIGCCD::grabCCDHelper(void *context)
{
    return ((SBIGCCD *)context)->grabCCD();
}

void *SBIGCCD::grabCCD()
{
    LOG_DEBUG("grabCCD thread started...");
    INDI::CCDChip *targetChip = nullptr;
    pthread_mutex_lock(&condMutex);
    while (true)
    {
        while (grabPredicate == GRAB_NO_CCD)
        {
            pthread_cond_wait(&cv, &condMutex);
        }
        targetChip    = (grabPredicate == GRAB_PRIMARY_CCD) ? &PrimaryCCD : &GuideCCD;
        grabPredicate = GRAB_NO_CCD;
        if (terminateThread)
            break;
        pthread_mutex_unlock(&condMutex);
        if (grabImage(targetChip) == false)
        {
            targetChip->setExposureFailed();
        }
        pthread_mutex_lock(&condMutex);
    }
    pthread_mutex_unlock(&condMutex);
    LOG_DEBUG("grabCCD thread finished");
    return 0;
}
#endif

bool SBIGCCD::grabImage(INDI::CCDChip *targetChip)
{
    uint16_t left   = targetChip->getSubX() / targetChip->getBinX();
    uint16_t top    = targetChip->getSubY() / targetChip->getBinX();
    uint16_t width  = targetChip->getSubW() / targetChip->getBinX();
    uint16_t height = targetChip->getSubH() / targetChip->getBinY();

    LOGF_DEBUG("%s readout in progress...", targetChip == &PrimaryCCD ? "Primary camera" : "Guide head");

    if (isSimulation())
    {
        uint8_t *image = targetChip->getFrameBuffer();
        for (int i = 0; i < height * 2; i++)
        {
            for (int j = 0; j < width; j++)
            {
                image[i * width + j] = rand() % 255;
            }
        }
    }
    else
    {
        uint16_t *buffer = reinterpret_cast<uint16_t *>(targetChip->getFrameBuffer());
        int res                = 0;
        for (int i = 0; i < MAX_THREAD_RETRIES; i++)
        {
            res = readoutCCD(left, top, width, height, buffer, targetChip);
            if (res == CE_NO_ERROR)
                break;
            LOGF_DEBUG("Readout error, retrying...", res);
            usleep(MAX_THREAD_WAIT);
        }
        if (res != CE_NO_ERROR)
        {
            LOGF_ERROR("%s readout error",
                       targetChip == &PrimaryCCD ? "Primary camera" : "Guide head");
            return false;
        }
    }
    LOGF_DEBUG("%s readout complete", targetChip == &PrimaryCCD ? "Primary camera" : "Guide head");
    ExposureComplete(targetChip);
    return true;
}

bool SBIGCCD::saveConfigItems(FILE *fp)
{
    INDI::CCD::saveConfigItems(fp);

    IUSaveConfigSwitch(fp, &PortSP);
    IUSaveConfigText(fp, &IpTP);
    IUSaveConfigSwitch(fp, &IgnoreErrorsSP);

    INDI::FilterInterface::saveConfigItems(fp);

    IUSaveConfigSwitch(fp, &FilterTypeSP);
    return true;
}

void SBIGCCD::TimerHit()
{
    INDI::CCDChip *targetChip = nullptr;

    if (isConnected() == false)
    {
        return;
    }

    if (InExposure)
    {
        targetChip = &PrimaryCCD;
        std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - ExpStart;
        double timeLeft = std::max(0.0, ExposureRequest - elapsed.count());
        if (isExposureDone(targetChip))
        {
            LOG_DEBUG("Primay camera exposure done, downloading image...");
            targetChip->setExposureLeft(0);
            InExposure = false;
            if (grabImage(targetChip) == false)
                targetChip->setExposureFailed();
        }
        else
        {
            targetChip->setExposureLeft(timeLeft);
            LOGF_DEBUG("Primary camera exposure in progress with %.2f seconds left...", timeLeft);
        }
    }

    if (InGuideExposure)
    {
        targetChip = &GuideCCD;
        std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - GuideExpStart;
        double timeLeft = std::max(0.0, GuideExposureRequest - elapsed.count());
        if (isExposureDone(targetChip))
        {
            LOG_DEBUG("Guide head exposure done, downloading image...");
            targetChip->setExposureLeft(0);
            InGuideExposure = false;
            if (grabImage(targetChip) == false)
                targetChip->setExposureFailed();
        }
        else
        {
            targetChip->setExposureLeft(timeLeft);
            LOGF_DEBUG("Guide head exposure in progress with %.2f seconds left...", timeLeft);
        }
    }

    SetTimer(getCurrentPollingPeriod());
    return;
}

//=========================================================================

int SBIGCCD::GetDriverInfo(GetDriverInfoParams *gdip, void *res)
{
    return SBIGUnivDrvCommand(CC_GET_DRIVER_INFO, gdip, res);
}

int SBIGCCD::SetDriverHandle(SetDriverHandleParams *sdhp)
{
    int res = SBIGUnivDrvCommand(CC_SET_DRIVER_HANDLE, sdhp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_SET_DRIVER_HANDLE -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::GetDriverHandle(GetDriverHandleResults *gdhr)
{
    int res = SBIGUnivDrvCommand(CC_GET_DRIVER_HANDLE, nullptr, gdhr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_GET_DRIVER_HANDLE -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::StartExposure(StartExposureParams2 *sep)
{
    if (isSimulation())
    {
        return CE_NO_ERROR;
    }
    int res = SBIGUnivDrvCommand(CC_START_EXPOSURE2, sep, nullptr);
    if (res != CE_NO_ERROR)
    {
        // If we need to ignore shutter errors, let's do so.
        if (res == CE_SHUTTER_ERROR && IgnoreErrorsS[0].s == ISS_ON)
            res = CE_NO_ERROR;
        else
            LOGF_ERROR("%s: CC_START_EXPOSURE2 -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::EndExposure(EndExposureParams *eep)
{
    if (isSimulation())
    {
        return CE_NO_ERROR;
    }
    int res = SBIGUnivDrvCommand(CC_END_EXPOSURE, eep, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_END_EXPOSURE -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::StartReadout(StartReadoutParams *srp)
{
    int res = SBIGUnivDrvCommand(CC_START_READOUT, srp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_START_READOUT -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::ReadoutLine(ReadoutLineParams *rlp, uint16_t *results, bool bSubtract)
{
    int res;
    if (bSubtract)
    {
        res = SBIGUnivDrvCommand(CC_READ_SUBTRACT_LINE, rlp, results);
    }
    else
    {
        res = SBIGUnivDrvCommand(CC_READOUT_LINE, rlp, results);
    }
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_READ_SUBTRACT_LINE/CC_READOUT_LINE -> (%s)", __FUNCTION__,
                   GetErrorString(res));
    }
    return res;
}

int SBIGCCD::DumpLines(DumpLinesParams *dlp)
{
    int res = SBIGUnivDrvCommand(CC_DUMP_LINES, dlp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_DUMP_LINES -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::EndReadout(EndReadoutParams *erp)
{
    int res = SBIGUnivDrvCommand(CC_END_READOUT, erp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_END_READOUT -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::SetTemperatureRegulation(SetTemperatureRegulationParams *strp)
{
    int res = SBIGUnivDrvCommand(CC_SET_TEMPERATURE_REGULATION, strp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_SET_TEMPERATURE_REGULATION -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::SetTemperatureRegulation(double temperature, bool enable)
{
    int res;
    SetTemperatureRegulationParams strp;
    if (isSimulation())
    {
        TemperatureNP[0].setValue(temperature);
        return CE_NO_ERROR;
    }
    if (CheckLink())
    {
        strp.regulation  = enable ? REGULATION_ON : REGULATION_OFF;
        strp.ccdSetpoint = CalcSetpoint(temperature);
        res              = SBIGUnivDrvCommand(CC_SET_TEMPERATURE_REGULATION, &strp, nullptr);
    }
    else
    {
        res = CE_DEVICE_NOT_OPEN;
    }
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_SET_TEMPERATURE_REGULATION -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::QueryTemperatureStatus(bool &enabled, double &ccdTemp, double &setpointTemp, double &power)
{
    int res;
    QueryTemperatureStatusResults qtsr;
    if (isSimulation())
    {
        enabled      = (CoolerS[0].s == ISS_ON);
        ccdTemp      = TemperatureNP[0].getValue();
        setpointTemp = ccdTemp;
        power        = enabled ? 0.5 : 0;
        return CE_NO_ERROR;
    }
    if (CheckLink())
    {
        res = SBIGUnivDrvCommand(CC_QUERY_TEMPERATURE_STATUS, nullptr, &qtsr);
        if (res == CE_NO_ERROR)
        {
            enabled      = (qtsr.enabled != 0);
            ccdTemp      = CalcTemperature(CCD_THERMISTOR, qtsr.ccdThermistor);
            setpointTemp = CalcTemperature(CCD_THERMISTOR, qtsr.ccdSetpoint);
            power        = qtsr.power / 255.0;
            LOGF_DEBUG("Cooler: %s Temperature: %.3f Set Point: %.3f Power: %.2f",
                       enabled ? "On" : "Off", ccdTemp, setpointTemp, power);
        }
    }
    else
    {
        res = CE_DEVICE_NOT_OPEN;
    }
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_SET_TEMPERATURE_REGULATION -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

uint16_t SBIGCCD::CalcSetpoint(double temperature)
{
    // Calculate 'setpoint' from the temperature T in degr. of Celsius.
    double expo = (log(R_RATIO_CCD) * (T0 - temperature)) / DT_CCD;
    double r    = R0 * exp(expo);
    return (static_cast<uint16_t>(((MAX_AD / (R_BRIDGE_CCD / r + 1.0)) + 0.5)));
}

double SBIGCCD::CalcTemperature(short thermistorType, short setpoint)
{
    double r, expo, rBridge, rRatio, dt;
    switch (thermistorType)
    {
        case AMBIENT_THERMISTOR:
            rBridge = R_BRIDGE_AMBIENT;
            rRatio  = R_RATIO_AMBIENT;
            dt      = DT_AMBIENT;
            break;
        case CCD_THERMISTOR:
        default:
            rBridge = R_BRIDGE_CCD;
            rRatio  = R_RATIO_CCD;
            dt      = DT_CCD;
            break;
    }
    // Calculate temperature T in degr. Celsius from the 'setpoint'
    r    = rBridge / ((MAX_AD / setpoint) - 1.0);
    expo = log(r / R0) / log(rRatio);
    return (T0 - dt * expo);
}

int SBIGCCD::ActivateRelay(ActivateRelayParams *arp)
{
    int res = SBIGUnivDrvCommand(CC_ACTIVATE_RELAY, arp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_ACTIVATE_RELAY -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::PulseOut(PulseOutParams *pop)
{
    int res = SBIGUnivDrvCommand(CC_PULSE_OUT, pop, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_PULSE_OUT -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::TxSerialBytes(TXSerialBytesParams *txsbp, TXSerialBytesResults *txsbr)
{
    int res = SBIGUnivDrvCommand(CC_TX_SERIAL_BYTES, txsbp, txsbr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_TX_SERIAL_BYTES -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::GetSerialStatus(GetSerialStatusResults *gssr)
{
    int res = SBIGUnivDrvCommand(CC_GET_SERIAL_STATUS, nullptr, gssr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_GET_SERIAL_STATUS -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::AoTipTilt()
{
    int res = SBIGUnivDrvCommand(CC_AO_TIP_TILT, &m_AOParams, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_AO_TIP_TILT -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::AoDelay(AODelayParams *aodp)
{
    int res = SBIGUnivDrvCommand(CC_AO_DELAY, aodp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_AO_DELAY -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::AoSetFocus(AOSetFocusParams *aofc)
{
    int res = SBIGUnivDrvCommand(CC_AO_SET_FOCUS, aofc, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_AO_SET_FOCUS -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::AoCenter()
{
    int res = SBIGUnivDrvCommand(CC_AO_CENTER, nullptr, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_AO_CENTER -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::CFW(CFWParams *CFWp, CFWResults *CFWr)
{
    int res = SBIGUnivDrvCommand(CC_CFW, CFWp, CFWr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_CFW -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::EstablishLink()
{
    EstablishLinkParams elp;
    EstablishLinkResults elr;

    elp.sbigUseOnly = 0;
    int res         = SBIGUnivDrvCommand(CC_ESTABLISH_LINK, &elp, &elr);
    if (res == CE_NO_ERROR)
    {
        SetCameraType(static_cast<CAMERA_TYPE>(elr.cameraType));
        SetLinkStatus(true);
    }
    else
    {
        LOGF_ERROR("%s: CC_ESTABLISH_LINK -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::GetCcdInfo(GetCCDInfoParams *gcp, void *gcr)
{
    int res = SBIGUnivDrvCommand(CC_GET_CCD_INFO, gcp, gcr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_GET_CCD_INFO -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::getCCDSizeInfo(int ccd, int binning, int &frmW, int &frmH, double &pixW, double &pixH)
{
    int roModeIdx = binning & 0x00FF;  // mask low byte because vertical binning is written to high byte
    GetCCDInfoParams gcp;
    GetCCDInfoResults0 gcr;
    if (isSimulation())
    {
        if (ccd == CCD_IMAGING)
        {
            frmW = 1024;
            frmH = 1024;
        }
        else
        {
            frmW = 512;
            frmH = 512;
        }
        pixW = 5.2;
        pixH = 5.2;
        return CE_NO_ERROR;
    }
    gcp.request = ccd;
    int res     = SBIGUnivDrvCommand(CC_GET_CCD_INFO, &gcp, &gcr);
    // If there is no name, then it is invalid
    if (!gcr.name[0])
        return CE_DEVICE_NOT_IMPLEMENTED;
    if (res == CE_NO_ERROR)
    {
        frmW = gcr.readoutInfo[roModeIdx].width;
        pixW = BcdPixel2double(gcr.readoutInfo[roModeIdx].pixelWidth);
        // modify height and pixelHeight for readout modes with vertical binning
        if ( ( (binning & 0x00FF) == CCD_BIN_1xN_I ) ||
                ( (binning & 0x00FF) == CCD_BIN_2xN_I ) ||
                ( (binning & 0x00FF) == CCD_BIN_3xN_I ) )
        {
            frmH = gcr.readoutInfo[0].height / ( (binning & 0xFF00) >> 8);
            pixH = BcdPixel2double(gcr.readoutInfo[0].pixelHeight * ( (binning & 0xFF00) >> 8) );
        }
        else
        {
            frmH = gcr.readoutInfo[roModeIdx].height;
            pixH = BcdPixel2double(gcr.readoutInfo[roModeIdx].pixelHeight);
        }
        LOGF_DEBUG(
            "%s: CC_GET_CCD_INFO -> binning (%d) width (%d) height (%d) pixW (%g) pixH (%g)", __FUNCTION__, binning,
            frmW, frmH, pixW, pixH);
    }
    else
    {
        LOGF_ERROR("%s: CC_GET_CCD_INFO -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::QueryCommandStatus(QueryCommandStatusParams *qcsp, QueryCommandStatusResults *qcsr)
{
    int res = SBIGUnivDrvCommand(CC_QUERY_COMMAND_STATUS, qcsp, qcsr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_QUERY_COMMAND_STATUS -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::MiscellaneousControl(MiscellaneousControlParams *mcp)
{
    int res = SBIGUnivDrvCommand(CC_MISCELLANEOUS_CONTROL, mcp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_QUERY_COMMAND_STATUS -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::ReadOffset(ReadOffsetParams *rop, ReadOffsetResults *ror)
{
    int res = SBIGUnivDrvCommand(CC_READ_OFFSET, rop, ror);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_READ_OFFSET -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::GetLinkStatus(GetLinkStatusResults *glsr)
{
    int res = SBIGUnivDrvCommand(CC_GET_LINK_STATUS, glsr, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_GET_LINK_STATUS -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

char *SBIGCCD::GetErrorString(int err)
{
    GetErrorStringParams gesp;
    gesp.errorNo = err;
    static GetErrorStringResults gesr;
    int res = SBIGUnivDrvCommand(CC_GET_ERROR_STRING, &gesp, &gesr);
    if (res == CE_NO_ERROR)
    {
        return gesr.errorString;
    }
    static char str[128];
    sprintf(str, "No error string found! Error code: %d", err);
    return str;
}

int SBIGCCD::SetDriverControl(SetDriverControlParams *sdcp)
{
    int res = SBIGUnivDrvCommand(CC_SET_DRIVER_CONTROL, sdcp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_SET_DRIVER_CONTROL -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::GetDriverControl(GetDriverControlParams *gdcp, GetDriverControlResults *gdcr)
{
    int res = SBIGUnivDrvCommand(CC_GET_DRIVER_CONTROL, gdcp, gdcr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_GET_DRIVER_CONTROL -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::UsbAdControl(USBADControlParams *usbadcp)
{
    int res = SBIGUnivDrvCommand(CC_USB_AD_CONTROL, usbadcp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_USB_AD_CONTROL -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::QueryUsb(QueryUSBResults *qusbr)
{
    int res = SBIGUnivDrvCommand(CC_QUERY_USB, nullptr, qusbr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_QUERY_USB -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::RwUsbI2c(RWUSBI2CParams *rwusbi2cp)
{
    int res = SBIGUnivDrvCommand(CC_RW_USB_I2C, rwusbi2cp, nullptr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_RW_USB_I2C -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

int SBIGCCD::BitIo(BitIOParams *biop, BitIOResults *bior)
{
    int res = SBIGUnivDrvCommand(CC_BIT_IO, biop, bior);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_BIT_IO -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    return res;
}

const char *SBIGCCD::GetCameraName()
{
    if (isSimulation())
        return "Simulated camera";

    GetCCDInfoParams gccdip;
    static GetCCDInfoResults0 gccdir;
    gccdip.request = CCD_INFO_IMAGING;
    int res        = SBIGUnivDrvCommand(CC_GET_CCD_INFO, &gccdip, &gccdir);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_GET_CCD_INFO -> (%s)", __FUNCTION__, GetErrorString(res));
        return "Unknown camera";
    }

    if (gccdir.cameraType == NO_CAMERA)
    {
        return "No camera";
    }

    return gccdir.name;
}

const char *SBIGCCD::GetCameraID()
{
    if (isSimulation())
    {
        return "Simulated ID";
    }

    GetCCDInfoParams gccdip;
    static GetCCDInfoResults2 gccdir2;
    gccdip.request = CCD_INFO_EXTENDED;
    int res        = GetCcdInfo(&gccdip, &gccdir2);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_GET_CCD_INFO -> (%s)", __FUNCTION__, GetErrorString(res));
        return "Unknown ID";
    }
    return gccdir2.serialNumber;
}

int SBIGCCD::getReadoutModes(INDI::CCDChip *targetChip, int &numModes, int &maxBinX, int &maxBinY)
{
    int res = CE_BAD_PARAMETER;
    GetCCDInfoParams gccdip;
    static GetCCDInfoResults0 gccdir;

    if (targetChip == &PrimaryCCD)
        gccdip.request = CCD_INFO_IMAGING;
    else if (targetChip == &GuideCCD)
        gccdip.request = CCD_INFO_TRACKING;
    else
    {
        LOGF_ERROR("%s: CCD not selected and/or uninitialized", __FUNCTION__);
        return res;
    }
    res = SBIGUnivDrvCommand(CC_GET_CCD_INFO, &gccdip, &gccdir);
    if (res == CE_NO_ERROR)
    {
        numModes = gccdir.readoutModes - 1;
        switch(numModes)
        {
            case CCD_BIN_2x2_I:
            case CCD_BIN_2x2_E:
                maxBinX = 2;
                maxBinY = maxBinX;
                break;
            case CCD_BIN_3x3_I:
            case CCD_BIN_3x3_E:
                maxBinX = 3;
                maxBinY = maxBinX;
                break;
            case CCD_BIN_1xN_I:
                maxBinX = 1;
                maxBinY = 255;
                break;
            case CCD_BIN_2xN_I:
                maxBinX = 2;
                maxBinY = 255;
                break;
            case CCD_BIN_3xN_I:
                maxBinX = 3;
                maxBinY = 255;
                break;
            case CCD_BIN_9x9_I:
                maxBinX = 9;
                maxBinY = 255; // because vertical binning modes are also available
                break;
            case CCD_BIN_NxN_I:
                maxBinX = 255;
                maxBinY = maxBinX;
                break;
            case CCD_BIN_1x1_I:
            case CCD_BIN_1x1_E:
            default: // no binning at all
                maxBinX = 1;
                maxBinY = maxBinX;
                break;
        }
    }
    LOGF_DEBUG("%s: max horizontal/vertical binning (%d / %d) supported readout modes (%d)", __FUNCTION__,
               maxBinX, maxBinY, numModes);
    return res;
}

int SBIGCCD::GetExtendedCCDInfo()
{
    int res = CE_NO_ERROR;
    GetCCDInfoParams gccdip;
    GetCCDInfoResults4 results4;
    GetCCDInfoResults6 results6;

    LOG_DEBUG("Fetching extended CCD info from device ...");

    if (isSimulation())
    {
        m_hasGuideHead   = true;
        m_hasFilterWheel = true;
        return res;
    }

    // Extended 2 Imaging caps
    gccdip.request = CCD_INFO_EXTENDED2_IMAGING;
    if ((res = GetCcdInfo(&gccdip, &results4)) == CE_NO_ERROR)
        LOGF_DEBUG("CCD_IMAGING Extended CCD Info 4. CapabilitiesBit: (%u) Dump Extra (%u)", results4.capabilitiesBits,
                   results4.dumpExtra);
    else
    {
        LOGF_WARN("%s: CCD_INFO_EXTENDED2_IMAGING -> (%s)", __FUNCTION__, GetErrorString(res));
    }

    // Extended 2 Tracking
    gccdip.request = CCD_INFO_EXTENDED2_TRACKING;
    if ((res = GetCcdInfo(&gccdip, &results4)) == CE_NO_ERROR)
    {
        m_hasGuideHead = true;
        m_useExternalTrackingCCD = results4.capabilitiesBits & CB_CCD_EXT_TRACKER_YES;
        LOGF_DEBUG("TRACKING_CCD Extended CCD Info 4. CapabilitiesBit: (%u) Dump Extra (%u)",
                   results4.capabilitiesBits, results4.dumpExtra);
    }
    else
    {
        m_hasGuideHead = false;
        LOGF_DEBUG("%s: CCD_INFO_EXTENDED2_TRACKING -> (%s). No guide head detected.", __FUNCTION__, GetErrorString(res));
    }

    gccdip.request = CCD_INFO_EXTENDED3;
    if ((res = GetCcdInfo(&gccdip, &results6)) == CE_NO_ERROR)
    {
        LOGF_DEBUG("Extended CCD Info 6. Camerabit: (%ld) CCD bits (%ld) Extra bit (%ld)",   results6.cameraBits, results6.ccdBits,
                   results6.extraBits);
        if (results6.ccdBits & 0x0001)
        {
            LOG_DEBUG("Color CCD detected.");
            m_isColor = true;
            LOGF_DEBUG("Detected color matrix is %s.",
                       (results6.ccdBits & 0x0002) ? "Truesense" : "Bayer");
        }
        else
        {
            LOG_DEBUG("Mono CCD detected.");
            m_isColor = false;
        }
    }
    else
    {
        LOGF_DEBUG("Error getting extended CCD Info 6 (%s)", GetErrorString(res));
    }

    CFWParams CFWp;
    CFWResults CFWr;
    CFWp.cfwModel   = CFWSEL_AUTO;
    CFWp.cfwCommand = CFWC_GET_INFO;
    CFWp.cfwParam1  = CFWG_FIRMWARE_VERSION;
    if ((res = SBIGUnivDrvCommand(CC_CFW, &CFWp, &CFWr)) == CE_NO_ERROR)
    {
        LOGF_DEBUG("Filter wheel detected (firmware %ld).", CFWr.cfwResult1);
        m_hasFilterWheel = true;
    }
    else
    {
        m_hasFilterWheel = false;
    }
    return CE_NO_ERROR;
}

//==========================================================================
/*int SBIGCCD::SetDeviceName(const char *name) {
  int res = CE_NO_ERROR;
  if (strlen(name) < PATH_MAX) {
    strcpy(m_dev_name, name);
  } else {
    res = CE_BAD_PARAMETER;
  }
  return res;
}*/

//==========================================================================
// SBIGUnivDrvCommand:
// Bottleneck function for all calls to the driver that logs the command
// and error. First it activates our handle and then it calls the driver.
// Activating the handle first allows having multiple instances of this
// class dealing with multiple cameras on different communications port.
// Also allows direct access to the SBIG Universal Driver after the driver
// has been opened.

int SBIGCCD::SBIGUnivDrvCommand(PAR_COMMAND command, void *params, void *results)
{
    int res;
    SetDriverHandleParams sdhp;
    if (isSimulation())
    {
        return CE_NO_ERROR;
    }
    // Make sure we have a valid handle to the driver.
    if (GetDriverHandle() == INVALID_HANDLE_VALUE)
    {
        res = CE_DRIVER_NOT_OPEN;
    }
    else
    {
        // Handle is valid so install it in the driver.
        sdhp.handle = GetDriverHandle();
        res         = ::SBIGUnivDrvCommand(CC_SET_DRIVER_HANDLE, &sdhp, nullptr);
        if (res == CE_NO_ERROR)
        {
            res = ::SBIGUnivDrvCommand(command, params, results);
        }
    }
    return res;
}

bool SBIGCCD::CheckLink()
{
    if (GetCameraType() != NO_CAMERA && GetLinkStatus())
    {
        return true;
    }
    return false;
}

/*int SBIGCCD::getNumberOfINDI::CCDChips()
{
    int res;

    switch(GetCameraType())
    {
        case ST237_CAMERA:
        case ST5C_CAMERA:
        case ST402_CAMERA:
        case STI_CAMERA:
        case STT_CAMERA:
        case STF_CAMERA:
                    res = 1;
                    break;
        case ST7_CAMERA:
        case ST8_CAMERA:
        case ST9_CAMERA:
        case ST10_CAMERA:
        case ST2K_CAMERA:
                    res = 2;
                    break;
        case STL_CAMERA:
                    res = 3;
                    break;
        case NO_CAMERA:
        default:
                    res = 0;
                    break;
    }

    LOGF_DEBUG("%s Camera Type (%d) Number of chips (%d)", __FUNCTION__, GetCameraType(), res);
    return res;
}*/

//==========================================================================

bool SBIGCCD::IsFanControlAvailable()
{
    CAMERA_TYPE camera = GetCameraType();
    if (camera == ST5C_CAMERA || camera == ST402_CAMERA || camera == STI_CAMERA)
    {
        return false;
    }
    return true;
}

double SBIGCCD::BcdPixel2double(ulong bcd)
{
    double value = 0.0;
    double digit = 0.01;
    for (int i = 0; i < 8; i++)
    {
        value += (bcd & 0x0F) * digit;
        digit *= 10.0;
        bcd >>= 4;
    }
    return value;
}

void SBIGCCD::InitVars()
{
    SetFileDescriptor();
    SetCameraType();
    SetLinkStatus();
    memset(&rp, 0, sizeof(rp));
    m_AOParams.xDeflection = m_AOParams.yDeflection = 2048;
}

//==========================================================================

int SBIGCCD::getBinningMode(INDI::CCDChip *targetChip, int &binning)
{
    int maxBinX, maxBinY, numModes;
    int res = getReadoutModes(targetChip, numModes, maxBinX, maxBinY);

    if (res != CE_NO_ERROR || targetChip->getBinX() > maxBinX || targetChip->getBinY() > maxBinY)
    {
        binning = CCD_BIN_1x1_I;
        return res;
    }

    if (targetChip->getBinX() == targetChip->getBinY())
    {
        if (targetChip->getBinX() == 1)
            binning = CCD_BIN_1x1_I;
        else if (targetChip->getBinX() == 2)
            binning = CCD_BIN_2x2_I;
        else if (targetChip->getBinX() == 3)
            binning = CCD_BIN_3x3_I;
        else if (targetChip->getBinX() == 9)
            binning = CCD_BIN_9x9_I;
        else
            // store amount of binning in the high byte
            // not mentioned in the SBIG documentation, but probably similar to the
            // vertical binning case (see below)
            binning = CCD_BIN_NxN_I + (targetChip->getBinX() << 8);
    }
    else
    {
        if (targetChip->getBinX() == 1)
            binning = CCD_BIN_1xN_I;
        else if (targetChip->getBinX() == 2)
            binning = CCD_BIN_2xN_I;
        else if (targetChip->getBinX() == 3)
            binning = CCD_BIN_3xN_I;
        else // this should not happen
        {
            res = CE_BAD_PARAMETER;
            LOG_ERROR("Bad CCD binning mode: x-binning > 3 and y-binning != x-binning");
        }
        // amount of vertical binning is specified in the most significant byte of the readout mode
        // (see notes in Sec. 3.2.4 of SBIG Universal Driver documentation)
        binning += targetChip->getBinY() << 8;
    }

    // check if the requested binning mode (low byte only) is supported by the device
    if ((binning & 0x00FF) > (numModes - 1))
    {
        res = CE_BAD_PARAMETER;
        binning = CCD_BIN_1x1_I; // fallback mode -> no binning
        LOG_ERROR("Binning mode not supported by the device");
    }

    LOGF_DEBUG("%s: binx (%d) biny (%d) binning_mode (%d)", __FUNCTION__, targetChip->getBinX(),
               targetChip->getBinY(), binning);
    return res;
}

int SBIGCCD::getFrameType(INDI::CCDChip *targetChip, INDI::CCDChip::CCD_FRAME *frameType)
{
    *frameType = targetChip->getFrameType();
    return CE_NO_ERROR;
}

int SBIGCCD::getShutterMode(INDI::CCDChip *targetChip, int &shutter)
{
    int res = CE_NO_ERROR;

    INDI::CCDChip::CCD_FRAME frameType;
    getFrameType(targetChip, &frameType);

    int ccd = CCD_IMAGING;
    if (targetChip == &PrimaryCCD)
    {
        ccd = CCD_IMAGING;
    }
    else if (targetChip == &GuideCCD)
    {
        ccd = m_useExternalTrackingCCD ? CCD_EXT_TRACKING : CCD_TRACKING;
    }
    if (frameType == INDI::CCDChip::LIGHT_FRAME || frameType == INDI::CCDChip::FLAT_FRAME)
    {
        if (ccd == CCD_EXT_TRACKING)
        {
            shutter = SC_OPEN_EXT_SHUTTER;
        }
        else
        {
            shutter = SC_OPEN_SHUTTER;
        }
    }
    else if (frameType == INDI::CCDChip::DARK_FRAME || frameType == INDI::CCDChip::BIAS_FRAME)
    {
        if (ccd == CCD_EXT_TRACKING)
        {
            shutter = SC_CLOSE_EXT_SHUTTER;
        }
        else
        {
            shutter = SC_CLOSE_SHUTTER;
        }
    }
    else
    {
        res = CE_OS_ERROR;
        LOGF_ERROR("Unknown selected CCD frame type %s", targetChip->getFrameTypeName(frameType));
    }
    return res;
}

bool SBIGCCD::SelectFilter(int position)
{
    CFWResults CFWr;
    int res = CFWGoto(&CFWr, position);
    if (res == CE_NO_ERROR)
    {
        int type = GetCFWSelType();
        if (type == CFWSEL_CFW6A || type == CFWSEL_CFW8)
        {
            LOG_INFO("CFW position reached");
            CFWr.cfwPosition = position;
        }
        else
        {
            LOGF_INFO("CFW position %d reached.", CFWr.cfwPosition);
        }
        SelectFilterDone(CurrentFilter = CFWr.cfwPosition);
        return true;
    }
    else
    {
        FilterSlotNP.setState(IPS_ALERT);
        FilterSlotNP.apply();
        LOG_INFO("Failed to reach position");
        return false;
    }
}

int SBIGCCD::QueryFilter()
{
    return CurrentFilter;
}

void SBIGCCD::updateTemperatureHelper(void *p)
{
    if (static_cast<SBIGCCD *>(p)->isConnected())
        static_cast<SBIGCCD *>(p)->updateTemperature();
}

void SBIGCCD::updateTemperature()
{
    bool enabled;
    double ccdTemp, setpointTemp, percentTE, power;

    std::unique_lock<std::mutex> guard(sbigLock);
    int res = QueryTemperatureStatus(enabled, ccdTemp, setpointTemp, percentTE);
    guard.unlock();

    if (res == CE_NO_ERROR)
    {
        power = 100.0 * percentTE;
        // Compare the current temperature against the setpoint value:
        //        if (fabs(setpointTemp - ccdTemp) <= TEMP_DIFF)
        //        {
        //            TemperatureNP.s = IPS_OK;
        //        }
        if (power == 0)
        {
            TemperatureNP.setState(IPS_IDLE);
        }
        else
        {
            TemperatureNP.setState(IPS_BUSY);
            LOGF_DEBUG("CCD temperature %+.1f [C], TE cooler: %.1f [%%].", ccdTemp, power);
        }
        TemperatureNP[0].setValue(ccdTemp);
        // Check the TE cooler if inside the range:
        if (power <= CCD_COOLER_THRESHOLD)
        {
            CoolerNP.s = IPS_OK;
        }
        else
        {
            CoolerNP.s = IPS_BUSY;
        }
        CoolerN[0].value = power;
        TemperatureNP.apply();
        IDSetNumber(&CoolerNP, nullptr);
    }
    else
    {
        // ignore share errors
        if (res == CE_SHARE_ERROR)
        {
            LOGF_DEBUG("Erro reading temperature. %s", GetErrorString(res));
            TemperatureNP.setState(IPS_IDLE);
        }
        else
        {
            LOGF_ERROR("Erro reading temperature. %s", GetErrorString(res));
            TemperatureNP.setState(IPS_ALERT);
        }
        TemperatureNP.apply();
    }
    IEAddTimer(TEMPERATURE_POLL_MS, SBIGCCD::updateTemperatureHelper, this);
}

bool SBIGCCD::isExposureDone(INDI::CCDChip *targetChip)
{
    int ccd = 0;

    if (isSimulation())
    {
        double timeLeft = 1e6;

        if (targetChip == &PrimaryCCD)
        {
            std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - ExpStart;
            timeLeft = ExposureRequest - elapsed.count();
        }
        else
        {
            std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - GuideExpStart;
            timeLeft = GuideExposureRequest - elapsed.count();
        }

        return (timeLeft <= 0);
    }

    if (targetChip == &PrimaryCCD)
    {
        ccd = CCD_IMAGING;
    }
    else
    {
        ccd = m_useExternalTrackingCCD ? CCD_EXT_TRACKING : CCD_TRACKING;
    }

    EndExposureParams eep;
    QueryCommandStatusParams qcsp;
    QueryCommandStatusResults qcsr;

    // Query command status:
    qcsp.command = CC_START_EXPOSURE2;
    std::unique_lock<std::mutex> guard(sbigLock);
    int res = QueryCommandStatus(&qcsp, &qcsr);
    if (res != CE_NO_ERROR)
    {
        guard.unlock();
        return false;
    }

    int mask = 12; // Tracking & external tracking CCD chip mask.
    if (ccd == CCD_IMAGING)
    {
        mask = 3; // Imaging chip mask.
    }

    // Check exposure progress:
    if ((qcsr.status & mask) != mask)
    {
        // The exposure is still in progress, decrement an
        // exposure time:
        guard.unlock();
        return false;
    }
    // Exposure done - update client's property:
    eep.ccd = ccd;
    EndExposure(&eep);
    guard.unlock();
    return true;
}

//==========================================================================

int SBIGCCD::readoutCCD(uint16_t left, uint16_t top, uint16_t width, uint16_t height,
                        uint16_t *buffer, INDI::CCDChip *targetChip)
{
    int h, ccd, binning, res;
    if (targetChip == &PrimaryCCD)
    {
        ccd = CCD_IMAGING;
    }
    else
    {
        ccd = m_useExternalTrackingCCD ? CCD_EXT_TRACKING : CCD_TRACKING;
    }
    if ((res = getBinningMode(targetChip, binning)) != CE_NO_ERROR)
    {
        return res;
    }
    StartReadoutParams srp;
    srp.ccd         = ccd;
    srp.readoutMode = binning;
    srp.left        = left;
    srp.top         = top;
    srp.width       = width;
    srp.height      = height;
    std::unique_lock<std::mutex> guard(sbigLock);
    res = StartReadout(&srp);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s readoutCCD - StartReadout error! (%s)",
                   (targetChip == &PrimaryCCD) ? "Primary" : "Guide", GetErrorString(res));
        guard.unlock();
        return res;
    }
    ReadoutLineParams rlp;
    rlp.ccd         = ccd;
    rlp.readoutMode = binning;
    rlp.pixelStart  = left;
    rlp.pixelLength = width;
    for (h = 0; h < height; h++)
    {
        ReadoutLine(&rlp, buffer + (h * width), false);
    }
    EndReadoutParams erp;
    erp.ccd = ccd;
    if ((res = EndReadout(&erp)) != CE_NO_ERROR)
    {
        LOGF_ERROR("%s readoutCCD - EndReadout error! (%s)",
                   (targetChip == &PrimaryCCD) ? "Primary" : "Guide", GetErrorString(res));
        guard.unlock();
        return res;
    }
    guard.unlock();
    return res;
}

//==========================================================================

int SBIGCCD::CFWConnect()
{
    IUResetSwitch(&FilterConnectionSP);
    if (isConnected() == false)
    {
        LOG_ERROR("You must establish connection to CCD before connecting to filter wheel.");
        FilterConnectionSP.s   = IPS_IDLE;
        FilterConnectionS[1].s = ISS_ON;
        IDSetSwitch(&FilterConnectionSP, nullptr);
        return CE_OS_ERROR;
    }

    CFWResults CFWr;
    CFWParams CFWp;
    int res       = CE_NO_ERROR;
    CFWp.cfwModel = GetCFWSelType();
    if (CFWp.cfwModel == CFWSEL_CFW10_SERIAL)
    {
        CFWp.cfwCommand = CFWC_OPEN_DEVICE;
        res             = SBIGUnivDrvCommand(CC_CFW, &CFWp, &CFWr);
        if (res != CE_NO_ERROR)
            LOGF_ERROR("%s: CC_CFW/CFWC_OPEN_DEVICE -> (%s)", __FUNCTION__, GetErrorString(res));
    }
    if (res == CE_NO_ERROR)
    {
        CFWp.cfwCommand = CFWC_INIT;
        for (int i = 0; i < 3; i++)
        {
            res = SBIGUnivDrvCommand(CC_CFW, &CFWp, &CFWr);
            if (res == CE_NO_ERROR)
            {
                res = CFWGotoMonitor(&CFWr);
                break;
            }
            LOGF_ERROR("%s: CC_CFW/CFWC_INIT -> (%s)", __FUNCTION__, GetErrorString(res));
            sleep(1);
        }
    }
    if (res == CE_NO_ERROR)
    {
        if (isSimulation())
        {
            CFWr.cfwModel    = CFWp.cfwModel;
            CFWr.cfwPosition = 1;
            CFWr.cfwResult1  = 0;
            int cfwsim[16]   = { 2, 5, 6, 8, 4, 10, 10, 8, 9, 8, 10, 5, 5, 8, 7, 8 };
            int filnum       = IUFindOnSwitchIndex(&FilterTypeSP);
            if (filnum < 0)
            {
                CFWr.cfwResult2 = 5;
            }
            else
            {
                CFWr.cfwResult2 = cfwsim[filnum];
            }
        }
        else
        {
            CFWp.cfwCommand = CFWC_GET_INFO;
            CFWp.cfwParam1  = CFWG_FIRMWARE_VERSION;
            res             = SBIGUnivDrvCommand(CC_CFW, &CFWp, &CFWr);
            if (res != CE_NO_ERROR)
                LOGF_ERROR("%s: CC_CFW/CFWC_GET_INFO -> (%s)", __FUNCTION__, GetErrorString(res));
        }
    }
    if (res == CE_NO_ERROR)
    {
        const char *name = "Unknown filterwheel";
        char fw[64]      = "Unknown ID";
        bool bClear      = true;
        int model        = CFWr.cfwModel;
        for (int i = 0; i < MAX_CFW_TYPES; i++)
        {
            if (model == SBIGFilterMap[i])
            {
                name   = FilterTypeS[i].label;
                bClear = false;
                break;
            }
        }
        IText *pIText = IUFindText(&FilterProdcutTP, "NAME");
        if (pIText)
        {
            IUSaveText(pIText, name);
        }
        LOGF_DEBUG("CFW Product ID: %s", name);
        if (!bClear)
        {
            sprintf(fw, "%ld", CFWr.cfwResult1);
        }
        pIText = IUFindText(&FilterProdcutTP, "ID");
        if (pIText)
        {
            IUSaveText(pIText, fw);
        }
        LOGF_DEBUG("CFW Firmware: %s", fw);
        FilterProdcutTP.s = IPS_OK;
        defineProperty(&FilterProdcutTP);
        FilterSlotNP[0].setMin(1);
        FilterSlotNP[0].setMax(CFWr.cfwResult2);
        FilterSlotNP[0].setValue(CFWr.cfwPosition);
        if (FilterSlotNP[0].getValue() < FilterSlotNP[0].getMin())
        {
            FilterSlotNP[0].setValue(FilterSlotNP[0].getMin());
        }
        else if (FilterSlotNP[0].getValue() > FilterSlotNP[0].getMax())
        {
            FilterSlotNP[0].setValue(FilterSlotNP[0].getMax());
        }

        LOGF_DEBUG("CFW min: 1 Max: %g Current Slot: %g", FilterSlotNP[0].getMax(), FilterSlotNP[0].getValue());

        defineProperty(FilterSlotNP);
        if (FilterNameTP.size() == 0)
            GetFilterNames();
        if (FilterNameTP.size() > 0)
            defineProperty(FilterNameTP);

        LOG_DEBUG("Loading FILTER_SLOT from config file...");
        loadConfig(true, "FILTER_SLOT");

        FilterConnectionSP.s = IPS_OK;
        LOG_INFO("CFW connected.");
        FilterConnectionS[0].s = ISS_ON;
        IDSetSwitch(&FilterConnectionSP, nullptr);
    }
    else
    {
        FilterConnectionSP.s   = IPS_ALERT;
        FilterConnectionS[1].s = ISS_ON;
        IUResetSwitch(&FilterConnectionSP);
        FilterConnectionSP.sp[1].s = ISS_ON;
        LOG_ERROR("Failed to connect CFW");
        IDSetSwitch(&FilterConnectionSP, nullptr);
    }
    return res;
}

//==========================================================================

int SBIGCCD::CFWDisconnect()
{
    CFWParams CFWp;
    CFWResults CFWr;
    CFWp.cfwModel   = GetCFWSelType();
    CFWp.cfwCommand = CFWC_CLOSE_DEVICE;
    IUResetSwitch(&FilterConnectionSP);
    int res = SBIGUnivDrvCommand(CC_CFW, &CFWp, &CFWr);
    if (res != CE_NO_ERROR)
    {
        LOGF_ERROR("%s: CC_CFW/CFWC_CLOSE_DEVICE -> (%s)", __FUNCTION__, GetErrorString(res));
        FilterConnectionS[0].s = ISS_ON;
        FilterConnectionSP.s   = IPS_ALERT;
        IDSetSwitch(&FilterConnectionSP, "Failed to disconnect CFW");
    }
    else
    {
        FilterConnectionS[1].s = ISS_ON;
        FilterConnectionSP.s   = IPS_IDLE;
        IDSetSwitch(&FilterConnectionSP, "CFW disconnected");
        deleteProperty(FilterSlotNP);
        deleteProperty(FilterProdcutTP.name);
        deleteProperty(FilterNameTP);
    }
    return res;
}

//==========================================================================

int SBIGCCD::CFWQuery(CFWResults *CFWr)
{
    CFWParams CFWp;
    CFWp.cfwModel   = GetCFWSelType();
    CFWp.cfwCommand = CFWC_QUERY;
    int res         = SBIGUnivDrvCommand(CC_CFW, &CFWp, CFWr);
    if (res != CE_NO_ERROR)
        LOGF_ERROR("%s: CC_CFW/CFWC_QUERY -> (%s)", __FUNCTION__, GetErrorString(res));
    return res;
}

//==========================================================================

int SBIGCCD::CFWGoto(CFWResults *CFWr, int position)
{
    if (CFWr == nullptr)
        return CE_NO_ERROR;

    if (isSimulation())
    {
        CFWr->cfwPosition = position;
        return CE_NO_ERROR;
    }
    LOGF_DEBUG("CFW GOTO: %d", position);

    // 2014-06-16: Do we need to also checking if the position is reached here? A test will determine.
    CFWParams CFWp;
    CFWp.cfwModel   = GetCFWSelType();
    CFWp.cfwCommand = CFWC_GOTO;
    CFWp.cfwParam1  = position;
    int res         = SBIGUnivDrvCommand(CC_CFW, &CFWp, CFWr);
    if (res == CE_NO_ERROR)
    {
        if (CFWp.cfwParam1 == CFWr->cfwPosition)
        {
            LOGF_DEBUG("CFW Reached position %d", CFWr->cfwPosition);
            return res;
        }
        LOG_DEBUG("CFW did not reach position yet, invoking CFWGotoMonitor");
        return CFWGotoMonitor(CFWr);
    }
    LOGF_ERROR("%s: CC_CFW/CFWC_GOTO -> (%s)", __FUNCTION__, GetErrorString(res));
    return res;
}

//==========================================================================

int SBIGCCD::CFWGotoMonitor(CFWResults *CFWr)
{
    int res;
    if (isSimulation())
        return CE_NO_ERROR;
    do
    {
        if ((res = CFWQuery(CFWr)) != CE_NO_ERROR)
            return res;
        switch (CFWr->cfwStatus)
        {
            case CFWS_IDLE:
                LOG_DEBUG("CFW Status Idle.");
                break;
            case CFWS_BUSY:
                LOG_DEBUG("CFW Status Busy.");
                break;
            default:
                LOG_DEBUG("CFW Status Unknown.");
                break;
        }
        sleep(1);
    }
    while (CFWr->cfwStatus != CFWS_IDLE);
    return res;
}

//==========================================================================

int SBIGCCD::GetCFWSelType()
{
    int filnum = IUFindOnSwitchIndex(&FilterTypeSP);
    if (filnum < 0)
    {
        return CFWSEL_UNKNOWN;
    }
    return *(static_cast<uint32_t *>(FilterTypeS[filnum].aux));
}

/*
 Moravian Instruments INDI Driver

 Copyright (C) 2014 Jasem Mutlaq (mutlaqja@ikarustech.com)
 Copyright (C) 2014 Zhirong Li (lzr@qhyccd.com)
 Copyright (C) 2015 Peter Polakovic (peter.polakovic@cloudmakers.eu)
 Copyright (C) 2016 Jakub Smutny (linux@gxccd.com)

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

#include "mi_ccd.h"

#include "config.h"

#include <math.h>
#include <deque>
#include <memory>
#include <utility>

#define TEMP_THRESHOLD  0.2  /* Differential temperature threshold (°C) */
#define TEMP_COOLER_OFF 100  /* High enough temperature for the camera cooler to turn off (°C) */
#define MAX_DEVICES     4    /* Max device cameraCount */
#define MAX_ERROR_LEN   64   /* Max length of error buffer */

// There is _one_ binary for USB and ETH driver, but each binary is renamed
// to its variant (indi_mi_ccd_usb and indi_mi_ccd_eth). The main function will
// fetch from std args the binary name and ISInit will create the appropriate
// driver afterwards.
extern char *__progname;

static char *rtrim(char *str)
{
    if (!str)
        return str;

    char *end = str + strlen(str) - 1;
    while (end >= str && isspace(*end))
        end--;
    *(end + 1) = '\0';
    return str;
}

static class Loader
{
        std::deque<std::unique_ptr<MICCD>> cameras;

    public:
        Loader();

    public:
        std::deque<std::pair<int /* id */, bool /* eth */>> initCameras;

} loader;

Loader::Loader()
{
    if (strstr(__progname, "indi_mi_ccd_eth"))
    {
        gxccd_enumerate_eth([](int id)
        {
            loader.initCameras.emplace_back(id, true);
        });
    }
    else
    {
        // "__progname" shoud be indi_mi_ccd_usb, however accept all names as USB
        gxccd_enumerate_usb([](int id)
        {
            loader.initCameras.emplace_back(id, false);
        });
    }

    for (const auto &args : initCameras)
    {
        cameras.push_back(std::unique_ptr<MICCD>(new MICCD(args.first, args.second)));
    }

    initCameras.clear();
}

MICCD::MICCD(int camId, bool eth) : FilterInterface(this)
{
    cameraId = camId;
    isEth    = eth;

    if (isEth)
        cameraHandle = gxccd_initialize_eth(cameraId);
    else
        cameraHandle = gxccd_initialize_usb(cameraId);
    if (!cameraHandle)
    {
        IDLog("Error connecting MI camera!\n");
        return;
    }

    char sp[MAXINDINAME];
    if (gxccd_get_string_parameter(cameraHandle, GSP_CAMERA_DESCRIPTION, sp, sizeof(sp)) < 0)
    {
        gxccd_get_last_error(cameraHandle, sp, sizeof(sp));
        IDLog("Error getting MI camera info: %s.\n", sp);
        strncpy(name, "MI Camera", MAXINDIDEVICE);
    }
    else
    {
        rtrim(sp);
        strncpy(name, "MI ", MAXINDINAME);
        strncat(name, sp, MAXINDINAME - 3);
        IDLog("Detected camera: %s.\n", name);
    }

    gxccd_get_integer_parameter(cameraHandle, GIP_READ_MODES, &numReadModes);
    gxccd_get_integer_parameter(cameraHandle, GIP_FILTERS, &numFilters);
    gxccd_get_integer_parameter(cameraHandle, GIP_MAX_FAN, &maxFanValue);
    gxccd_get_integer_parameter(cameraHandle, GIP_MAX_WINDOW_HEATING, &maxHeatingValue);
    gxccd_get_integer_parameter(cameraHandle, GIP_MAX_GAIN, &maxGainValue);

    gxccd_release(cameraHandle);
    cameraHandle = nullptr;

    hasGain    = false;
    useShutter = true;

    canDoPreflash = false;

    setDeviceName(name);
    setVersion(INDI_MI_VERSION_MAJOR, INDI_MI_VERSION_MINOR);
}

MICCD::~MICCD()
{
    gxccd_release(cameraHandle);
}

const char *MICCD::getDefaultName()
{
    return name;
}

bool MICCD::initProperties()
{
    INDI::CCD::initProperties();
    INDI::FilterInterface::initProperties(FILTER_TAB);

    FilterSlotNP[0].setMin(1);
    FilterSlotNP[0].setMax(numFilters);

    CaptureFormat mono = {"INDI_MONO", "Mono", 16, true};
    addCaptureFormat(mono);

    IUFillSwitch(&CoolerS[0], "COOLER_ON", "ON", ISS_ON);
    IUFillSwitch(&CoolerS[1], "COOLER_OFF", "OFF", ISS_OFF);
    IUFillSwitchVector(&CoolerSP, CoolerS, 2, getDeviceName(), "CCD_COOLER", "Cooler", MAIN_CONTROL_TAB, IP_WO,
                       ISR_1OFMANY, 0, IPS_IDLE);

    // CCD Regulation power
    IUFillNumber(&CoolerN[0], "CCD_COOLER_VALUE", "Cooling Power (%)", "%+6.2f", 0.0, 1.0, 0.01, 0.0);
    IUFillNumberVector(&CoolerNP, CoolerN, 1, getDeviceName(), "CCD_COOLER_POWER", "Cooling Power", MAIN_CONTROL_TAB,
                       IP_RO, 60, IPS_IDLE);

    // CCD Fan
    IUFillNumber(&FanN[0], "FAN", "Fan speed", "%2.0f", 0, maxFanValue, 1, maxFanValue);
    IUFillNumberVector(&FanNP, FanN, 1, getDeviceName(), "CCD_FAN", "Fan", MAIN_CONTROL_TAB, IP_WO, 60, IPS_IDLE);

    // CCD Window heating
    IUFillNumber(&WindowHeatingN[0], "WINDOW_HEATING", "Heating Intensity", "%2.0f", 0, maxHeatingValue, 1, 0);
    IUFillNumberVector(&WindowHeatingNP, WindowHeatingN, 1, getDeviceName(), "CCD_WINDOW_HEATING", "Window Heating",
                       MAIN_CONTROL_TAB, IP_WO, 60, IPS_IDLE);

    // CCD Gain
    if (maxGainValue > 0) // Camera can set gain
    {
        IUFillNumber(&GainN[0], "GAIN", "Gain", "%2.0f", 0, maxGainValue, 1, 0);
        IUFillNumberVector(&GainNP, GainN, 1, getDeviceName(), "CCD_GAIN", "Gain", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);
    }
    else
    {
        IUFillNumber(&GainN[0], "GAIN", "Gain (e-/ADU)", "%2.2f", 0, 100, 1, 0);
        IUFillNumberVector(&GainNP, GainN, 1, getDeviceName(), "CCD_GAIN", "Gain", MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);
    }

    // Image read mode
    IUFillSwitch(&ReadModeS[0], "READ_MODE_1", "", ISS_OFF);
    IUFillSwitch(&ReadModeS[1], "READ_MODE_2", "", ISS_OFF);
    IUFillSwitch(&ReadModeS[2], "READ_MODE_3", "", ISS_OFF);
    IUFillSwitch(&ReadModeS[3], "READ_MODE_4", "", ISS_OFF);
    IUFillSwitchVector(&ReadModeSP, ReadModeS, numReadModes, getDeviceName(), "CCD_READ_MODE", "Read Mode",
                       MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // NIR Preflash
    IUFillNumber(&PreflashN[0], "NIR_EXPOSURE_TIME", "Preflash duration (s)", "%4.3f", 0.0, 65.535, 0.001, 0.0);
    IUFillNumber(&PreflashN[1], "NIR_CLEAR_NUM", "Num. clear", "%2.0f", 1, 16, 1, 3);
    IUFillNumberVector(&PreflashNP, PreflashN, 2, getDeviceName(), "NIR_PRE_FLASH", "NIR Preflash",
                       MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    addAuxControls();

    setDriverInterface(getDriverInterface() | FILTER_INTERFACE);

    return true;
}

void MICCD::ISGetProperties(const char *dev)
{
    INDI::CCD::ISGetProperties(dev);

    if (isConnected())
    {
        if (HasCooler())
        {
            defineProperty(&CoolerSP);
            defineProperty(&CoolerNP);
        }

        if (numReadModes > 0)
            defineProperty(&ReadModeSP);

        if (maxFanValue > 0)
            defineProperty(&FanNP);

        if (maxHeatingValue > 0)
            defineProperty(&WindowHeatingNP);

        if (hasGain)
            defineProperty(&GainNP);

        if (canDoPreflash)
            defineProperty(&PreflashNP);

        if (numFilters > 0)
        {
            INDI::FilterInterface::updateProperties();
        }
    }
}

bool MICCD::updateProperties()
{
    INDI::CCD::updateProperties();

    if (isConnected())
    {
        if (HasCooler())
        {
            defineProperty(&CoolerSP);
            defineProperty(&CoolerNP);
            temperatureID = IEAddTimer(getCurrentPollingPeriod(), MICCD::updateTemperatureHelper, this);
        }

        if (numReadModes > 0)
            defineProperty(&ReadModeSP);

        if (maxFanValue > 0)
            defineProperty(&FanNP);

        if (maxHeatingValue > 0)
            defineProperty(&WindowHeatingNP);

        if (hasGain)
            defineProperty(&GainNP);

        if (canDoPreflash)
            defineProperty(&PreflashNP);

        if (numFilters > 0)
        {
            INDI::FilterInterface::updateProperties();
        }

        // Let's get parameters now from CCD
        setupParams();

        timerID = SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        if (HasCooler())
        {
            deleteProperty(CoolerSP.name);
            deleteProperty(CoolerNP.name);
            RemoveTimer(temperatureID);
        }

        if (numReadModes > 0)
            deleteProperty(ReadModeSP.name);

        if (maxFanValue > 0)
            deleteProperty(FanNP.name);

        if (maxHeatingValue > 0)
            deleteProperty(WindowHeatingNP.name);

        if (hasGain)
            deleteProperty(GainNP.name);

        if (canDoPreflash)
            deleteProperty(PreflashNP.name);

        if (numFilters > 0)
        {
            INDI::FilterInterface::updateProperties();
        }
        RemoveTimer(timerID);
    }

    return true;
}

bool MICCD::Connect()
{
    uint32_t cap = 0;

    if (isSimulation())
    {
        LOGF_INFO("Connected to %s", name);

        cap = CCD_CAN_SUBFRAME | CCD_CAN_ABORT | CCD_CAN_BIN | CCD_HAS_SHUTTER | CCD_HAS_COOLER;
        SetCCDCapability(cap);

        numFilters = 5;

        return true;
    }

    if (!cameraHandle)
    {
        if (isEth)
            cameraHandle = gxccd_initialize_eth(cameraId);
        else
            cameraHandle = gxccd_initialize_usb(cameraId);
    }
    if (!cameraHandle)
    {
        LOGF_ERROR("Error connecting to %s.", name);
        return false;
    }

    LOGF_INFO("Connected to %s.", name);

    bool value;
    cap = CCD_CAN_ABORT | CCD_CAN_BIN;

    gxccd_get_boolean_parameter(cameraHandle, GBP_SUB_FRAME, &value);
    if (value)
        cap |= CCD_CAN_SUBFRAME;

    gxccd_get_boolean_parameter(cameraHandle, GBP_GUIDE, &value);
    if (value)
        cap |= CCD_HAS_ST4_PORT;

    gxccd_get_boolean_parameter(cameraHandle, GBP_SHUTTER, &value);
    if (value)
        cap |= CCD_HAS_SHUTTER;

    gxccd_get_boolean_parameter(cameraHandle, GBP_COOLER, &value);
    if (value)
        cap |= CCD_HAS_COOLER;

    gxccd_get_boolean_parameter(cameraHandle, GBP_GAIN, &hasGain);

    gxccd_get_boolean_parameter(cameraHandle, GBP_PREFLASH, &canDoPreflash);

    SetCCDCapability(cap);

    gxccd_get_integer_parameter(cameraHandle, GIP_MAX_BINNING_X, &maxBinX);
    gxccd_get_integer_parameter(cameraHandle, GIP_MAX_BINNING_Y, &maxBinY);
    PrimaryCCD.setMinMaxStep("CCD_BINNING", "HOR_BIN", 1, maxBinX, 1, false);
    PrimaryCCD.setMinMaxStep("CCD_BINNING", "VER_BIN", 1, maxBinY, 1, false);

    if (numReadModes > 0)
    {
        int defaultRM = 0;
        gxccd_get_integer_parameter(cameraHandle, GIP_DEFAULT_READ_MODE, &defaultRM);
        for (int i = 0; i < numReadModes; i++)
        {
            gxccd_enumerate_read_modes(cameraHandle, i, ReadModeS[i].label, MAXINDILABEL);
            if (i == defaultRM)
                ReadModeS[i].s = ISS_ON;
        }
        IDSetSwitch(&ReadModeSP, nullptr);
    }
    return true;
}

bool MICCD::Disconnect()
{
    LOGF_INFO("Disconnected from %s.", name);
    gxccd_release(cameraHandle);
    cameraHandle = nullptr;
    return true;
}

bool MICCD::setupParams()
{
    bool sim = isSimulation();
    if (sim)
    {
        SetCCDParams(4032, 2688, 16, 9, 9);
    }
    else
    {
        int chipW, chipD, pixelW, pixelD;
        gxccd_get_integer_parameter(cameraHandle, GIP_CHIP_W, &chipW);
        gxccd_get_integer_parameter(cameraHandle, GIP_CHIP_D, &chipD);
        gxccd_get_integer_parameter(cameraHandle, GIP_PIXEL_W, &pixelW);
        gxccd_get_integer_parameter(cameraHandle, GIP_PIXEL_D, &pixelD);

        SetCCDParams(chipW, chipD, 16, pixelW / 1000.0, pixelD / 1000.0);
    }

    PrimaryCCD.setFrameBufferSize(PrimaryCCD.getXRes() * PrimaryCCD.getYRes() * PrimaryCCD.getBPP() / 8);

    int expTime = 0;
    gxccd_get_integer_parameter(cameraHandle, GIP_MINIMAL_EXPOSURE, &expTime);
    minExpTime = expTime / 1000000.0; // convert to seconds
    PrimaryCCD.setMinMaxStep("CCD_EXPOSURE", "CCD_EXPOSURE_VALUE", minExpTime, 3600, 1, true);

    if (!sim && maxGainValue <= 0)
    {
        float gain = 0;
        if (gxccd_get_value(cameraHandle, GV_ADC_GAIN, &gain) < 0)
        {
            char errorStr[MAX_ERROR_LEN];
            gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
            LOGF_ERROR("Getting gain failed: %s.", errorStr);
            GainN[0].value = 0;
            GainNP.s       = IPS_ALERT;
            IDSetNumber(&GainNP, nullptr);
            return false;
        }
        else
        {
            GainN[0].value = gain;
            GainNP.s       = IPS_OK;
            IDSetNumber(&GainNP, nullptr);
        }
    }

    if (!sim && canDoPreflash)
    {
        if (gxccd_set_preflash(cameraHandle, PreflashN[0].value, PreflashN[1].value) < 0)
        {
            char errorStr[MAX_ERROR_LEN];
            gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
            LOGF_ERROR("Setting default NIR preflash value failed: %s.", errorStr);
            PreflashNP.s = IPS_ALERT;
        }
    }

    return true;
}

int MICCD::SetTemperature(double temperature)
{
    // If there difference, for example, is less than TEMP_THRESHOLD degrees, let's immediately return OK.
    if (fabs(temperature - TemperatureNP[0].getValue()) < TEMP_THRESHOLD)
        return 1;

    TemperatureRequest = temperature;

    if (!isSimulation() && gxccd_set_temperature(cameraHandle, temperature) < 0)
    {
        char errorStr[MAX_ERROR_LEN];
        gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
        LOGF_ERROR("Setting temperature failed: %s.", errorStr);
        return -1;
    }

    return 0;
}

bool MICCD::StartExposure(float duration)
{
    imageFrameType = PrimaryCCD.getFrameType();
    useShutter = (imageFrameType == INDI::CCDChip::LIGHT_FRAME || imageFrameType == INDI::CCDChip::FLAT_FRAME);

    if (!isSimulation())
    {
        int mode = IUFindOnSwitchIndex(&ReadModeSP);
        gxccd_set_read_mode(cameraHandle, mode);

        // send binned coords
        int x = PrimaryCCD.getSubX() / PrimaryCCD.getBinX();
        int y = PrimaryCCD.getSubY() / PrimaryCCD.getBinY();
        int w = PrimaryCCD.getSubW() / PrimaryCCD.getBinX();
        int d = PrimaryCCD.getSubH() / PrimaryCCD.getBinY();
        // invert frame, libgxccd has 0 on the bottom
        int fd = PrimaryCCD.getYRes() / PrimaryCCD.getBinY();
        int fy = fd - y - d;
        gxccd_start_exposure(cameraHandle, duration, useShutter, x, fy, w, d);
    }

    ExposureRequest = duration;
    PrimaryCCD.setExposureDuration(duration);

    gettimeofday(&ExpStart, nullptr);
    InExposure  = true;
    downloading = false;
    LOGF_DEBUG("Taking a %.3f seconds frame...", ExposureRequest);
    return true;
}

bool MICCD::AbortExposure()
{
    if (InExposure && !isSimulation())
    {
        if (gxccd_abort_exposure(cameraHandle, false) < 0)
        {
            char errorStr[MAX_ERROR_LEN];
            gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
            LOGF_ERROR("Aborting exposure failed: %s.", errorStr);
            return false;
        }
    }

    InExposure  = false;
    downloading = false;
    LOG_INFO("Exposure aborted.");
    return true;
}

bool MICCD::UpdateCCDFrame(int x, int y, int w, int h)
{
    /* Add the X and Y offsets */
    long x_1 = x / PrimaryCCD.getBinX();
    long y_1 = y / PrimaryCCD.getBinY();

    long x_2 = x_1 + (w / PrimaryCCD.getBinX());
    long y_2 = y_1 + (h / PrimaryCCD.getBinY());

    if (x_2 > PrimaryCCD.getXRes() / PrimaryCCD.getBinX())
    {
        LOGF_ERROR("Error: Requested width out of bounds %ld", x_2);
        return false;
    }
    else if (y_2 > PrimaryCCD.getYRes() / PrimaryCCD.getBinY())
    {
        LOGF_ERROR("Error: Requested height out of bounds %ld", y_2);
        return false;
    }

    LOGF_DEBUG("The Final image area is (%ld, %ld), (%ld, %ld)\n", x_1, y_1, x_2, y_2);

    int imageWidth  = x_2 - x_1;
    int imageHeight = y_2 - y_1;

    // Set UNBINNED coords
    PrimaryCCD.setFrame(x, y, w, h);
    PrimaryCCD.setFrameBufferSize(imageWidth * imageHeight * PrimaryCCD.getBPP() / 8);
    return true;
}

bool MICCD::UpdateCCDBin(int hor, int ver)
{
    if (hor < 1 || hor > maxBinX || ver < 1 || ver > maxBinY)
    {
        LOGF_ERROR("Binning (%dx%d) are out of range. Range from (1x1) to (%dx%d)",
                   hor, ver, maxBinX, maxBinY);
        return false;
    }
    if (gxccd_set_binning(cameraHandle, hor, ver) < 0)
    {
        char errorStr[MAX_ERROR_LEN];
        gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
        LOGF_ERROR("Setting binning failed: %s.", errorStr);
        return false;
    }
    PrimaryCCD.setBin(hor, ver);
    return UpdateCCDFrame(PrimaryCCD.getSubX(), PrimaryCCD.getSubY(), PrimaryCCD.getSubW(), PrimaryCCD.getSubH());
}

float MICCD::calcTimeLeft()
{
    double timesince;
    struct timeval now;
    gettimeofday(&now, nullptr);

    timesince = (now.tv_sec * 1000.0 + now.tv_usec / 1000.0) - (ExpStart.tv_sec * 1000.0 + ExpStart.tv_usec / 1000.0);
    return ExposureRequest - timesince / 1000.0;
}

static void mirror_image(void *buf, size_t w, size_t d)
{
    size_t w2     = w * 2;
    size_t half_d = d / 2;

    for (size_t line = 1; line <= half_d; line++)
    {
        uint16_t *sa = (uint16_t *)((char *)buf + (line - 1) * w2);
        uint16_t *da = (uint16_t *)((char *)buf + (d - line) * w2);
        for (size_t index = 1; index <= w; index++)
        {
            uint16_t tmp = *sa;
            *sa          = *da;
            *da          = tmp;
            ++sa;
            ++da;
        }
    }
}

/* Downloads the image from the CCD. */
int MICCD::grabImage()
{
    std::unique_lock<std::mutex> guard(ccdBufferLock);
    int ret              = 0;
    unsigned char *image = (unsigned char *)PrimaryCCD.getFrameBuffer();
    int width  = PrimaryCCD.getSubW() / PrimaryCCD.getBinX();
    int height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY();

    if (isSimulation())
    {
        uint16_t *buffer = (uint16_t *)image;

        for (int i = 0; i < height; i++)
            for (int j = 0; j < width; j++)
                buffer[i * width + j] = rand() % UINT16_MAX;
    }
    else
    {
        ret = gxccd_read_image(cameraHandle, image, PrimaryCCD.getFrameBufferSize());
        if (ret < 0)
        {
            char errorStr[MAX_ERROR_LEN];
            gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
            LOGF_ERROR("Error getting image: %s.", errorStr);
        }
        else
        {
            mirror_image(image, width, height);
        }
    }

    guard.unlock();

    if (ExposureRequest > 5 && !ret)
        LOG_INFO("Download complete.");

    downloading = false;
    ExposureComplete(&PrimaryCCD);

    return ret;
}

void MICCD::TimerHit()
{
    if (!isConnected())
        return; // No need to reset timer if we are not connected anymore

    if (InExposure)
    {
        float timeleft = calcTimeLeft();
        bool ready     = false;

        if (!downloading && (gxccd_image_ready(cameraHandle, &ready) < 0))
        {
            char errorStr[MAX_ERROR_LEN];
            gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
            LOGF_ERROR("Getting image ready failed: %s.", errorStr);
        }
        if (ready)
        {
            PrimaryCCD.setExposureLeft(0);
            InExposure  = false;
            downloading = true;

            // Don't spam the session log unless it is a long exposure > 5 seconds
            if (ExposureRequest > 5)
                LOG_INFO("Exposure done, downloading image...");

            // grab and save image
            grabImage();
        }
        // camera may need some time for image download -> update client only for positive values
        else if (timeleft >= 0)
        {
            LOGF_DEBUG("Exposure in progress: Time left %.2fs", timeleft);
            PrimaryCCD.setExposureLeft(timeleft);
        }
    }

    SetTimer(getCurrentPollingPeriod());
}

int MICCD::QueryFilter()
{
    return CurrentFilter;
}

bool MICCD::SelectFilter(int position)
{
    if (!isSimulation() && gxccd_set_filter(cameraHandle, position - 1) < 0)
    {
        char errorStr[MAX_ERROR_LEN];
        gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
        LOGF_ERROR("Setting filter failed: %s.", errorStr);
        return false;
    }

    CurrentFilter = position;
    SelectFilterDone(position);
    LOGF_DEBUG("Filter changed to %d", position);
    return true;
}

IPState MICCD::GuideNorth(uint32_t ms)
{
    if (gxccd_move_telescope(cameraHandle, 0, static_cast<int16_t>(ms)) < 0)
    {
        char errorStr[MAX_ERROR_LEN];
        gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
        LOGF_ERROR("GuideNorth() failed: %s.", errorStr);
        return IPS_ALERT;
    }
    return IPS_OK;
}

IPState MICCD::GuideSouth(uint32_t ms)
{
    if (gxccd_move_telescope(cameraHandle, 0, (-1 * static_cast<int16_t>(ms))) < 0)
    {
        char errorStr[MAX_ERROR_LEN];
        gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
        LOGF_ERROR("GuideSouth() failed: %s.", errorStr);
        return IPS_ALERT;
    }
    return IPS_OK;
}

IPState MICCD::GuideEast(uint32_t ms)
{
    if (gxccd_move_telescope(cameraHandle, (-1 * static_cast<int16_t>(ms)), 0) < 0)
    {
        char errorStr[MAX_ERROR_LEN];
        gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
        LOGF_ERROR("GuideEast() failed: %s.", errorStr);
        return IPS_ALERT;
    }
    return IPS_OK;
}

IPState MICCD::GuideWest(uint32_t ms)
{
    if (gxccd_move_telescope(cameraHandle, static_cast<int16_t>(ms), 0) < 0)
    {
        char errorStr[MAX_ERROR_LEN];
        gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
        LOGF_ERROR("GuideWest() failed: %s.", errorStr);
        return IPS_ALERT;
    }
    return IPS_OK;
}

bool MICCD::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(name, ReadModeSP.name))
        {
            IUUpdateSwitch(&ReadModeSP, states, names, n);
            ReadModeSP.s = IPS_OK;
            IDSetSwitch(&ReadModeSP, nullptr);
            return true;
        }
        else if (!strcmp(name, CoolerSP.name))
        {

            IUUpdateSwitch(&CoolerSP, states, names, n);
            CoolerSP.s = IPS_OK;

            if (HasCooler() && !isSimulation())
            {
                bool on = !IUFindOnSwitchIndex(&CoolerSP);
                double temp = on ? TemperatureRequest : TEMP_COOLER_OFF;

                if (gxccd_set_temperature(cameraHandle, temp) < 0)
                {
                    char errorStr[MAX_ERROR_LEN];
                    gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
                    LOGF_ERROR("Setting temperature failed: %s.", errorStr);
                    CoolerSP.s = IPS_ALERT;
                }
            }

            IDSetSwitch(&CoolerSP, nullptr);
            return true;
        }
    }

    return INDI::CCD::ISNewSwitch(dev, name, states, names, n);
}

bool MICCD::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0)
    {
        if (INDI::FilterInterface::processText(dev, name, texts, names, n))
            return true;

    }

    return INDI::CCD::ISNewText(dev, name, texts, names, n);
}

bool MICCD::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0)
    {
        if (INDI::FilterInterface::processNumber(dev, name, values, names, n))
            return true;


        if (!strcmp(name, FanNP.name))
        {
            IUUpdateNumber(&FanNP, values, names, n);

            if (!isSimulation() && gxccd_set_fan(cameraHandle, FanN[0].value) < 0)
            {
                char errorStr[MAX_ERROR_LEN];
                gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
                LOGF_ERROR("Setting fan failed: %s.", errorStr);
                FanNP.s = IPS_ALERT;
            }
            else
            {
                FanNP.s = IPS_OK;
            }

            IDSetNumber(&FanNP, nullptr);
            return true;
        }

        if (!strcmp(name, WindowHeatingNP.name))
        {
            IUUpdateNumber(&WindowHeatingNP, values, names, n);

            if (!isSimulation() && gxccd_set_window_heating(cameraHandle, WindowHeatingN[0].value) < 0)
            {
                char errorStr[MAX_ERROR_LEN];
                gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
                LOGF_ERROR("Setting heating failed: %s.", errorStr);
                WindowHeatingNP.s = IPS_ALERT;
            }
            else
            {
                WindowHeatingNP.s = IPS_OK;
            }

            IDSetNumber(&WindowHeatingNP, nullptr);
            return true;
        }

        if (!strcmp(name, PreflashNP.name))
        {
            IUUpdateNumber(&PreflashNP, values, names, n);

            // set NIR pre-flash if available.
            if (canDoPreflash)
            {
                if (!isSimulation() && gxccd_set_preflash(cameraHandle, PreflashN[0].value, PreflashN[1].value) < 0)
                {
                    char errorStr[MAX_ERROR_LEN];
                    gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
                    LOGF_ERROR("Setting NIR preflash failed: %s.", errorStr);
                    PreflashNP.s = IPS_ALERT;
                }
                else
                {
                    PreflashNP.s = IPS_OK;
                }
            }

            IDSetNumber(&PreflashNP, nullptr);
            return true;
        }

        if (!strcmp(name, GainNP.name))
        {
            IUUpdateNumber(&GainNP, values, names, n);

            if (!isSimulation() && gxccd_set_gain(cameraHandle, static_cast<uint16_t>(GainN[0].value)) < 0)
            {
                char errorStr[MAX_ERROR_LEN];
                gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
                LOGF_ERROR("Setting gain failed: %s.", errorStr);
                GainNP.s = IPS_ALERT;
            }
            else
            {
                GainNP.s = IPS_OK;
            }

            IDSetNumber(&GainNP, nullptr);
            return true;
        }
    }

    return INDI::CCD::ISNewNumber(dev, name, values, names, n);
}

void MICCD::updateTemperatureHelper(void *p)
{
    if (static_cast<MICCD *>(p)->isConnected())
        static_cast<MICCD *>(p)->updateTemperature();
}

void MICCD::updateTemperature()
{
    float ccdtemp  = 0;
    float ccdpower = 0;
    int err        = 0;

    if (isSimulation())
    {
        ccdtemp = TemperatureNP[0].getValue();
        if (TemperatureNP[0].getValue() < TemperatureRequest)
            ccdtemp += TEMP_THRESHOLD;
        else if (TemperatureNP[0].getValue() > TemperatureRequest)
            ccdtemp -= TEMP_THRESHOLD;

        ccdpower = 30;
    }
    else
    {
        if (gxccd_get_value(cameraHandle, GV_CHIP_TEMPERATURE, &ccdtemp) < 0)
        {
            char errorStr[MAX_ERROR_LEN];
            gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
            LOGF_ERROR("Getting temperature failed: %s.", errorStr);
            err |= 1;
        }
        if (gxccd_get_value(cameraHandle, GV_POWER_UTILIZATION, &ccdpower) < 0)
        {
            char errorStr[MAX_ERROR_LEN];
            gxccd_get_last_error(cameraHandle, errorStr, sizeof(errorStr));
            LOGF_ERROR("Getting voltage failed: %s.", errorStr);
            err |= 2;
        }
    }

    TemperatureNP[0].setValue(ccdtemp);
    CoolerN[0].value      = ccdpower * 100.0;

    //    if (TemperatureNP.s == IPS_BUSY && fabs(TemperatureN[0].value - TemperatureRequest) <= TEMP_THRESHOLD)
    //    {
    //        // end of temperature ramp
    //        TemperatureN[0].value = TemperatureRequest;
    //        TemperatureNP.s       = IPS_OK;
    //    }

    if (err)
    {
        if (err & 1)
            TemperatureNP.setState(IPS_ALERT);
        if (err & 2)
            CoolerNP.s = IPS_ALERT;
    }
    else
    {
        CoolerNP.s = IPS_OK;
    }

    TemperatureNP.apply();
    IDSetNumber(&CoolerNP, nullptr);
    temperatureID = IEAddTimer(getCurrentPollingPeriod(), MICCD::updateTemperatureHelper, this);
}

bool MICCD::saveConfigItems(FILE *fp)
{
    INDI::CCD::saveConfigItems(fp);

    IUSaveConfigSwitch(fp, &ReadModeSP);

    if (numFilters > 0)
    {
        INDI::FilterInterface::saveConfigItems(fp);
    }

    if (maxFanValue > 0)
        IUSaveConfigNumber(fp, &FanNP);

    if (maxHeatingValue > 0)
        IUSaveConfigNumber(fp, &WindowHeatingNP);

    if (maxGainValue > 0)
        IUSaveConfigNumber(fp, &GainNP);

    return true;
}

void MICCD::addFITSKeywords(INDI::CCDChip *targetChip, std::vector<INDI::FITSRecord> &fitsKeywords)
{
    INDI::CCD::addFITSKeywords(targetChip, fitsKeywords);

    char svalue[256];
    int ivalue = 0;

    if (hasGain)
        fitsKeywords.push_back({"GAIN", GainN[0].value, 3, "Gain"});

    if (!gxccd_get_integer_parameter(cameraHandle, GIP_MAX_PIXEL_VALUE, &ivalue))
        fitsKeywords.push_back({"DATAMAX", ivalue, nullptr});

    if (numReadModes > 0)
    {
        ivalue = IUFindOnSwitchIndex(&ReadModeSP);
        strncpy(svalue, ReadModeS[ivalue].label, sizeof(svalue));
    }
    else
    {
        ivalue = 0;
        strncpy(svalue, "No read mode", sizeof(svalue));
    }
    fitsKeywords.push_back({"READMODE", ivalue, svalue});

    if (!gxccd_get_string_parameter(cameraHandle, GSP_CHIP_DESCRIPTION, svalue, 256))
    {
        rtrim(svalue);
        fitsKeywords.push_back({"CHIPTYPE", svalue, nullptr});

        if (!strcmp(svalue, "GSENSE4040"))
        {
            // we use hardcoded values here, because:
            // - so far there is no possibility to read / set HDR threshold in libgxccd
            // - it's not even easy to find out if the camera supports HDR...
            fitsKeywords.push_back({"HDRTHRES", 3600, nullptr});
        }
    }

    if (canDoPreflash)
    {
        fitsKeywords.push_back({"PREFLASH", PreflashN[0].value, 3, "seconds"});
        fitsKeywords.push_back({"NUM-CLR", PreflashN[1].value, 3, nullptr});
    }
}

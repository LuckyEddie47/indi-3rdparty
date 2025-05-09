#if 0
FLI PDF
INDI Interface for Finger Lakes Instrument Focusers
Copyright (C) 2003 - 2012 Jasem Mutlaq (mutlaqja@ikarustech.com)

    This library is free software;
you can redistribute it and / or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation;
either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY;
without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library;
if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110 - 1301  USA

#endif

#include "fli_pdf.h"

#include "indidevapi.h"
#include "eventloop.h"

#include <memory>
#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>

static std::unique_ptr<FLIPDF> fliPDF(new FLIPDF());

const flidomain_t Domains[] = { FLIDOMAIN_USB, FLIDOMAIN_SERIAL, FLIDOMAIN_PARALLEL_PORT, FLIDOMAIN_INET };

FLIPDF::FLIPDF()
{
    sim = false;

    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE);
}

const char *FLIPDF::getDefaultName()
{
    return "FLI PDF";
}

bool FLIPDF::initProperties()
{
    // Init parent properties first
    INDI::Focuser::initProperties();

    IUFillSwitch(&PortS[0], "USB", "USB", ISS_ON);
    IUFillSwitch(&PortS[1], "SERIAL", "Serial", ISS_OFF);
    IUFillSwitch(&PortS[2], "PARALLEL", "Parallel", ISS_OFF);
    IUFillSwitch(&PortS[3], "INET", "INet", ISS_OFF);
    IUFillSwitchVector(&PortSP, PortS, 4, getDeviceName(), "PORTS", "Port", MAIN_CONTROL_TAB, IP_WO, ISR_1OFMANY, 0,
                       IPS_IDLE);

    IUFillSwitch(&HomeS[0], "Go", "", ISS_OFF);
    IUFillSwitchVector(&HomeSP, HomeS, 1, getDeviceName(), "Home", "", MAIN_CONTROL_TAB, IP_WO, ISR_1OFMANY, 0,
                       IPS_IDLE);

    IUFillText(&FocusInfoT[0], "Model", "", "");
    IUFillText(&FocusInfoT[1], "HW Rev", "", "");
    IUFillText(&FocusInfoT[2], "FW Rev", "", "");
    IUFillTextVector(&FocusInfoTP, FocusInfoT, 3, getDeviceName(), "Model", "", "Focuser Info", IP_RO, 60, IPS_IDLE);
    return true;
}

void FLIPDF::ISGetProperties(const char *dev)
{
    INDI::Focuser::ISGetProperties(dev);

    defineProperty(&PortSP);

    addAuxControls();
}

bool FLIPDF::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineProperty(FocusAbsPosNP);
        defineProperty(FocusRelPosNP);
        defineProperty(&HomeSP);
        defineProperty(&FocusInfoTP);
        setupParams();

        timerID = SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(FocusAbsPosNP);
        deleteProperty(FocusRelPosNP);
        deleteProperty(HomeSP.name);
        deleteProperty(FocusInfoTP.name);

        rmTimer(timerID);
    }

    return true;
}

bool FLIPDF::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0)
    {
        /* Home */
        if (!strcmp(name, HomeSP.name))
        {
            if (IUUpdateSwitch(&HomeSP, states, names, n) < 0)
                return false;

            goHomePosition();
            return true;
        }

        /* Ports */
        if (!strcmp(name, PortSP.name))
        {
            if (IUUpdateSwitch(&PortSP, states, names, n) < 0)
                return false;

            PortSP.s = IPS_OK;
            IDSetSwitch(&PortSP, nullptr);
            return true;
        }
    }

    //  Nobody has claimed this, so, ignore it
    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool FLIPDF::Connect()
{
    int err = 0;

    IDMessage(getDeviceName(), "Attempting to find the FLI PDF...");

    sim = isSimulation();

    if (sim)
        return true;

    int portSwitchIndex = IUFindOnSwitchIndex(&PortSP);

    if (findFLIPDF(Domains[portSwitchIndex]) == false)
    {
        LOG_ERROR("Error: no focusers were detected.");
        return false;
    }

    if ((err = FLIOpen(&fli_dev, FLIFocus.name, FLIDEVICE_FOCUSER | FLIFocus.domain)))
    {
        LOGF_ERROR("Error: FLIOpen() failed. %s.", strerror((int) - err));
        return false;
    }

    /* Success! */
    LOG_INFO("Focuser is online. Retrieving basic data.");
    return true;
}

bool FLIPDF::Disconnect()
{
    int err;

    if (sim)
        return true;

    if ((err = FLIClose(fli_dev)))
    {
        LOGF_ERROR("Error: FLIClose() failed. %s.", strerror((int) - err));

        return false;
    }

    LOG_INFO("Focuser is offline.");
    return true;
}

bool FLIPDF::setupParams()
{
    int err = 0;

    char hw_rev[16], fw_rev[16];

    //////////////////////
    // 1. Get Focuser Model
    //////////////////////
    if (!sim && (err = FLIGetModel(fli_dev, FLIFocus.model, 200))) //ToDo: lazy
    {
        LOGF_ERROR("FLIGetModel() failed. %s.", strerror((int) - err));
        return false;
    }

    if (sim)
        IUSaveText(&FocusInfoT[0], getDeviceName());
    else
        IUSaveText(&FocusInfoT[0], FLIFocus.model);

    ///////////////////////////
    // 2. Get Hardware revision
    ///////////////////////////
    if (sim)
        FLIFocus.HWRevision = 1;
    else if ((err = FLIGetHWRevision(fli_dev, &FLIFocus.HWRevision)))
    {
        LOGF_ERROR("FLIGetHWRevision() failed. %s.", strerror((int) - err));

        if (isDebug())
            IDLog("FLIGetHWRevision() failed. %s.\n", strerror((int) - err));

        return false;
    }

    snprintf(hw_rev, 16, "%ld", FLIFocus.HWRevision);
    IUSaveText(&FocusInfoT[1], hw_rev);

    ///////////////////////////
    // 3. Get Firmware revision
    ///////////////////////////
    if (sim)
        FLIFocus.FWRevision = 1;
    else if ((err = FLIGetFWRevision(fli_dev, &FLIFocus.FWRevision)))
    {
        LOGF_ERROR("FLIGetFWRevision() failed. %s.", strerror((int) - err));
        return false;
    }

    snprintf(fw_rev, 16, "%ld", FLIFocus.FWRevision);
    IUSaveText(&FocusInfoT[2], fw_rev);

    IDSetText(&FocusInfoTP, nullptr);
    ///////////////////////////
    // 4. Focuser position
    ///////////////////////////
    if (sim)
        FLIFocus.current_pos = 3500;
    else if ((err = FLIGetStepperPosition(fli_dev, &FLIFocus.current_pos)))
    {
        LOGF_ERROR("FLIGetStepperPosition() failed. %s.", strerror((int) - err));
        return false;
    }

    ///////////////////////////
    // 5. Focuser max limit
    ///////////////////////////
    if (sim)
        FLIFocus.max_pos = 50000;
    else if ((err = FLIGetFocuserExtent(fli_dev, &FLIFocus.max_pos)))
    {
        LOGF_ERROR("FLIGetFocuserExtent() failed. %s.", strerror((int) - err));
        return false;
    }

    FocusAbsPosNP[0].setMin(1);
    FocusAbsPosNP[0].setMax(FLIFocus.max_pos);
    FocusAbsPosNP[0].setValue(FLIFocus.current_pos);

    FocusAbsPosNP.updateMinMax();
    LOG_INFO("Setting initial absolute position");

    FocusRelPosNP[0].setMin(1.);
    FocusRelPosNP[0].setMax(FLIFocus.max_pos);
    FocusRelPosNP[0].setValue(0.);

    FocusRelPosNP.updateMinMax();
    LOG_INFO("Setting initial relative position");

    /////////////////////////////////////////
    // 6. Focuser speed is set to 100 tick/sec
    //////////////////////////////////////////
    FocusSpeedNP[0].setValue(100);
    LOG_INFO("Setting initial speed");

    return true;
}

void FLIPDF::goHomePosition()
{
    int err = 0;

    if (!sim && (err = FLIHomeFocuser(fli_dev)))
    {
        LOGF_ERROR("FLIHomeFocuser() failed. %s.", strerror((int) - err));
        return;
    }

    HomeSP.s = IPS_OK;
    IUResetSwitch(&HomeSP);
    IDSetSwitch(&HomeSP, "Moving to home position...");
}

void FLIPDF::TimerHit()
{
    int timerID = -1;
    int err     = 0;

    if (isConnected() == false)
        return; //  No need to Home timer if we are not connected anymore

    if (InStep)
    {
        if (sim)
        {
            if (FLIFocus.current_pos < StepRequest)
            {
                FLIFocus.current_pos += 250;
                if (FLIFocus.current_pos > StepRequest)
                    FLIFocus.current_pos = StepRequest;
            }
            else
            {
                FLIFocus.current_pos -= 250;
                if (FLIFocus.current_pos < StepRequest)
                    FLIFocus.current_pos = StepRequest;
            }
        }
        // while moving, display the remaing steps
        else if ((err = FLIGetStepsRemaining(fli_dev, &FLIFocus.steps_remaing)))
        {
            LOGF_ERROR("FLIGetStepsRemaining() failed. %s.", strerror((int) - err));
            SetTimer(getCurrentPollingPeriod());
            return;
        }
        if (!FLIFocus.steps_remaing)
        {
            InStep          = false;
            FocusAbsPosNP.setState(IPS_OK);
            if (FocusRelPosNP.getState() == IPS_BUSY)
            {
                FocusRelPosNP.setState(IPS_OK);
                FocusRelPosNP.apply();
            }
        }

        if ((err = FLIGetStepperPosition(fli_dev, &FLIFocus.current_pos)))
        {
            LOGF_ERROR("FLIGetStepperPosition() failed. %s.", strerror((int) - err));
            SetTimer(getCurrentPollingPeriod());
            return;
        }
        FocusAbsPosNP[0].setValue(FLIFocus.current_pos);
        FocusAbsPosNP.apply();
    }
    else // we need to display the current position after move finished
    {
        if ((err = FLIGetStepperPosition(fli_dev, &FLIFocus.current_pos)))
        {
            LOGF_ERROR("FLIGetStepperPosition() failed. %s.", strerror((int) - err));
            return;
        }
        FocusAbsPosNP[0].setValue(FLIFocus.current_pos);
        FocusAbsPosNP.apply();
    }

    if (timerID == -1)
        SetTimer(getCurrentPollingPeriod());
    return;
}

IPState FLIPDF::MoveAbsFocuser(uint32_t targetTicks)
{
    int err = 0;

    if (targetTicks < FocusAbsPosNP[0].getMin() || targetTicks > FocusAbsPosNP[0].getMax())
    {
        LOG_ERROR("Error, requested absolute position is out of range.");
        return IPS_ALERT;
    }
    long current;
    if ((err = FLIGetStepperPosition(fli_dev, &current)))
    {
        LOGF_ERROR("FLIPDF::MoveAbsFocuser: FLIGetStepperPosition() failed. %s.", strerror((int) - err));
        return IPS_ALERT;
    }
    err = FLIStepMotorAsync(fli_dev, (targetTicks - current));
    if (!sim && (err))
    {
        LOGF_ERROR("FLIStepMotor() failed. %s.", strerror((int) - err));
        return IPS_ALERT;
    }

    StepRequest = targetTicks;
    InStep      = true;

    // We are still moving, didn't reach target yet
    return IPS_BUSY;
}

IPState FLIPDF::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    long cur_rpos = 0;
    long new_rpos = 0;

    cur_rpos = FLIFocus.current_pos;

    if (dir == FOCUS_INWARD)
        new_rpos = cur_rpos + ticks;
    else
        new_rpos = cur_rpos - ticks;

    return MoveAbsFocuser(new_rpos);
}

bool FLIPDF::findFLIPDF(flidomain_t domain)
{
    char **names;
    long err;

    LOGF_DEBUG("In find Focuser, the domain is %ld", domain);

    if ((err = FLIList(domain | FLIDEVICE_FOCUSER, &names)))
    {
        LOGF_ERROR("FLIList() failed. %s", strerror((int) - err));
        return false;
    }

    if (names != nullptr && names[0] != nullptr)
    {
        for (int i = 0; names[i] != nullptr; i++)
        {
            for (int j = 0; names[i][j] != '\0'; j++)
                if (names[i][j] == ';')
                {
                    names[i][j] = '\0';
                    break;
                }
        }

        FLIFocus.domain = domain;

        switch (domain)
        {
            case FLIDOMAIN_PARALLEL_PORT:
                FLIFocus.dname = strdup("parallel port");
                break;

            case FLIDOMAIN_USB:
                FLIFocus.dname = strdup("USB");
                break;

            case FLIDOMAIN_SERIAL:
                FLIFocus.dname = strdup("serial");
                break;

            case FLIDOMAIN_INET:
                FLIFocus.dname = strdup("inet");
                break;

            default:
                FLIFocus.dname = strdup("Unknown domain");
        }

        FLIFocus.name = strdup(names[0]);

        if ((err = FLIFreeList(names)))
        {
            LOGF_ERROR("FLIFreeList() failed. %s.", strerror((int) - err));
            return false;
        }

    } /* end if */
    else
    {
        if ((err = FLIFreeList(names)))
        {
            LOGF_ERROR("FLIFreeList() failed. %s.", strerror((int) - err));
            return false;
        }

        return false;
    }

    LOG_DEBUG("FindFLIPDF() finished successfully.");

    return true;
}

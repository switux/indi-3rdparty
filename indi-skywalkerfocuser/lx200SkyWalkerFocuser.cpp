/*
    MagnetDrives SkyWalker Focuser driver

    Copyright (C) 2021 F. Lesage

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

/*
 * Portions of this code come (or have been inspired by) from drivers for:
 *  AOK Skywalker mount by T.Schriber
 *  Avalon StarGo focuser by C.Contaxis and W.Reissenberger
 */

#include "lx200SkyWalkerFocuser.h"

#include <algorithm>
//#include <cmath>
//#include <cstdio>
//#include <memory>
//#include <cstring>
//#include <unistd.h>
//#ifndef _WIN32
//#include <termios.h>
//#endif

#include "config.h"

/*
 *
 * Required by the INDI framework
 *
 */

static std::unique_ptr<LX200SkyWalkerFocuser> focuser(new LX200SkyWalkerFocuser());

void ISGetProperties(const char *dev)
{
    focuser->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    focuser->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    focuser->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    focuser->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}
void ISSnoopDevice(XMLEle *root)
{
    focuser->ISSnoopDevice(root);
}

/*
 * ------------------------------------------------------------------------
 *
 * Start of the show
 *
 * ------------------------------------------------------------------------
 */

LX200SkyWalkerFocuser::LX200SkyWalkerFocuser()
{
    setVersion(SKYWALKERFOCUSER_VERSION_MAJOR, SKYWALKERFOCUSER_VERSION_MINOR);
    setSupportedConnections(CONNECTION_TCP);
    SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_REVERSE);
    DBG_SCOPE = INDI::Logger::DBG_DEBUG;
}

const char *LX200SkyWalkerFocuser::getDefaultName()
{
    return "SkyWalker Focuser";
}

bool LX200SkyWalkerFocuser::saveConfigItems(FILE *fp)
{
    IUSaveConfigSwitch(fp, &FocusReverseSP);
    IUSaveConfigSwitch(fp, &FocusSpeedSP);

    return INDI::Focuser::saveConfigItems(fp);
}

/*
 * Talk to the DDM controller (TCS) and get its firmware version. Consider we are not connected unless we successfully get this information.
 * Taken from AOK SkyWalker driver
 */
bool LX200SkyWalkerFocuser::Handshake()
{
    // TODO: make fwinfo length a constant, check with manufacturer.
    char fwinfo[64] = {0};
    if (!getFirmwareInfo(fwinfo)) {

        LOG_ERROR("Communication with TCS failed");
        return false;

    } else {

        char strinfo[1][64] = {""};
        sscanf(fwinfo, "%*[\"]%64[^\"]", strinfo[0]);
        strcpy(FirmwareVersionT[0].text, strinfo[0]);
        IDSetText(&FirmwareVersionTP, nullptr);
        LOGF_INFO("Handshake ok. Firmware version: %s", strinfo[0]);
        return true;
    }
    return true;
}


/*
 * --------------------------------------------------------------------
 *
 * Process client requests
 *
 * --------------------------------------------------------------------
*/
bool LX200SkyWalkerFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
     if (( dev != nullptr ) && ( strcmp( dev, getDeviceName() ) == 0 )) {

         if (!strcmp(name, FocusMotionSP.name)) {

            if (IUUpdateSwitch(&FocusMotionSP, states, names, n) < 0)
                return false;

            focuserDirection = IUFindOnSwitchIndex(&FocusMotionSP) > 0 ? FOCUS_OUTWARD : FOCUS_INWARD;
            FocusMotionSP.s = IPS_OK;
            IDSetSwitch(&FocusMotionSP, nullptr);
            return true;

        } else if (!strcmp(name, FocusAbortSP.name)) {

            IUResetSwitch(&FocusAbortSP);
            FocusAbortSP.s = AbortFocuser() ? IPS_OK : IPS_ALERT;
            FocusAbsPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, nullptr);
            IDSetSwitch(&FocusAbortSP, nullptr);
            return true;

        } else if (!strcmp(name, FocusReverseSP.name)) {

            if (IUUpdateSwitch(&FocusReverseSP, states, names, n) < 0)
                return false;

            focuserReversed = IUFindOnSwitchIndex(&FocusReverseSP) > 0 ? INDI_ENABLED : INDI_DISABLED;
            FocusReverseSP.s = IPS_OK;
            IDSetSwitch(&FocusReverseSP, nullptr);
            return true;

         } else if (!strcmp(name, FocusSpeedSP.name)) {

            if (IUUpdateSwitch(&FocusSpeedSP, states, names, n) < 0)
                return false;

            bool focuserSlowSpeed = ( IUFindOnSwitchIndex(&FocusSpeedSP) <= 0 );
            FocusSpeedSP.s = sendFocuserSetSpeed( focuserSlowSpeed ) ? IPS_OK : IPS_ALERT;
            IDSetSwitch(&FocusSpeedSP, nullptr);
            return true;
       }
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool LX200SkyWalkerFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0) {

        if (!strcmp(name, FocusAbsPosNP.name)) {

            uint32_t absolutePosition = static_cast<uint32_t>(values[0]);
            if(validateFocusAbsPos(absolutePosition)) {

                double currentPosition = FocusAbsPosN[0].value;
                IUUpdateNumber(&FocusAbsPosNP, values, names, n );
                FocusAbsPosN[0].value = currentPosition;
                FocusAbsPosNP.s = MoveAbsFocuser(absolutePosition);
                IDSetNumber(&FocusAbsPosNP, nullptr );
                return true;
            }
            FocusAbsPosNP.s = IPS_ALERT;
            IDSetNumber(&FocusAbsPosNP, nullptr );
            return false;

        }
    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool LX200SkyWalkerFocuser::initProperties()
{
//    setDeviceName("SkyWalker Focuser");

    /* Make sure to init parent properties first */
    INDI::Focuser::initProperties();

    IUFillSwitch(&FocusSpeedS[0], "FOCUS_SLOW", "Focus Slow", ISS_ON);
    IUFillSwitch(&FocusSpeedS[1], "FOCUS_FAST", "Focus Fast", ISS_OFF);
    IUFillSwitchVector(&FocusSpeedSP, FocusSpeedS, 2, getDeviceName(), "FOCUS_SPEED", "Speed", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);

    IUFillNumber(&FocusAbsPosN[0], "FOCUS_ABSOLUTE_POSITION", "Ticks", "%7.0f", 0.0, TCS_FOCUSER_MAX_POSITION, (TCS_FOCUSER_MAX_POSITION/10.0), 0);
    IUFillNumberVector(&FocusAbsPosNP, FocusAbsPosN, 1, getDeviceName(), "ABS_FOCUS_POSITION", "Absolute Position", MAIN_CONTROL_TAB, IP_RW, 60, IPS_OK);

    IUFillNumber(&FocusRelPosN[0], "FOCUS_RELATIVE_POSITION", "Ticks", "%7.0f", 0.0, TCS_FOCUSER_MAX_POSITION/2, (TCS_FOCUSER_MAX_POSITION/10.0), 100);
    IUFillNumberVector(&FocusRelPosNP, FocusRelPosN, 1, getDeviceName(), "REL_FOCUS_POSITION", "Relative Position", MAIN_CONTROL_TAB, IP_RW, 60, IPS_OK);

    IUFillNumber(&FocusMaxPosN[0], "FOCUS_MAX_VALUE", "Steps", "%7.0f", 0.0, TCS_FOCUSER_MAX_POSITION, (TCS_FOCUSER_MAX_POSITION/10.0), 0);
    IUFillNumberVector(&FocusMaxPosNP, FocusMaxPosN, 1, getDeviceName(), "FOCUS_MAX", "Max. Position", MAIN_CONTROL_TAB, IP_RW, 60, IPS_OK);

    IUFillText(&FirmwareVersionT[0], "Firmware", "Version", "123456");
    IUFillTextVector(&FirmwareVersionTP, FirmwareVersionT, 1, getDeviceName(), "Firmware", "Firmware", INFO_TAB, IP_RO, 60, IPS_IDLE);

    return true;
}

void LX200SkyWalkerFocuser::ISGetProperties(const char *dev)
{
    INDI::Focuser::ISGetProperties(dev);
}

bool LX200SkyWalkerFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected()) {

        defineProperty(&FocusMotionSP);
        defineProperty(&FocusSpeedSP);
        defineProperty(&FocusAbortSP);
        defineProperty(&FocusAbsPosNP);
        defineProperty(&FocusRelPosNP);
        defineProperty(&FocusMaxPosNP);
        defineProperty(&FirmwareVersionTP);

    } else {

        deleteProperty(FocusMotionSP.name);
        deleteProperty(FocusSpeedSP.name);
        deleteProperty(FocusAbortSP.name);
        deleteProperty(FocusAbsPosNP.name);
        deleteProperty(FocusRelPosNP.name);
        deleteProperty(FocusMaxPosNP.name);
        deleteProperty(FirmwareVersionTP.name);
    }

    return true;
}
bool LX200SkyWalkerFocuser::Connect()
{
    if (! DefaultDevice::Connect())
        return false;

    AbortFocuser();
    return true;
}

bool LX200SkyWalkerFocuser::Disconnect()
{
    AbortFocuser();
    return DefaultDevice::Disconnect();
}

void LX200SkyWalkerFocuser::TimerHit()
{
    if (!isConnected()) {

        SetTimer(getCurrentPollingPeriod());
        return;
    }

    ReadFocuserStatus();
}

/*
 * ------------------------------------------------------------------------------
 *
 * Glue between the INDI framework and the actual focuser commands
 *
 * ------------------------------------------------------------------------------
 */

IPState LX200SkyWalkerFocuser::MoveAbsFocuser(uint32_t absolutePosition)
{
    bool result = sendMoveFocuserToPosition(absolutePosition);
    if (!result) {

        LOGF_INFO("failed%s","");
        return IPS_ALERT;
    }

    return IPS_BUSY;
}

IPState LX200SkyWalkerFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int32_t newPosition = 0;
    if ( dir != focuserDirection )
        sendFocuserDirection( dir );
    if ( (( dir == FOCUS_INWARD ) && ( focuserReversed == false )) || (( dir == FOCUS_OUTWARD ) && ( focuserReversed == true )) )
        newPosition = FocusAbsPosN[0].value - ticks;
    else
        newPosition = FocusAbsPosN[0].value + ticks;

    newPosition = std::max(static_cast<int>(FocusAbsPosN[0].min), static_cast<int32_t>(std::min(static_cast<int32_t>(FocusAbsPosN[0].max), newPosition)));
    if (!MoveAbsFocuser(newPosition))
        return IPS_ALERT;

    FocusRelPosN[0].value = ticks;
    FocusRelPosNP.s       = IPS_BUSY;

    return IPS_BUSY;
}

/* Taken from LX200 Avalon StarGo Focuser */
bool LX200SkyWalkerFocuser::validateFocusAbsPos(uint32_t absolutePosition)
{
    uint32_t minPosition = static_cast<uint32_t>(FocusAbsPosN[0].min);
    uint32_t maxPosition = static_cast<uint32_t>(FocusAbsPosN[0].max);

    if (absolutePosition < minPosition || absolutePosition > maxPosition) {

        DEBUGF(INDI::Logger::DBG_ERROR, "%s: Cannot set focuser absolute position to %d, it is outside the valid range of [%d, %d]", getDeviceName(), absolutePosition, minPosition, maxPosition);
        return false;
    }
    return true;
}

/* Taken from LX200 Avalon StarGo Focuser */
bool LX200SkyWalkerFocuser::isFocuserMoving()
{
    bool moving = sendQueryFocuserMoving();
    return moving || FocusAbsPosNP.s == IPS_BUSY || FocusRelPosNP.s == IPS_BUSY;
}

/* Taken from LX200 Avalon StarGo Focuser */
bool LX200SkyWalkerFocuser::atFocuserTargetPosition()
{
    return static_cast<uint32_t>(FocusAbsPosN[0].value) == targetFocuserPosition;
}

bool LX200SkyWalkerFocuser::ReadFocuserStatus()
{
    if (!isConnected())
        return true;

    int absolutePosition = 0;
    if (sendQueryFocuserPosition(&absolutePosition)) {

        FocusAbsPosN[0].value = absolutePosition;
        IDSetNumber(&FocusAbsPosNP, nullptr);
    }
    else
        return false;

    if (isFocuserMoving() && atFocuserTargetPosition()) {

        FocusAbsPosNP.s = IPS_OK;
        IDSetNumber(&FocusAbsPosNP, nullptr);
        FocusRelPosNP.s = IPS_OK;
        IDSetNumber(&FocusRelPosNP, nullptr);
    }

    SetTimer(getCurrentPollingPeriod());

    return true;
}

/**
 * @brief Retrieve the firmware info from the mount
 * @param firmwareInfo - firmware description
 * @return
 */
/* Taken from AOK SkyWalker */
bool LX200SkyWalkerFocuser::getFirmwareInfo(char* vstring)
{
    char lstat[40] = {0};
    if(!getJSONData_gp(1, lstat, 40))
        return false;
    else
    {
        strcpy(vstring, lstat);
        return true;
    }
}

/*
 * -------------------------------------
 *
 * Helper functions
 *
 * -------------------------------------
 */

/**
 * @brief Receive answer from the communication port.
 * @param buffer - buffer holding the answer
 * @param bytes - number of bytes contained in the answer
 * @author CanisUrsa
 * @return true if communication succeeded, false otherwise
 */
/* Taken from AOK SkyWalker */
bool LX200SkyWalkerFocuser::receive(char* buffer, char end, int wait)
{
    LOGF_DEBUG("%s timeout=%ds",__FUNCTION__, wait);
    int bytes = 0;
    int timeout = wait;
    int returnCode = tty_read_section(PortFD, buffer, end, timeout, &bytes);
    if (returnCode != TTY_OK && (bytes < 1))
    {
        char errorString[MAXRBUF];
        tty_error_msg(returnCode, errorString, MAXRBUF);
        if(returnCode == TTY_TIME_OUT && wait <= 0) return false;
        LOGF_WARN("Failed to receive full response: %s. (Return code: %d)", errorString, returnCode);
        return false;
    }
    if(buffer[bytes - 1] == '#')
        buffer[bytes - 1] = '\0'; // remove #
    else
        buffer[bytes] = '\0';

    return true;
}

/* Taken from AOK SkyWalker */
bool LX200SkyWalkerFocuser::transmit(const char* buffer)
{
    LOG_DEBUG(__FUNCTION__);
    int bytesWritten = 0;
    flush();
    int returnCode = tty_write_string(PortFD, buffer, &bytesWritten);
    if (returnCode != TTY_OK)
    {
        char errorString[MAXRBUF];
        tty_error_msg(returnCode, errorString, MAXRBUF);
        LOGF_WARN("Failed to transmit %s. Wrote %d bytes and got error %s.", buffer, bytesWritten, errorString);
        return false;
    }
    return true;
}

/* Taken from AOK SkyWalker */
void LX200SkyWalkerFocuser::flush()
{
    //LOG_DEBUG(__FUNCTION__);
    //tcflush(PortFD, TCIOFLUSH);
}

/**
 * @brief Send a LX200 query to the communication port and read the result.
 * @param cmd LX200 query
 * @param response answer
 * @return true if the command succeeded, false otherwise
 */
/* Taken from AOK SkyWalker */
bool LX200SkyWalkerFocuser::sendQuery(const char* cmd, char* response, char end, int wait)
{
    LOGF_DEBUG("%s %s End:%c Wait:%ds", __FUNCTION__, cmd, end, wait);
    response[0] = '\0';
    char lresponse[TCS_RESPONSE_BUFFER_LENGTH];
    lresponse [0] = '\0';
    bool lresult = false;
    if(!transmit(cmd))
    {
        LOGF_ERROR("Command <%s> not transmitted.", cmd);
    }
    else if (wait > TCS_NOANSWER)
    {
        if (receive(lresponse, end, wait))
        {
            strcpy(response, lresponse);
            return true;
        }
    }
    else
        lresult = true;
    return lresult;
}

/* Taken from AOK SkyWalker */
bool LX200SkyWalkerFocuser::getJSONData_gp(int jindex, char *jstr, int jstrlen) // preliminary hardcoded  :gp-query
{
    char lresponse[128];
    lresponse [0] = '\0';
    char lcmd[4] = ":gp";
    char end = '}';
    if(!transmit(lcmd))
    {
        LOGF_ERROR("Command <%s> not transmitted.", lcmd);
    }
    if (receive(lresponse, end, 1))
    {
        flush();
    }
    else
    {
        LOG_ERROR("Failed to get JSONData");
        return false;
    }
    char data[3][40] = {"", "", ""};
    int returnCode = sscanf(lresponse, "%*[^[][%40[^\"]%40[^,]%*[,]%40[^]]", data[0], data[1], data[2]);
    if (returnCode < 1)
    {
        LOGF_ERROR("Failed to parse JSONData '%s'.", lresponse);
        return false;
    }
    strncpy(jstr, data[jindex], jstrlen);
    return true;
}

/*
 * ----------------------------------------------------------------
 *
 * Send commands to the focuser
 * TODO: ALL add simulation scenario
 * ----------------------------------------------------------------
 */

bool LX200SkyWalkerFocuser::sendQueryFocuserMoving()
{
    // Command  - :FB1#
    // Response - 1# Yes / 0# No

    char command[TCS_COMMAND_BUFFER_LENGTH] = {0};
    char response[TCS_RESPONSE_BUFFER_LENGTH] = {0};
    sprintf( command, ":FB1#" );
    if (!sendQuery( command, response )) {

        LOG_ERROR("Failed to send query focuser moving status.");
        return false;
    }

    int yesno;
    int returnCode = sscanf(response, "%01d", &yesno);
    if (returnCode <= 0) {

        LOGF_ERROR( "%s: Failed to parse moving query response '%s'.", getDeviceName(), response);
        return false;
    }

    return (yesno == 1);
}

bool LX200SkyWalkerFocuser::sendFocuserSetSpeed( bool slow )
{
    // Command  - :FF# (Fast) or :F+# (Slow)
    // Response - Nothing
    // "Fast" is 8x more than "Slow" (~408 ticks vs. 51 as reported by the TCS)

    char command[TCS_COMMAND_BUFFER_LENGTH] = {0};
    char response[TCS_RESPONSE_BUFFER_LENGTH] = {0};

    sprintf( command, ":F%c#", (slow == true)?'S':'F');
    if (!sendQuery( command, response, TCS_NOANSWER)) {

        char message[256];
        sprintf( message, "Failed to send query focuser %s speed request.",(slow == true)?"slow":"fast");
        LOG_ERROR(message);
        return false;
    }

    return true;
}

bool LX200SkyWalkerFocuser::sendFocuserDirection( FocusDirection dir )
{
    // Command  - :F-# (inward) or :F+# (outward)
    // Response - Nothing

    char command[TCS_COMMAND_BUFFER_LENGTH] = {0};
    char response[TCS_RESPONSE_BUFFER_LENGTH] = {0};

    sprintf( command, ":F%c#", (dir == FOCUS_INWARD)?'-':'+');
    if (!sendQuery( command, response, TCS_NOANSWER)) {

        char message[256];
        sprintf( message, "Failed to send query focuser %s request.",(dir == FOCUS_INWARD)?"INWARD":"OUTWARD");
        LOG_ERROR(message);
        return false;
    }

    return true;
}

bool LX200SkyWalkerFocuser::sendQueryFocuserPosition(int* position)
{
    // Command  - :Fp#
    // Response - xxxxxxx

    char response[TCS_RESPONSE_BUFFER_LENGTH] = {0};
//    LOGF_INFO("Focuser position? command%s","");
    if (!sendQuery(":Fp#", response)) {

        LOG_ERROR("Failed to send query focuser position request.");
        return false;
    }

    int tempPosition = 0;
    int returnCode = sscanf(response, "%07d", &tempPosition);
    if (returnCode <= 0) {

        DEBUGF(INDI::Logger::DBG_ERROR, "%s: Failed to parse position response '%s'.", getDeviceName(), response);
        return false;
    }

    *position = tempPosition;
 //   LOGF_INFO("Focuser at position %d",tempPosition);
    return true;
}

bool LX200SkyWalkerFocuser::sendMoveFocuserToPosition(uint32_t position)
{
    // Command  - :FM1xxxxxxx#
    // Response - Nothing

    // Be careful, position 100 has to be formatted as FM1100# : no leading zeros

    targetFocuserPosition = position;
    char command[TCS_COMMAND_BUFFER_LENGTH] = {0};
    char response[TCS_RESPONSE_BUFFER_LENGTH] = {0};

    sprintf(command, ":FM1%7d#", targetFocuserPosition);
    if (!sendQuery(command,response,TCS_NOANSWER)) {

        LOGF_ERROR("%s: Failed to send goto command.", getDeviceName());
        return false;
    }
    return true;
}

bool LX200SkyWalkerFocuser::AbortFocuser()
{
    char response[TCS_RESPONSE_BUFFER_LENGTH];
    if (!isSimulation() && !sendQuery(":FQ#", response, TCS_NOANSWER)) {

        LOG_ERROR("Failed to abort focuser motion.");
        return false;
    }
    return true;
}

/*
    AOK Focuser driver
    (based on AOK Skywalker driver)

    Copyright (C) F. Lesage

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

#ifndef AOKFOCUSER_H
#define AOKFOCUSER_H

#pragma once

#include <mounts/lx200driver.h>
#include "libindi/indifocuser.h"

#include <indicom.h>
#include <indilogger.h>
#include <termios.h>

#include <cstring>
#include <string>
#include <unistd.h>

#define LX200_TIMEOUT                                 5 /* FD timeout in seconds */
#define RB_MAX_LEN                                   64
#define TCS_TIMEOUT                                   1 /* 50ms? */
#define TCS_COMMAND_BUFFER_LENGTH                    32
#define TCS_RESPONSE_BUFFER_LENGTH                   32
#define TCS_NOANSWER                                  0

#define TCS_FOCUSER_MAX_POSITION                4000000.0

enum TFormat
{
    LX200_SHORT_FORMAT,
    LX200_LONG_FORMAT,
    LX200_LONGER_FORMAT
};

// Skywalker specific tabs
//extern const char *INFO_TAB; // Infotab for versionumber, etc.

class LX200SkyWalkerFocuser : public INDI::Focuser
{
    public:

        LX200SkyWalkerFocuser();
        virtual ~LX200SkyWalkerFocuser() = default;

        bool initProperties();
        bool updateProperties() override;

        virtual void ISGetProperties(const char *dev);
        bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;

        const char *getDefaultName() override;
        bool Handshake() override;
        bool ReadFocuserStatus();

//	bool isConnected();
        bool activate( bool enabled );
        virtual void TimerHit() override;

    protected:

        virtual bool Connect() override;
        virtual bool Disconnect() override;

        uint8_t DBG_SCOPE {0};
        uint32_t targetFocuserPosition;

        // TCS version
        ITextVectorProperty FirmwareVersionTP;
        IText FirmwareVersionT[1] {};

        // Fast or slow
        ISwitchVectorProperty FocusSpeedSP;
        ISwitch FocusSpeedS[2];

        bool focuserActivated;
        FocusDirection focuserDirection = FOCUS_INWARD;
        bool focuserReversed = false;

        bool saveConfigItems( FILE *fp ) override;
        IPState MoveAbsFocuser(uint32_t absolutePosition);
        IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks);

        bool isFocuserMoving();
        bool atFocuserTargetPosition();
        bool sendQueryFocuserPosition(int* position);
        bool validateFocusAbsPos(uint32_t absolutePosition);
        bool sendMoveFocuserToPosition(uint32_t position);
        bool sendFocuserDirection( FocusDirection dir );
        bool AbortFocuser();
        bool MoveFocuserToPosition(int position);
        bool sendFocuserSetSpeed( bool slow );
        bool sendQueryFocuserMoving();

        // helper functions
        virtual bool receive(char* buffer, int wait = TCS_TIMEOUT);
        virtual bool receive(char* buffer, char end, int wait = TCS_TIMEOUT);
        virtual bool transmit(const char* buffer);
        virtual void flush();
        bool getJSONData_gp(int jindex, char *jstr, int jstrlen);

        virtual bool sendQuery(const char* cmd, char* response, char end, int wait = TCS_TIMEOUT);
        // Wait for default "#' character
        virtual bool sendQuery(const char* cmd, char* response, int wait = TCS_TIMEOUT);
        virtual bool getFirmwareInfo(char* vstring);

};

inline bool LX200SkyWalkerFocuser::sendQuery(const char* cmd, char* response, int wait)
{
    return sendQuery(cmd, response, '#', wait);
}

inline bool LX200SkyWalkerFocuser::receive(char* buffer, int wait)
{
    return receive(buffer, '#', wait);
}

#endif

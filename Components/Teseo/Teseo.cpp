/**
*******************************************************************************
* @file    Teseo.h
* @author  AST / Central Lab
* @version V1.0.0
* @date    May-2017
* @brief   Teseo Location Class
*
*******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
********************************************************************************
*/

#include "Teseo.h"

static char TESEO_NAME[] = "Teseo-LIV3F";
/*
static char _OK[] = "OK";
static char _Failed[] = "Failed";
*/

struct teseoCmd {
    char *cmd;
};

static struct teseoCmd teseoCmds[] = {
  [Teseo::TEST] = {
    .cmd = "$\n\r",
  },
  [Teseo::GETSWVER] = {
    .cmd = "$PSTMGETSWVER,6\n\r",
  },
  [Teseo::FORCESTANDBY] = {
    .cmd = "$PSTMFORCESTANDBY,00010\n\r",
  },
  [Teseo::RFTESTON] = {
    .cmd  = "$PSTMRFTESTON,16\n\r",
  },
  [Teseo::RFTESTOFF] = {
    .cmd  = "$PSTMRFTESTOFF\n\r",
  },
  [Teseo::LOWPOWER] = {
    .cmd  = "$PSTMLOWPOWERONOFF,1,0,000,05,0,1,000,1,00010,01,0,0,1,01\n\r",
  },
  [Teseo::FWUPDATE] = {
    .cmd  = "$PSTMFWUPGRADE\n\r",
  }
};

Teseo::Teseo(PinName resetPin,
             PinName wakeupPin,
             PinName ppsPin,
             PinName uartTxPin,
             PinName uartRxPin,
             Serial *serialDebug):
  _loc_led2(LOC_LED2),
  _reset(resetPin, 1),
  _pps(ppsPin),
  _wakeup(wakeupPin, 0),
  _uartRx(uartRxPin),
  _uartTx(uartTxPin),
  _serialDebug(serialDebug)
{
  wait_ms(POWERON_STABLE_SIGNAL_DELAY_MS);
  _uart = NULL;
  _i2c = NULL;
  
  _locState = TESEO_LOC_STATE_IDLE;
  
  deviceInfo = TESEO_NAME;
}

Teseo::Teseo(PinName resetPin,
             PinName wakeupPin,
             PinName ppsPin,
             PinName uartTxPin,
             PinName uartRxPin,
             I2C    *i2cBus,
             Serial *serialDebug):
    _loc_led2(LOC_LED2),
    _reset(resetPin, 1),
    _pps(ppsPin),
    _wakeup(wakeupPin, 0),
    _uartRx(uartRxPin),
    _uartTx(uartTxPin),
    _serialDebug(serialDebug),
    _i2c(i2cBus)
{
  wait_ms(POWERON_STABLE_SIGNAL_DELAY_MS);
  _uart = NULL;
  
  _locState = TESEO_LOC_STATE_IDLE;

  deviceInfo = TESEO_NAME;
}

void
Teseo::TeseoLocRegOutput(teseo_app_output_callback app_output_cb, teseo_app_event_callback app_event_cb)
{
  appOutCb   = app_output_cb;
  appEventCb = app_event_cb;
}

int
Teseo::EnableLowPower()
{
  SendCommand(LOWPOWER);
  return 0;
}

void
Teseo::_ResetFast(Serial *serialDebug)
{
  if (serialDebug)
    serialDebug->printf("%s: Resetting...", TESEO_NAME);
  
  _reset.write(0);
  
  wait_ms(5);
  
  _reset.write(1);
  
  wait_ms(70);
  
  if (serialDebug)
    serialDebug->printf("Done...\n\r");

}

void
Teseo::_Reset(Serial *serialDebug)
{
  if (serialDebug)
    serialDebug->printf("%s: Resetting...", TESEO_NAME);

  //_pps.output();
  //_pps.write(0);
  
  //wait_ms(500);
  
  _reset.write(1);
  
  wait_ms(500);
  
  _reset.write(0);
  //_pps.write(0);
  
  wait_ms(500);
  
  _reset.write(1);
  
  if (serialDebug)
    serialDebug->printf("Done...\n\r");
  
  //_pps.write(1);
  
  //wait_ms(1000);
  
  //_pps.input();
}

void
Teseo::_SendString(char *buf, int len)
{
  for (int i = 0; i < len; ++i) {
    while (!_uart->writeable());
    _uart->putc(buf[i]);
  }
}

int
Teseo::_WakeUp()
{
  wait_ms(100);
  
  _wakeup.write(1);
  
  wait_ms(500);
  
  _wakeup.write(0);
  
  return 0;
}

int
Teseo::_CRC(char *buf, int size)
{
  int i = 0, ch = 0;
  
  if (buf[0] == '$') {
    ++i;
  }
  
  if (size) {
    for (; i < size; ++i) {
      ch ^= buf[i];
    }
  } else {
    for (; buf[i] != 0; ++i) {
      ch ^= buf[i];
    }
  }
  
  return ch;
}

void
Teseo::SendCommand(Teseo::eCmd c)
{
  char crc[3];
  
  _uart->baud(9600);
  
  sprintf(crc, "*%02X", _CRC(teseoCmds[c].cmd, -1));
  
  _SendString(teseoCmds[c].cmd, strlen(teseoCmds[c].cmd));
  _SendString(crc, 3);
}

char *
Teseo::_DetectSentence(const char *cmd, uint8_t *buf, unsigned long len)
{
  char *result = NULL;
  unsigned int i = 0;
  const unsigned long cmd_len = strlen(cmd);
  len -= strlen(cmd);
  
  while (!result && i < len) {
    for (; buf[i] != '$' && i < len; ++i); /* 1. check '$' char */
    if (i == len)
      break; /* no more char.... */
    
    ++i; /* to point to the char after '$' */
    
    if (strncmp((char *)(&buf[i]), cmd, cmd_len) == 0) {
      result = (char *)&buf[i];
    }
  }
  
  if (result) {
    for (i = 0; result[i] != '*'; ++i);
    result[i] = 0;
  }
#if 0
  if (_serialDebug)
    _serialDebug->printf("%s: %s: %s %s FOUND\n\r", TESEO_NAME, __FUNCTION__, cmd, result ? " " : "NOT");
#endif
  return result;
}

/** TBC */
int
Teseo::_CheckI2C()
{
  if (!_i2c)
    return -1;
  
  _i2c->start();
  int res = _i2c->write((TESEO_I2C_ADDRESS << 1) | 1);
  _i2c->stop();
  /*
  *  @returns
  *    '0' - NAK was received
  *    '1' - ACK was received,
  *    '2' - timeout
  */
  return res == 1 ? 0 : -1;
}

int
Teseo::_ReadMessage(uint8_t *buf, unsigned long len, Timer *t, float timeout)
{
  memset(buf, 0, len);
  
  _uart->baud(9600);
  
  for (unsigned int i = 0; i < len; ++i){
    if (t) {
      unsigned int now = t->read_ms();
      while (!_uart->readable() && (now + timeout*1000) > t->read_ms());
    } else
      while (!_uart->readable());
    if (_uart->readable())
      buf[i] = _uart->getc();
  }
#if 0
  if (_serialDebug) {
    unsigned int i;
    _serialDebug->printf("\n\r---------------------\n\r");
    for (i = 0; i < len ; ++i)
      _serialDebug->putc((int)buf[i]);
    _serialDebug->printf("\n\r---------------------\n\r");
  }
#endif
  return 0;
}

char *
Teseo::ReadSentence(const char *msg)
{
  int ret = _ReadMessage(aRxBuffer, MAX_LEN);
  if (ret) {
    return NULL;
  }
  
  return _DetectSentence(msg, aRxBuffer, MAX_LEN);
}

void
Teseo::_InitUART(int br)
{  
  _uart = new Serial(_uartRx, _uartTx);
  _uart->format(8, SerialBase::None, 1);
  _uart->baud(br);
}

bool
Teseo::setPowerMode(GPSProvider::PowerMode_t pwrMode)
{
  /* TBI */
  return false;
}

void
Teseo::eventHandler(eTeseoLocEventType event, uint32_t data)
{
  if (appEventCb) {
    appEventCb(event, data);
  }
}

void
Teseo::start(void)
{
  if(_locState == TESEO_LOC_STATE_IDLE) {
    _InitUART();
    _locState = TESEO_LOC_STATE_RUN;
    eventHandler(TESEO_LOC_EVENT_START_RESULT, 0);
  } else {
    TESEO_LOG_INFO("Already started\r\n");
  }
}

void
Teseo::stop(void)
{
  if(_locState == TESEO_LOC_STATE_IDLE) {
    return;
  }
  
  _locState = TESEO_LOC_STATE_IDLE;
  eventHandler(TESEO_LOC_EVENT_STOP_RESULT, 0);
}

void
Teseo::outputHandler(uint32_t msgId, uint32_t msgType, tTeseoData *pData)
{
  switch(msgId) {
  case LOC_OUTPUT_LOCATION:
    
    if(pData->gpgga_data.valid == VALID) {
      lastLocation.valid = true;
      lastLocation.lat = pData->gpgga_data.xyz.lat;
      lastLocation.lon = pData->gpgga_data.xyz.lon;
      lastLocation.altitude = pData->gpgga_data.xyz.alt;
      lastLocation.numGPSSVs = pData->gpgga_data.sats;
      lastLocation.utcTime = pData->gpgga_data.utc.utc;
    } else {
      lastLocation.valid = false;
    }
    
    if (locationCallback) {
      locationCallback(&lastLocation);
    }

    break;
    
  default:
    break;
  }
  
  if (appOutCb) {
    appOutCb(msgId, msgType, pData);
  }
}

void
Teseo::_GetMsg(Teseo::eMsg msg)
{  
  eStatus status;

  memset(aRxBuffer, 0, MAX_LEN);
  _ReadMessage(aRxBuffer, MAX_LEN);

  switch(msg) {
  case GPGGA:
    status = (eStatus)parse_gpgga(&pData.gpgga_data, aRxBuffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_LOCATION, msg, &pData);
    }
    break;

  case GNS:
    status = (eStatus)parse_gnsmsg(&pData.gns_data, aRxBuffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }
    break;
    
  case GPGST:
    status = (eStatus)parse_gpgst(&pData.gpgst_data, aRxBuffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }
    break;
    
  case GPRMC:
    status = (eStatus)parse_gprmc(&pData.gprmc_data, aRxBuffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }
    break;
    
  case GSA:
    status = (eStatus)parse_gsamsg(&pData.gsa_data, aRxBuffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }
    break;
    
  case GSV:
    status = (eStatus)parse_gsvmsg(&pData.gsv_data, aRxBuffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }    
    break;
    
  default:
    break;
    
  }
}

void
Teseo::_TeseoLocProcessNmeaStream(void)
{
  for(int m = 0; m < NMEA_MSGS_NUM; m++) {
    _GetMsg((eMsg)m);
  }
}

void
Teseo::process(void)
{  
  if(_locState == TESEO_LOC_STATE_RUN) {
    _TeseoLocProcessNmeaStream();
  }
}

uint32_t
Teseo::ioctl(uint32_t command, void *arg)
{
  /* TBI */
  return 0;
}

void
Teseo::lpmGetImmediateLocation(void)
{
  /* TBI */
}

const GPSProvider::LocationUpdateParams_t *
Teseo::getLastLocation(void) const
{
  return &lastLocation;
}

void
Teseo::reset(void)
{
  _ResetFast();
}

gps_provider_error_t
Teseo::configGeofences(GPSGeofence *geofences[])
{
  /* TBI */
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::geofenceReq(void)
{
  /* TBI */
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::configLog(GPSDatalog *datalog)
{
  /* TBI */
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::startLog(void)
{
  /* TBI */
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::stopLog(void)
{
  /* TBI */
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::eraseLog(void)
{
  /* TBI */
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::logReqStatus(void)
{
  /* TBI */
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::logReqQuery(GPSProvider::LogQueryParams_t &logReqQuery)
{
  /* TBI */
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::startOdo(void)
{
  /* TBI */
  return GPS_ERROR_NONE;
}
gps_provider_error_t
Teseo::stopOdo(void)
{
  /* TBI */
  return GPS_ERROR_NONE;
}
gps_provider_error_t
Teseo::resetOdo(void)
{
  /* TBI */
  return GPS_ERROR_NONE;
}
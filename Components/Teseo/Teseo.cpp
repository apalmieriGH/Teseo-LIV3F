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
    .cmd = (char*)"$\n\r",
  },
  [Teseo::GETSWVER] = {
    .cmd = (char*)"$PSTMGETSWVER,6\n\r",
  },
  [Teseo::FORCESTANDBY] = {
    .cmd = (char*)"$PSTMFORCESTANDBY,00010\n\r",
  },
  [Teseo::RFTESTON] = {
    .cmd  = (char*)"$PSTMRFTESTON,16\n\r",
  },
  [Teseo::RFTESTOFF] = {
    .cmd  = (char*)"$PSTMRFTESTOFF\n\r",
  },
  [Teseo::LOWPOWER] = {
    .cmd  = (char*)"$PSTMLOWPOWERONOFF,1,0,000,05,0,1,000,1,00010,01,0,0,1,01\n\r",
  },
  [Teseo::FWUPDATE] = {
    .cmd  = (char*)"$PSTMFWUPGRADE\n\r",
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
  
  sprintf(crc, "*%02X", _CRC(teseoCmds[c].cmd, -1));

  _SendString(teseoCmds[c].cmd, strlen(teseoCmds[c].cmd));
  _SendString(crc, 3);
}

void
Teseo::SendCommand(char *cmd)
{
  char crc[5];
  
  sprintf(crc, "*%02X\n\r", _CRC(cmd, strlen(cmd)));
  //printf("CRC=%s\n\r", crc);
  
  _SendString(cmd, strlen(cmd));
  _SendString(crc, 5);
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

void
Teseo::ReadSentence(Teseo::eMsg msg)
{
  eStatus status = TESEO_STATUS_FAILURE;

  do {
    osEvent evt = queue.get();
    if (evt.status == osEventMessage) {
      struct _teseoMsg *message = (struct _teseoMsg *)evt.value.p;
      if (message->len > 0) {
        status = _GetMsg(msg, message->buf);
      }
      
      mpool.free(message);
    }
  } while (status != TESEO_STATUS_SUCCESS);
}

void
Teseo::_InitUART(int br)
{  
  _uart = new (std::nothrow) Serial(_uartRx, _uartTx);
  if(_uart != NULL) {
    _uart->format(8, SerialBase::None, 1);
    _uart->baud(br);
  } else {
    TESEO_LOG_INFO("Error allocating UART.\r\n");
  }
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
Teseo::ReadProcess(void)
{
  char c;
  
  struct _teseoMsg *msg = mpool.alloc();
  msg->len = 0;
  
  while(true) {
    if (_uart->readable()) {
      c = _uart->getc();
      
      if (c == '$') {
        queue.put(msg);
        msg = mpool.alloc();
        msg->len = 0;
      }
      msg->buf[msg->len++] = c;

    } else {
      Thread::yield(); //wait_us(100); Allow other threads to run
    }
  }
}

static void
_UARTStreamProcess(Teseo *gnss)
{
  gnss->ReadProcess();
}

void
Teseo::start(void)
{
  if(_locState == TESEO_LOC_STATE_IDLE) {

    _InitUART();
    _locState = TESEO_LOC_STATE_RUN;
    
    // Start thread for UART listener and set the highest priority
    serialStreamThread = new (std::nothrow) Thread();
    if(serialStreamThread != NULL) {
      serialStreamThread->set_priority(osPriorityRealtime);
      serialStreamThread->start(callback(_UARTStreamProcess, this));
    } else {
      TESEO_LOG_INFO("Error allocating serialStreamThread\r\n");
    }

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
  // Stop thread for UART listener
  if(serialStreamThread != NULL) {
    serialStreamThread->terminate();
    delete serialStreamThread;
  }
  
  if(_uart != NULL) {
    delete _uart;
    _uart = NULL;
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
   
  case LOC_OUTPUT_PSTM: {
    Teseo::ePSTMsg msg = (Teseo::ePSTMsg)msgType;
    int code;

    switch(msg) {
    case PSTMGEOFENCE: {

      /* Geofence enabling */
      if(pData->geofence_data.op == GNSS_FEATURE_EN_MSG) {
        code = pData->geofence_data.result ?
          GPS_ERROR_FEATURE_ENABLING : GPS_ERROR_NONE;
          
        if (code == GPS_ERROR_NONE) {
          saveConfigParams();
        }
      }

      /* Geofence configuration */
      if(pData->geofence_data.op == GNSS_GEOFENCE_CFG_MSG) {
        code = pData->geofence_data.result ?
          GPS_ERROR_GEOFENCE_CFG : GPS_ERROR_NONE;
        if (geofenceCfgMessageCallback) {
          geofenceCfgMessageCallback(code);
        }
      }

      /* Geofence Status */
      if(pData->geofence_data.op == GNSS_GEOFENCE_STATUS_MSG) {
        code = pData->geofence_data.result ? GPS_ERROR_GEOFENCE_STATUS : GPS_ERROR_NONE;
        if(code == GPS_ERROR_NONE) {
          geofenceStatus.timestamp.hh = pData->geofence_data.timestamp.hh;
          geofenceStatus.timestamp.mm = pData->geofence_data.timestamp.mm;
          geofenceStatus.timestamp.ss = pData->geofence_data.timestamp.ss;
          geofenceStatus.timestamp.day = pData->geofence_data.timestamp.day;
          geofenceStatus.timestamp.month = pData->geofence_data.timestamp.month;
          geofenceStatus.timestamp.year = pData->geofence_data.timestamp.year;
          geofenceStatus.currentStatus = pData->geofence_data.status;
          geofenceStatus.numGeofences = MAX_GEOFENCES_NUM;
        }
        if (geofenceStatusMessageCallback) {
          geofenceStatusMessageCallback(&geofenceStatus, code);
        }
      }

      /* Geofence Alarm */
      if(pData->geofence_data.op == GNSS_GEOFENCE_ALARM_MSG) {
        code = pData->geofence_data.result ? GPS_ERROR_GEOFENCE_STATUS : GPS_ERROR_NONE;
        if(code == GPS_ERROR_NONE) {
          geofenceStatus.timestamp.hh = pData->geofence_data.timestamp.hh;
          geofenceStatus.timestamp.mm = pData->geofence_data.timestamp.mm;
          geofenceStatus.timestamp.ss = pData->geofence_data.timestamp.ss;
          geofenceStatus.currentStatus = pData->geofence_data.status;
          geofenceStatus.idAlarm = pData->geofence_data.idAlarm;
        }
        if (geofenceStatusMessageCallback) {
          geofenceStatusMessageCallback(&geofenceStatus, code);
        }
      }
    }
    break;

    case PSTMODO: {
      /* Odometer enabling */
      if(pData->odo_data.op == GNSS_FEATURE_EN_MSG) {
        
        code = pData->odo_data.result ?
          GPS_ERROR_FEATURE_ENABLING : GPS_ERROR_NONE;

        if (code == GPS_ERROR_NONE) {
          saveConfigParams();
        }
      }

      /* Odometer start */
      if(pData->odo_data.op == GNSS_ODO_START_MSG) {
        code = pData->odo_data.result ?
          GPS_ERROR_ODO_START : GPS_ERROR_NONE;
      }
      /* Odometer stop */
      if(pData->odo_data.op == GNSS_ODO_STOP_MSG) {
        code = pData->odo_data.result ?
          GPS_ERROR_ODO_STOP : GPS_ERROR_NONE;
      }
    }
    break;
    
    case PSTMDATALOG: {
      /* Datalog enabling */
      if(pData->datalog_data.op == GNSS_FEATURE_EN_MSG) {
        
        code = pData->datalog_data.result ?
          GPS_ERROR_FEATURE_ENABLING : GPS_ERROR_NONE;

        if (code == GPS_ERROR_NONE) {
          saveConfigParams();
        }
      }
      /* Datalog create */
      if(pData->datalog_data.op == GNSS_DATALOG_CFG_MSG) {
        code = pData->datalog_data.result ?
          GPS_ERROR_DATALOG_CFG : GPS_ERROR_NONE;
      }
      /* Datalog start */
      if(pData->datalog_data.op == GNSS_DATALOG_START_MSG) {
        code = pData->datalog_data.result ?
          GPS_ERROR_DATALOG_START : GPS_ERROR_NONE;
      }
      /* Datalog stop */
      if(pData->datalog_data.op == GNSS_DATALOG_STOP_MSG) {
        code = pData->datalog_data.result ?
          GPS_ERROR_DATALOG_STOP : GPS_ERROR_NONE;
      }
      /* Datalog erase */
      if(pData->datalog_data.op == GNSS_DATALOG_ERASE_MSG) {
        code = pData->datalog_data.result ?
          GPS_ERROR_DATALOG_ERASE : GPS_ERROR_NONE;
      }
    }
    break;

    case PSTMSGL: {
      /* Msg List cfg */
      code = pData->ack ?
          GPS_ERROR_MSGLIST_CFG : GPS_ERROR_NONE;

      if (code == GPS_ERROR_NONE) {
        saveConfigParams();
      }
    }
    break;
    
    case PSTMSAVEPAR: {
      code = pData->ack ?
          GPS_ERROR_SAVEPAR : GPS_ERROR_NONE;

      if (code == GPS_ERROR_NONE) {
        reset();
      }
    }
    break;
    
  } /* end switch */
  } /* end case LOC_OUTPUT_PSTM */

  break;

  default:
    break;
  }
  
  if (appOutCb) {
    appOutCb(msgId, msgType, pData);
  }
  
}


eStatus
Teseo::_GetMsg(Teseo::eMsg msg, uint8_t *buffer)
{  
  eStatus status = TESEO_STATUS_FAILURE;

  switch(msg) {

  case GPGGA:
    status = (eStatus)parse_gpgga(&pData.gpgga_data, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_LOCATION, msg, &pData);
    }
    break;
    
  case GNS:
    status = (eStatus)parse_gnsmsg(&pData.gns_data, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }
    break;
    
  case GPGST:
    status = (eStatus)parse_gpgst(&pData.gpgst_data, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }
    break;
    
  case GPRMC:
    status = (eStatus)parse_gprmc(&pData.gprmc_data, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }
    break;
    
  case GSA:
    status = (eStatus)parse_gsamsg(&pData.gsa_data, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }
    break;
    
  case GSV:
    status = (eStatus)parse_gsvmsg(&pData.gsv_data, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_NMEA, msg, &pData);
    }    
    break;
    
  default:
    break;
    
  }
  
  return status;
}

void
Teseo::setVerboseMode(int level)
{
  cfgMessageList(level);
}

void
Teseo::_GetLocationMsg(Teseo::eMsg msg, uint8_t *buffer)
{  
  eStatus status;

#if 0
  _serialDebug->printf("\n\r --------------------->\n\r");
  for (int i = 0; i < TESEO_RXBUF_LEN ; ++i) {
    _serialDebug->putc((int)buffer[i]);
  }
  _serialDebug->printf("\n\r<---------------------\n\r");
#endif

  status = (eStatus)parse_gpgga(&pData.gpgga_data, buffer);
  if(status == TESEO_STATUS_SUCCESS) {
    outputHandler(LOC_OUTPUT_LOCATION, msg, &pData);
  }
}
  
void
Teseo::_GetPSTMsg(Teseo::ePSTMsg msg, uint8_t *buffer)
{
  eStatus status;

#if 0
  _serialDebug->printf("\n\r --------------------->\n\r");
  for (int i = 0; i < TESEO_RXBUF_LEN ; ++i) {
    _serialDebug->putc((int)buffer[i]);
  }
  _serialDebug->printf("\n\r<---------------------\n\r");
#endif
  
  switch(msg) {

  case PSTMGEOFENCE:
    status = (eStatus)parse_pstmgeofence(&pData.geofence_data, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_PSTM, msg, &pData);
    }
    break;
  case PSTMODO:
    status = (eStatus)parse_pstmodo(&pData.odo_data, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_PSTM, msg, &pData);
    }
    break;
  case PSTMDATALOG:
    status = (eStatus)parse_pstmdatalog(&pData.datalog_data, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_PSTM, msg, &pData);
    }
    break;
  case PSTMSGL:
    status = (eStatus)parse_pstmsgl(&pData.ack, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_PSTM, msg, &pData);
    }
    break;
  case PSTMSAVEPAR:
    status = (eStatus)parse_pstmsavepar(&pData.ack, buffer);
    if(status == TESEO_STATUS_SUCCESS) {
      outputHandler(LOC_OUTPUT_PSTM, msg, &pData);
    }
    break;

  default:
    break;
  }
  /* Recover the normal state */

}

void
Teseo::process(void)
{
  osEvent evt = queue.get();
  if (evt.status == osEventMessage) {
    struct _teseoMsg *message = (struct _teseoMsg *)evt.value.p;
    if (message->len > 0) {

      for(int m = 0; m < PSTM_NMEA_MSGS_NUM; m++) {
        _GetPSTMsg((ePSTMsg)m, message->buf);
      }
      for(int m = 0; m < NMEA_MSGS_NUM; m++) {
        _GetMsg((eMsg)m, message->buf);
      }
    }

    mpool.free(message);
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
  _ResetFast(_serialDebug);
}

gps_provider_error_t
Teseo::cfgMessageList(int level)
{
  int lowMask = 0x2;
  int highMask = 0x80000;

  if(level == 2) {
    lowMask = 0x18004F;
  }

  sprintf(_teseoCmd, "$PSTMCFGMSGL,%d,%d,%x,%x",
          0, /*NMEA 0*/
          1, /*Rate*/
          lowMask,
          highMask);

  SendCommand(_teseoCmd);
  
  return GPS_ERROR_NONE;  
}

gps_provider_error_t
Teseo::saveConfigParams(void)
{  
  sprintf(_teseoCmd, "$PSTMSAVEPAR");
  SendCommand(_teseoCmd);
  
  return GPS_ERROR_NONE;  
}

bool
Teseo::isGeofencingSupported(void)
{
  return true;
}

gps_provider_error_t
Teseo::enableGeofence(void)
{
  //$PSTMCFGGEOFENCE,<en>,<tol>*<checksum><cr><lf>
  sprintf(_teseoCmd, "$PSTMCFGGEOFENCE,%d,%d",1,1);
  SendCommand(_teseoCmd);

  return GPS_ERROR_NONE;
}

//FIXME!
gps_provider_error_t
Teseo::cfgGeofenceCircle(void)
{
  GPSGeofence::GeofenceCircle_t circle = {
    .id = 0,
    .enabled = 1,
    .tolerance = 1,
    .lat = 40.336055,
    .lon = 18.120611,
    .radius = 200
  };
  
  sprintf(_teseoCmd, "$PSTMCFGGEOCIR,%d,%d,%lf,%lf,%lf",
          circle.id,
          circle.enabled,
          circle.lat,
          circle.lon,
          circle.radius);
  SendCommand(_teseoCmd);

  sprintf(_teseoCmd, "$PSTMSAVEPAR");
  SendCommand(_teseoCmd);
  
  return GPS_ERROR_NONE;
}


gps_provider_error_t
Teseo::configGeofences(GPSGeofence *geofences[], unsigned geofenceCount)
{
  uint8_t trials;
     
  if(geofenceCount > MAX_GEOFENCES_NUM) {
    return GPS_ERROR_GEOFENCE_MAX_EXCEEDED;
  }
  
  for(uint8_t i = 0; i < geofenceCount; i++) {
    trials = 1;
    //printf("Teseo::configGeofences id=%d\r\n", (geofences[i]->getGeofenceCircle()).id);
    /*
    printf("Teseo::configGeofences en=%d\r\n", en);
    printf("Teseo::configGeofences tol=%d\r\n", (geofences[i]->getGeofenceCircle()).tolerance);
    printf("Teseo::configGeofences lat=%02f\r\n", (geofences[i]->getGeofenceCircle()).lat);
    printf("Teseo::configGeofences lon=%02f\r\n", (geofences[i]->getGeofenceCircle()).lon);
    printf("Teseo::configGeofences radius=%02f\r\n", (geofences[i]->getGeofenceCircle()).radius);
    */
    sprintf(_teseoCmd, "$PSTMGEOFENCECFG,%d,%d,%d,%lf,%lf,%lf",
            (geofences[i]->getGeofenceCircle()).id,
            (geofences[i]->getGeofenceCircle()).enabled,
            (geofences[i]->getGeofenceCircle()).tolerance,
            (geofences[i]->getGeofenceCircle()).lat,
            (geofences[i]->getGeofenceCircle()).lon,
            (geofences[i]->getGeofenceCircle()).radius);
    
    do{
      trials--; 
      
      SendCommand(_teseoCmd);
      /*
      sprintf(_teseoCmd, "$PSTMCFGGEOFENCE,%d,%d\n\r",1,1);
      
      //sprintf(_teseoCmd, "$PSTMGETSWVER,6");
      SendCommand(_teseoCmd);
      
      sprintf(_teseoCmd, "$PSTMSAVEPAR\n\r");
      SendCommand(_teseoCmd);
      */
      //printf("Teseo::configGeofences _teseoCmd=%s\r\n", _teseoCmd);
    } while (trials > 0);
  }
  //printf("Teseo::configGeofences sizeof(geofences)=%d numGeofences=%d strlen(_teseoCmd)=%d\r\n", sizeof(geofences), numGeofences, strlen(_teseoCmd));

  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::geofenceReq(void)
{
  sprintf(_teseoCmd, "$PSTMGEOFENCEREQ");
  SendCommand(_teseoCmd);

  return GPS_ERROR_NONE;
}

bool
Teseo::isDataloggingSupported(void)
{
  return true;
}

gps_provider_error_t
Teseo::enableDatalog(void)
{
  //$PSTMCFGLOG,<en>,<circ>,<rectype>,<oneshot>,<rate>,<speed>,<dist>*<checksum><cr><lf>
  sprintf(_teseoCmd, "$PSTMCFGLOG,%d,%d,%d,%d,%u,%u,%u",
          1, //Enable/Disable the log
          1, //Enable/Disable circular mode
          1, //Record type
          0, //Enable/Disable one shot mode
          5, //time interval in seconds between two consecutive logged records
          0, //minimum speed threshold
          0  //distance threshold
          );
  SendCommand(_teseoCmd);
  
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::configDatalog(GPSDatalog *datalog)
{
  //printf("Teseo::configDatalog 0x%03x\r\n", (datalog->getEnableBufferFullAlarm())<<1|(datalog->getEnableCircularBuffer()));
  //$PSTMLOGCREATE,<cfg>,<min-rate>,<min-speed>,<min-position>,<logmask>*<checksum><cr><lf>
  sprintf(_teseoCmd, "$PSTMLOGCREATE,%03x,%u,%u,%u,%d",
          (datalog->getEnableBufferFullAlarm())<<1|(datalog->getEnableCircularBuffer()),
          datalog->getMinRate(),
          datalog->getMinSpeed(),
          datalog->getMinPosition(),
          datalog->getLogMask()
          );
  SendCommand(_teseoCmd);

  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::startDatalog(void)
{
  sprintf(_teseoCmd, "$PSTMLOGSTART");
  SendCommand(_teseoCmd);

  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::stopDatalog(void)
{
  sprintf(_teseoCmd, "$PSTMLOGSTOP");
  SendCommand(_teseoCmd);

  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::eraseDatalog(void)
{
  sprintf(_teseoCmd, "$PSTMLOGERASE");
  SendCommand(_teseoCmd);

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

bool
Teseo::isOdometerSupported(void)
{
  return true;
}

gps_provider_error_t
Teseo::enableOdo(void)
{
  //$PSTMCFGODO,<en>,<enmsg>,<alarm>*<checksum><cr><lf>
  sprintf(_teseoCmd, "$PSTMCFGODO,1,1,1");
  SendCommand(_teseoCmd);
  
  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::startOdo(unsigned alarmDistance)
{
  sprintf(_teseoCmd, "$PSTMODOSTART,%08x", alarmDistance);
  SendCommand(_teseoCmd);

  return GPS_ERROR_NONE;
}

gps_provider_error_t
Teseo::stopOdo(void)
{
  sprintf(_teseoCmd, "$PSTMODOSTOP");
  SendCommand(_teseoCmd);

  return GPS_ERROR_NONE;
}
gps_provider_error_t
Teseo::resetOdo(void)
{
  /* TBI */
  return GPS_ERROR_NONE;
}

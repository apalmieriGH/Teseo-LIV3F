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

#ifndef __TESEO_H__
#define __TESEO_H__

#include "mbed.h"
#include "GPSProviderImplBase.h"

#include "TeseoConfig.h"
#include "NMEAParser.h"

#define STD_UART_BAUD               9600//    9600
#define FWU_UART_BAUD               115200
#define TESEO_I2C_ADDRESS           0x3A
#define POWERON_STABLE_SIGNAL_DELAY_MS  150

/** Indicates the outputted location information */
#define LOC_OUTPUT_LOCATION             (1)
#define LOC_OUTPUT_NMEA                 (2)
#define LOC_OUTPUT_PSTM                 (3)

#if 1
#define TESEO_LOG_INFO(...)
#else
#define TESEO_LOG_INFO(...) {                             \
        if (_serialDebug != NULL) {                     \
            (_serialDebug->printf(__VA_ARGS__));        \
        }                                               \
    }
#endif

/**
 * @brief Constant that indicates the maximum number of nmea messages to be processed.
 */
#define NMEA_MSGS_NUM 6 //Note: update this constant coherently to eMsg enum type

/**
 * @brief Constant that indicates the maximum number of proprietary nmea messages to be processed.
 */
#define PSTM_NMEA_MSGS_NUM 5 //Note: update this constant coherently to ePSTMsg enum type

/**
 * @brief Constant that indicates the maximum number of positions that can be stored.
 */
#define MAX_STOR_POS 64
   
/**
 * @brief Constant that indicates the lenght of the buffer that stores the GPS data read by the GPS expansion.
 */

#define TESEO_RXBUF_LEN		256//90
#define TESEO_RXQUEUE_LEN	8

/**
 * @brief Enumeration structure that containes the two success states of a process
 */
typedef enum {
  TESEO_STATUS_SUCCESS = 0, /**< Success status */
  TESEO_STATUS_FAILURE = 1  /**< Failure status */
} eStatus;

/** Location event definitions */
typedef enum {
    /** Start result event */
    TESEO_LOC_EVENT_START_RESULT,
    /** Stop result event */
    TESEO_LOC_EVENT_STOP_RESULT,
} eTeseoLocEventType;

/** Teseo Location state */
typedef enum {
    TESEO_LOC_STATE_IDLE,
    TESEO_LOC_STATE_RUN,
    TESEO_LOC_STATE_FEATURE,
    TESEO_LOC_STATE_DEBUG
} eTeseoLocState;

/**
 * @brief Enumeration structure that containes the two states of a debug process
 */
typedef enum {
  DEBUG_OFF = 0, /**< In this case, nothing will be printed on the console (nmea strings, positions and so on) */
  DEBUG_ON = 1   /**< In this case, nmea strings and just acquired positions will be printed on the console */
} eDebugState;

/**
 * @brief Data structure that contains the driver informations
 */
typedef struct TeseoData {
  eDebugState debug;      /**< Debug status */
  GPGGA_Infos gpgga_data; /**< $GPGGA Data holder */
  GNS_Infos   gns_data;   /**< $--GNS Data holder */
  GPGST_Infos gpgst_data; /**< $GPGST Data holder */
  GPRMC_Infos gprmc_data; /**< $GPRMC Data holder */
  GSA_Infos   gsa_data;   /**< $--GSA Data holder */
  GSV_Infos   gsv_data;   /**< $--GSV Data holder */
  
  Geofence_Infos geofence_data; /**< Geofence Data holder */
  Odometer_Infos odo_data;      /**< Odometer Data holder */
  Datalog_Infos datalog_data;   /**< Datalog Data holder */
  
  Ack_Info ack;                 /**<  */

} tTeseoData;

/** Application register this out callback function and Teseo class will pass outputted information to application */
typedef void (*teseo_app_output_callback)(uint32_t msgId, uint32_t msgType, tTeseoData *pData);
/** Application register this event callback function and Teseo class will pass internal processing event to application */
typedef void (*teseo_app_event_callback)(eTeseoLocEventType event, uint32_t data);

class Teseo : public GPSProviderImplBase {
public:

  typedef enum {
    TEST,
    GETSWVER,
    FORCESTANDBY,
    RFTESTON,
    RFTESTOFF,
    LOWPOWER,
    FWUPDATE
  } eCmd;

  /** NMEA messages types */
  typedef enum {
    GPGGA,
    GNS,
    GPGST,
    GPRMC,
    GSA,
    GSV,
  } eMsg;
  
  /** NMEA proprietary messages types */
  typedef enum {
    PSTMGEOFENCE,
    PSTMODO,
    PSTMDATALOG,
    PSTMSGL,
    PSTMSAVEPAR
  } ePSTMsg;

private:

  eTeseoLocState _locState;
    
  DigitalOut    _loc_led2;
  DigitalOut    _reset;
  DigitalOut    _pps;
  DigitalOut    _wakeup;
  PinName       _uartRx;
  PinName       _uartTx;
  
  Serial        *_uart;
  Serial        *_serialDebug;
  I2C           *_i2c;
        
  tTeseoData pData;
  GPGGA_Infos stored_positions[MAX_STOR_POS];

  int FwWaitAck();

  Thread *serialStreamThread;

  /**
   * Command string
   */
  char _teseoCmd[TESEO_RXBUF_LEN];

  /**
   * Message struct
   */
  struct _teseoMsg {
    uint8_t len;
    uint8_t buf[TESEO_RXBUF_LEN];
  };
  
public:
  
  /** Constructor: Teseo
   * Create the Teseo, accept specified configuration
   *
   * @param [in] resetPin
   *             GPIO pin to control location chip reset.
   * @param [in] wakeupPin
   *             GPIO pin to detect if the chip is still wakeup.
   * @param [in] ppsPin
   *             GPIO pin... .
   * @param [in] uartTxPin
   *             GPIO pin for serial Tx channel between the host and the GNSS controller.
   * @param [in] uartRxPin
   *             GPIO pin for serial Rx channel between the host and the GNSS controller.
   * @param [in] serialDebug
   *             The debug port for diagnostic messages; can be NULL.
   */
  Teseo(PinName resetPin,
        PinName wakeupPin,
        PinName ppsPin,
        PinName uartTxPin,
        PinName uartRxPin,
        Serial *serialDebug = NULL);

  /** Constructor: Teseo
   * Create the Teseo, accept specified configuration
   *
   * @param [in] resetPin
   *             GPIO pin to control location chip reset.
   * @param [in] wakeupPin
   *             GPIO pin to detect if the chip is still wakeup.
   * @param [in] ppsPin
   *             GPIO pin... .
   * @param [in] uartTxPin
   *             GPIO pin for serial Tx channel between the host and the GNSS controller.
   * @param [in] uartRxPin
   *             GPIO pin for serial Rx channel between the host and the GNSS controller.
   * @param [in] i2cBus
   *             I2C Bus not supported yet.
   * @param [in] serialDebug
   *             The debug port for diagnostic messages; can be NULL.
   */
  Teseo(PinName resetPin,
        PinName wakeupPin,
        PinName ppsPin,
        PinName uartTxPin,
        PinName uartRxPin,
        I2C    *i2cBus,
        Serial *serialDebug = NULL);
  
  /** Register output callback and event callback functions
   * @param app_output_cb Teseo class output the location and satellite information to application
   * @param app_event_cb Teseo class output the start and stop result to application
   */
  void TeseoLocRegOutput(teseo_app_output_callback app_output_cb, teseo_app_event_callback app_event_cb);
  
  void SendCommand(Teseo::eCmd c);
  void SendCommand(char *cmd);
  
  int EnableLowPower();
  
  void ReadSentence(Teseo::eMsg msg);
    
  eStatus WakeStatus(void){
    return _wakeup.read() ? TESEO_STATUS_SUCCESS : TESEO_STATUS_FAILURE;
  }
     
  void ReadProcess(void);

private:
  
  virtual bool setPowerMode(GPSProvider::PowerMode_t pwrMode);
  virtual void start(void);
  virtual void stop(void);
  virtual void process(void);
  virtual uint32_t ioctl(uint32_t command, void *arg);
  virtual void lpmGetImmediateLocation(void);
  virtual void reset(void);
  virtual const GPSProvider::LocationUpdateParams_t *getLastLocation(void) const;

  gps_provider_error_t cfgMessageList(int level);
  gps_provider_error_t saveConfigParams(void);
  
  /** Set NMEA stream verbosity */
  virtual void setVerboseMode(int level);

  /** Geofencing */
  virtual bool isGeofencingSupported(void);
  virtual gps_provider_error_t enableGeofence(void);
  virtual gps_provider_error_t configGeofences(GPSGeofence *geofences[], unsigned geofenceCount);
  virtual gps_provider_error_t geofenceReq(void);
  gps_provider_error_t cfgGeofenceCircle(void);

  /** Datalogging */
  virtual bool isDataloggingSupported(void);
  virtual gps_provider_error_t enableDatalog(void);
  virtual gps_provider_error_t configDatalog(GPSDatalog *datalog);
  virtual gps_provider_error_t startDatalog(void);
  virtual gps_provider_error_t stopDatalog(void);
  virtual gps_provider_error_t eraseDatalog(void);
  virtual gps_provider_error_t logReqStatus(void);
  virtual gps_provider_error_t logReqQuery(GPSProvider::LogQueryParams_t &logReqQuery);
  
  /* Odometer */
  virtual bool isOdometerSupported(void);
  virtual gps_provider_error_t enableOdo(void);
  virtual gps_provider_error_t startOdo(unsigned alarmDistance);
  virtual gps_provider_error_t stopOdo(void);
  virtual gps_provider_error_t resetOdo(void);

  void _InitUART(int br = STD_UART_BAUD);
  void _ResetFast(Serial *serialDebug = NULL);
  void _Reset(Serial *serialDebug = NULL);
  void _SendString(char *buf, int len);
  int _WakeUp();
  int _CRC(char *buf, int size);
  int _CheckI2C();
  /**
   * @brief  This function gets a chunck of NMEA messages
   * @param  msg NMEA message to search for
   * @retval eStatus TESEO_STATUS_SUCCESS if the parse process goes ok, TESEO_FAILURE if it doesn't
   */
  eStatus _GetMsg(Teseo::eMsg msg, uint8_t *buffer);
  /**
   * @brief  This function gets a chunck of PSTM NMEA messages
   * @param  msg PSTM NMEA message to search for
   * @retval eStatus TESEO_STATUS_SUCCESS if the parse process goes ok, TESEO_FAILURE if it doesn't
   */
  void _GetPSTMsg(Teseo::ePSTMsg msg, uint8_t *buffer);
  void _GetLocationMsg(Teseo::eMsg msg, uint8_t *buffer);
  
  void _LocLed2Set(void){
    _loc_led2.write(1);
  }
  void _LocLed2Reset(void){
    _loc_led2.write(0);
  }
  
  void outputHandler(uint32_t msgId, uint32_t msgType, tTeseoData *pData);
  void eventHandler(eTeseoLocEventType event, uint32_t data);
  
  teseo_app_output_callback appOutCb;
  teseo_app_event_callback appEventCb;
  
  MemoryPool<struct _teseoMsg, TESEO_RXQUEUE_LEN> mpool;
  Queue<struct _teseoMsg, TESEO_RXQUEUE_LEN> queue;
};

#endif /*__TESEO_H__*/

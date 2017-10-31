/**
 ******************************************************************************
 * @file    createTeseoGNSS.cpp
 * @author  AST/CL
 * @version V1.1.0
 * @date    May, 2017
 * @brief   .
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
 ******************************************************************************
 */

#include "mbed.h"
#include "Teseo.h"
#include "GPSProviderImplBase.h"

#include "TeseoConfig.h"

static void _AppOutputCallback(uint32_t msgId, uint32_t msgType, tTeseoData *pData);
static void _AppEventCallback(eTeseoLocEventType event, uint32_t data);
static char msg[256];

static char *geofenceCirclePosition[] = {
  "Unknown",
  "Outside",
  "Boundary",
  "Inside"
};

extern Serial serialDebug;
#define TESEO_APP_LOG_INFO(...) serialDebug.printf(__VA_ARGS__)

GPSProviderImplBase *
createGPSProviderInstance(void)
{
    static Teseo gnss(TESEO_PIN_RESET,
                      TESEO_PIN_WAKEUP,
                      TESEO_PIN_PPS,
                      TESEO_PIN_TX,
                      TESEO_PIN_RX,
                      &serialDebug);

    /* Register output callback and event callback functions */
    gnss.TeseoLocRegOutput(_AppOutputCallback, _AppEventCallback);

    return &gnss;
}

/** 
 * @brief  This function prints on the console the info about Fix data for single or combined satellite navigation system
 * @param  pData
 * @retval None
 */
static void
GetGNSMsgInfos(tTeseoData *pData)
{
//  char msg[256];
  
  TESEO_APP_LOG_INFO("\r\n");
  
  sprintf(msg, "Constellation:\t\t[ %s ]\t",
          pData->gns_data.constellation);
  TESEO_APP_LOG_INFO(msg);
  
  if (strcmp(pData->gns_data.constellation, "$GPGSV") == 0) {
    TESEO_APP_LOG_INFO("-- only GPS constellation is enabled\n\r");  
  }
  else if (strcmp(pData->gns_data.constellation, "$GLGSV") == 0) {
    TESEO_APP_LOG_INFO("-- only GLONASS constellation is enabled\n\r");
  }
  else if (strcmp(pData->gns_data.constellation, "$GAGSV") == 0) {
    TESEO_APP_LOG_INFO("-- only GALILEO constellation is enabled\n\r");
  }
  else if (strcmp(pData->gns_data.constellation, "$BDGSV") == 0) {
    TESEO_APP_LOG_INFO("-- only BEIDOU constellation is enabled\n\r");    
  }
  else if (strcmp(pData->gns_data.constellation, "$QZGSV") == 0) {
    TESEO_APP_LOG_INFO("-- only QZSS constellation is enabled\n\r");
  }
  else if (strcmp(pData->gns_data.constellation, "$GNGSV") == 0) {
    TESEO_APP_LOG_INFO("-- message to report all satellites for all enabled constellations\n\r");   
  }
  
  sprintf(msg, "UTC:\t\t\t[ %02d:%02d:%02d ]\n\r",
          pData->gns_data.utc.hh, 
          pData->gns_data.utc.mm, 
          pData->gns_data.utc.ss);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Latitude:\t\t[ %.0f' %d'' %c ]\n\r",
          (pData->gns_data.xyz.lat - ((int)pData->gns_data.xyz.lat % 100)) / 100, 
          ((int)pData->gns_data.xyz.lat % 100), 
          pData->gns_data.xyz.ns);          
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Longitude:\t\t[ %.0f' %d'' %c ]\n\r",
          (pData->gns_data.xyz.lon - ((int)pData->gns_data.xyz.lon % 100)) / 100, 
          ((int)pData->gns_data.xyz.lon % 100),
          pData->gns_data.xyz.ew);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Satellites locked:\t[ %d ]\n\r",
          pData->gns_data.sats);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"HDOP:\t\t\t[ %.01f ]\n\r",
          pData->gns_data.hdop);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Altitude:\t\t[ %.01f ]\n\r",
          pData->gns_data.xyz.alt);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Geoid infos:\t\t[ %.01f ]\n\r",
          pData->gns_data.geo_sep);
  TESEO_APP_LOG_INFO(msg);
  
  //  sprintf(msg,"ID - Checksum:\t\t[ *%x ]\n\r",
  //          (pData->gns_data.checksum);
  //  TESEO_APP_LOG_INFO(msg);
  
  TESEO_APP_LOG_INFO("\n\n\r");
  
  return;
}

/** 
 * @brief  This function prints on the console the info about GPS Pseudorange Noise Statistics
 * @param  pData
 * @retval None
 */
static void
GetGPGSTInfos(tTeseoData *pData)
{
//  char msg[256];
  
  TESEO_APP_LOG_INFO("\r\n");
    
  sprintf(msg, "UTC:\t\t\t[ %02d:%02d:%02d ]\n\r",
          pData->gpgst_data.utc.hh, 
          pData->gpgst_data.utc.mm, 
          pData->gpgst_data.utc.ss);
  TESEO_APP_LOG_INFO(msg);
    
  sprintf(msg,"EHPE:\t\t[ %.01f ]\n\r",
          pData->gpgst_data.EHPE);
  TESEO_APP_LOG_INFO(msg);
    
  sprintf(msg,"Semi-major Dev:\t\t[ %.01f ]\n\r",
          pData->gpgst_data.semi_major_dev);
  TESEO_APP_LOG_INFO(msg);
    
  sprintf(msg,"Semi-minor Dev:\t\t[ %.01f ]\n\r",
          pData->gpgst_data.semi_minor_dev);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Semi-maior Angle:\t\t[ %.01f ]\n\r",
          pData->gpgst_data.semi_major_angle);
  TESEO_APP_LOG_INFO(msg);
      
  sprintf(msg,"Lat Err Dev:\t\t[ %.01f ]\n\r",
          pData->gpgst_data.lat_err_dev);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Lon Err Dev:\t\t[ %.01f ]\n\r",
          pData->gpgst_data.lon_err_dev);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Alt Err Dev:\t\t[ %.01f ]\n\r",
          pData->gpgst_data.alt_err_dev);
  TESEO_APP_LOG_INFO(msg);
    
//  sprintf(msg,"ID - Checksum:\t\t[ *%x ]\n\r",
//          pData->gpgst_data.checksum);
//  TESEO_APP_LOG_INFO(msg);
    
  TESEO_APP_LOG_INFO("\n\n\r");

  return;
}

/** 
 * @brief  This function prints on the console the info about Recommended Minimum Specific GPS/Transit data got by the most recent reception process
 * @param  pData
 * @retval None
 */
static void
GetGPRMCInfos(tTeseoData *pData)
{
//  char msg[256];
  
  TESEO_APP_LOG_INFO("\r\n");
    
  sprintf(msg, "UTC:\t\t\t\t[ %02d:%02d:%02d ]\n\r",
          pData->gprmc_data.utc.hh, 
          pData->gprmc_data.utc.mm, 
          pData->gprmc_data.utc.ss);
  TESEO_APP_LOG_INFO(msg);
    
  sprintf(msg,"Status:\t\t\t\t[ %c ]\t\t",
          pData->gprmc_data.status);
  TESEO_APP_LOG_INFO(msg);
  if (pData->gprmc_data.status == 'A') {
    TESEO_APP_LOG_INFO("-- Valid (reported in 2D and 3D fix conditions)\n\r");
  }
  else if (pData->gprmc_data.status == 'V') {
    TESEO_APP_LOG_INFO("-- Warning (reported in NO FIX conditions)\n\r");
  }
  else {
    TESEO_APP_LOG_INFO("-- Unknown status\n\r");
  }
    
  sprintf(msg,"Latitude:\t\t\t[ %.0f' %02d'' %c ]\n\r",
          ((pData->gprmc_data.xyz.lat - ((int)pData->gprmc_data.xyz.lat % 100))) / 100, 
          ((int)pData->gprmc_data.xyz.lat % 100), 
          pData->gprmc_data.xyz.ns);          
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Longitude:\t\t\t[ %.0f' %02d'' %c ]\n\r",
          ((pData->gprmc_data.xyz.lon - ((int)pData->gprmc_data.xyz.lon % 100))) / 100, 
          ((int)pData->gprmc_data.xyz.lon % 100),
          pData->gprmc_data.xyz.ew);
  TESEO_APP_LOG_INFO(msg);
    
  sprintf(msg,"Speed over ground (knots):\t[ %.01f ]\n\r",
          pData->gprmc_data.speed);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Trackgood:\t\t\t[ %.01f ]\n\r",
          pData->gprmc_data.trackgood);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Date (ddmmyy):\t\t\t[ %d ]\n\r",
          pData->gprmc_data.date);
  TESEO_APP_LOG_INFO(msg);
    
  sprintf(msg,"Magnetic Variation:\t\t[ %.01f ]\n\r",
          pData->gprmc_data.mag_var);
  TESEO_APP_LOG_INFO(msg);
  
  if (pData->gprmc_data.mag_var_dir != 'E' &&
      pData->gprmc_data.mag_var_dir != 'W') {
    sprintf(msg,"Magnetic Var. Direction:\t[ - ]\n\r");
  }
  else {
    sprintf(msg,"Magnetic Var. Direction:\t[ %c ]\n\r",
          pData->gprmc_data.mag_var_dir);
  }
  TESEO_APP_LOG_INFO(msg);
  
//  sprintf(msg,"Checksum:\t\t[ *%x ]\n\r",
//          pData->gprmc_data.checksum);
//  TESEO_APP_LOG_INFO(msg);
    
  TESEO_APP_LOG_INFO("\n\n\r");

  return;
}

/** 
 * @brief  This function prints on the console the info about GNSS satellites got by the most recent reception process
 * @param  pData
 * @retval None
 */
static void
GetGSAMsgInfos(tTeseoData *pData)
{
//  char msg[256];
  
  TESEO_APP_LOG_INFO("\r\n");
  
  sprintf(msg, "Constellation:\t\t[ %s ]\t",
          pData->gsa_data.constellation);
  TESEO_APP_LOG_INFO(msg);
  
  if (strcmp(pData->gsa_data.constellation, "$GPGSA") == 0) {
    TESEO_APP_LOG_INFO("-- only GPS constellation is enabled\n\r");    
  }
  else if (strcmp(pData->gsa_data.constellation, "$GLGSA") == 0) {
    TESEO_APP_LOG_INFO("-- only GLONASS constellation is enabled\n\r");
  }
  else if (strcmp(pData->gsa_data.constellation, "$GAGSA") == 0) {
    TESEO_APP_LOG_INFO("-- only GALILEO constellation is enabled\n\r");
  }
  else if (strcmp(pData->gsa_data.constellation, "$BDGSA") == 0) {
    TESEO_APP_LOG_INFO("-- only BEIDOU constellation is enabled\n\r");
  }
  else if (strcmp(pData->gsa_data.constellation, "$GNGSA") == 0) {
    TESEO_APP_LOG_INFO("-- more than one constellation is enabled\n\r");   
  }
  
  sprintf(msg, "Operating Mode:\t\t[ %c ]\t\t",
          pData->gsa_data.operating_mode);
  TESEO_APP_LOG_INFO(msg);
  if (pData->gsa_data.operating_mode == 'A') {
    TESEO_APP_LOG_INFO("-- Auto (2D/3D)\n\r");
  }
  else if (pData->gsa_data.operating_mode == 'M') {
    TESEO_APP_LOG_INFO("-- Manual\n\r");
  }
  
  sprintf(msg,"Current Mode:\t\t[ %d ]\t\t",
          pData->gsa_data.current_mode);          
  TESEO_APP_LOG_INFO(msg);
  if (pData->gsa_data.current_mode == 1) {
    TESEO_APP_LOG_INFO("-- no fix available\n\r");
  }
  else if (pData->gsa_data.current_mode == 2) {
    TESEO_APP_LOG_INFO("-- 2D\n\r");
  }
  else if (pData->gsa_data.current_mode == 3) {
    TESEO_APP_LOG_INFO("-- 3D\n\r");
  }
  
  for (uint8_t i=0; i<12; i++) {  
    sprintf(msg,"SatPRN%02d:\t\t[ %d ]\n\r", i+1,
            pData->gsa_data.sat_prn[i]);
    TESEO_APP_LOG_INFO(msg);
  }
  
  sprintf(msg,"PDOP:\t\t\t[ %.01f ]\n\r",
          pData->gsa_data.pdop);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"HDOP:\t\t\t[ %.01f ]\n\r",
          pData->gsa_data.hdop);
  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"VDOP:\t\t\t[ %.01f ]\n\r",
          pData->gsa_data.vdop);
  TESEO_APP_LOG_INFO(msg);
  
//  sprintf(msg,"ID - Checksum:\t\t[ *%x ]\n\r",
//          pData->gsa_data.checksum);
//  TESEO_APP_LOG_INFO(msg);
  
  TESEO_APP_LOG_INFO("\n\n\r");
  
}
            
/** 
 * @brief  This function prints on the console the info about GNSS satellites got by the most recent reception process
 * @param  pData
 * @retval None
 */
static void
GetGSVMsgInfos(tTeseoData *pData)
{
  uint8_t i;
  uint8_t tot_sats = pData->gsv_data.tot_sats;
//  char msg[256];
  
  char degree_sym = 248;
  
  TESEO_APP_LOG_INFO("\r\n");
  
  sprintf(msg, "Constellation:\t\t[ %s ]\t",
          pData->gsv_data.constellation);
  TESEO_APP_LOG_INFO(msg);
  
  if (strcmp(pData->gsv_data.constellation, "$GPGSV") == 0) {
    TESEO_APP_LOG_INFO("-- message to report all GPS satellites\n\r");    
  }
  else if (strcmp(pData->gsv_data.constellation, "$GLGSV") == 0) {
    TESEO_APP_LOG_INFO("-- message to report all GLONASS satellites\n\r");
  }
  else if (strcmp(pData->gsv_data.constellation, "$GAGSV") == 0) {
    TESEO_APP_LOG_INFO("-- message to report all GALILEO satellites\n\r");
  }
  else if (strcmp(pData->gsv_data.constellation, "$BDGSV") == 0) {
    TESEO_APP_LOG_INFO("-- message to report all BEIDOU satellites\n\r");
  }
  else if (strcmp(pData->gsv_data.constellation, "$QZGSV") == 0) {
    TESEO_APP_LOG_INFO("-- message to report all QZSS satellites\n\r");
  }
  else if (strcmp(pData->gsv_data.constellation, "$GNGSV") == 0) {
    TESEO_APP_LOG_INFO("-- message to report all satellites for all enabled constellations\n\r");   
  }
  
  /* debug */
//  sprintf(msg,"Tot Messages:\t\t[ %d ]\n\r",
//          ((tTeseoData *)(handle->pData))->gsv_data.amount);
//  TESEO_APP_LOG_INFO(msg);
//  
//  sprintf(msg,"Message Num:\t\t[ %d ]\n\r",
//          ((tTeseoData *)(handle->pData))->gsv_data.number);
//  TESEO_APP_LOG_INFO(msg);
  
  sprintf(msg,"Num of Satellites:\t[ %d ]\n\r", tot_sats);
  TESEO_APP_LOG_INFO(msg);
  
  TESEO_APP_LOG_INFO("\n\r");
  
  for (i=0; i<tot_sats; i++) {
    sprintf(msg,"Sat%02dPRN:\t\t[ %03d ]\n\r", i+1, 
            pData->gsv_data.gsv_sat_i[i].prn);
    TESEO_APP_LOG_INFO(msg);
    
    sprintf(msg,"Sat%02dElev (%c):\t\t[ %03d ]\n\r", i+1, degree_sym,
            pData->gsv_data.gsv_sat_i[i].elev);
    TESEO_APP_LOG_INFO(msg);
    
    sprintf(msg,"Sat%02dAzim (%c):\t\t[ %03d ]\n\r", i+1, degree_sym,
            pData->gsv_data.gsv_sat_i[i].azim);
    TESEO_APP_LOG_INFO(msg);
    
    sprintf(msg,"Sat%02dCN0 (dB):\t\t[ %03d ]\n\r", i+1, 
            pData->gsv_data.gsv_sat_i[i].cn0);
    TESEO_APP_LOG_INFO(msg);  
    
    TESEO_APP_LOG_INFO("\n\r");
  }
  
  TESEO_APP_LOG_INFO("\r\n");
  
}

/** 
 * @brief  This function prints on the console the info Geofence
 * @param  pData
 * @retval None
 */
static void
GetGeofenceInfos(tTeseoData *pData)
{
//  char msg[256];
  
  TESEO_APP_LOG_INFO("\r\n");

  if(pData->geofence_data.op == GNSS_FEATURE_EN_MSG) {
    sprintf(msg, "Geofence Enabling:\t\t[ %s ]\t",
            pData->geofence_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
  }
  if(pData->geofence_data.op == GNSS_GEOFENCE_CFG_MSG) {
    sprintf(msg, "Geofence Configuration:\t\t[ %s ]\t",
          pData->geofence_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
  }
  if(pData->geofence_data.op == GNSS_GEOFENCE_STATUS_MSG) {
    sprintf(msg, "Geofence Status:\t\t[ %s ]\t",
          pData->geofence_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
    if(pData->geofence_data.result == 0) {
      TESEO_APP_LOG_INFO("\r\n");
      sprintf(msg, "Time/Date:\t\t%02d:%02d:%02d %02d/%02d/%04d\n",
          pData->geofence_data.timestamp.hh,
          pData->geofence_data.timestamp.mm,
          pData->geofence_data.timestamp.ss,
          pData->geofence_data.timestamp.day,
          pData->geofence_data.timestamp.month,
          pData->geofence_data.timestamp.year);
      TESEO_APP_LOG_INFO(msg);
      
      for(uint8_t i = 0; i<MAX_GEOFENCES_NUM; i++) {
        sprintf(msg, "Position circle[%d]:\t%s\n",
          i, geofenceCirclePosition[pData->geofence_data.status[i]]);
        TESEO_APP_LOG_INFO(msg);
      }
    }
  }
  if(pData->geofence_data.op == GNSS_GEOFENCE_ALARM_MSG) {
    sprintf(msg, "Geofence Alarm:\t\t[ %s ]\t",
          pData->geofence_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
    if(pData->geofence_data.result == 0) {
      TESEO_APP_LOG_INFO("\r\n");
      sprintf(msg, "Time:\t\t%02d:%02d:%02d\n",
          pData->geofence_data.timestamp.hh,
          pData->geofence_data.timestamp.mm,
          pData->geofence_data.timestamp.ss);
      TESEO_APP_LOG_INFO(msg);
      int i = pData->geofence_data.idAlarm;
      sprintf(msg, "Position circle[%d]:\t%s\n",
              i, geofenceCirclePosition[pData->geofence_data.status[i]]);
      TESEO_APP_LOG_INFO(msg);
    }
  }  
  TESEO_APP_LOG_INFO("\r\n");
  
}

/** 
 * @brief  This function prints on the console the info about Odometer
 * @param  pData
 * @retval None
 */
static void
GetOdometerInfos(tTeseoData *pData)
{
//  char msg[256];
  
  TESEO_APP_LOG_INFO("\r\n");
  
  if(pData->odo_data.op == GNSS_FEATURE_EN_MSG) {
    sprintf(msg, "Odometer Enabling:\t\t[ %s ]\t",
            pData->odo_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
  }
  if((pData->odo_data.op == GNSS_ODO_START_MSG) ||
     (pData->odo_data.op == GNSS_ODO_STOP_MSG)) {
    sprintf(msg, "Odometer Operation:\t\t[ %s ]\t",
          pData->odo_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
  }  
  TESEO_APP_LOG_INFO("\r\n");
  
}

/** 
 * @brief  This function prints on the console the info about Datalog
 * @param  pData
 * @retval None
 */
static void
GetDatalogInfos(tTeseoData *pData)
{
//  char msg[256];
  
  TESEO_APP_LOG_INFO("\r\n");
  
  if(pData->datalog_data.op == GNSS_FEATURE_EN_MSG) {
    sprintf(msg, "Datalog Enabling:\t\t[ %s ]\t",
            pData->datalog_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
  }
  if(pData->datalog_data.op == GNSS_DATALOG_CFG_MSG) {
    sprintf(msg, "Datalog Configuring:\t\t[ %s ]\t",
            pData->datalog_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
  }
  if(pData->datalog_data.op == GNSS_DATALOG_START_MSG) {
    sprintf(msg, "Datalog Start:\t\t[ %s ]\t",
            pData->datalog_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
  }
  if(pData->datalog_data.op == GNSS_DATALOG_STOP_MSG) {
    sprintf(msg, "Datalog Stop:\t\t[ %s ]\t",
            pData->datalog_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
  }
  if(pData->datalog_data.op == GNSS_DATALOG_ERASE_MSG) {
    sprintf(msg, "Datalog Erase:\t\t[ %s ]\t",
            pData->datalog_data.result ? "ERROR" : "OK");
    TESEO_APP_LOG_INFO(msg);
  }
  TESEO_APP_LOG_INFO("\r\n");
  
}

/** 
 * @brief  This function prints on the console the ack about Message List cfg
 * @param  pData
 * @retval None
 */
static void
GetMsgListAck(tTeseoData *pData)
{
//  char msg[256];
  
  TESEO_APP_LOG_INFO("\r\n");
  
  sprintf(msg, "Msg List config:\t\t[ %s ]\t",
          pData->ack ? "ERROR" : "OK");
  TESEO_APP_LOG_INFO(msg);
    
  TESEO_APP_LOG_INFO("\r\n");
}

/** 
 * @brief  This function prints on the console the ack about Message List cfg
 * @param  pData
 * @retval None
 */
static void
GetAck(tTeseoData *pData)
{
//  char msg[256];
  
  TESEO_APP_LOG_INFO("\r\n");
  
  sprintf(msg, "Params configuration:\t\t[ %s ]\t",
          pData->ack ? "ERROR" : "OK");
  TESEO_APP_LOG_INFO(msg);
    
  TESEO_APP_LOG_INFO("\r\n");
}

void
_AppOutputCallback(uint32_t msgId, uint32_t msgType, tTeseoData *pData)
{
  switch (msgId) {
  case LOC_OUTPUT_LOCATION: {
    // Output last location
    TESEO_APP_LOG_INFO("Loc: lat=%lf, lon=%lf, alt=%f\r\n", pData->gpgga_data.xyz.lat, pData->gpgga_data.xyz.lon, pData->gpgga_data.xyz.alt);
    break;
  }
  case LOC_OUTPUT_NMEA: {
    //return;
    Teseo::eMsg msg = (Teseo::eMsg)msgType;
    switch(msg) {
    case Teseo::GNS:
      // GET Fix data for single or combined Satellite navigation system
      GetGNSMsgInfos(pData);
      break;
      
    case Teseo::GPGST:
      // GET GPS Pseudorange Noise Statistics
      GetGPGSTInfos(pData);
      break;
      
    case Teseo::GPRMC:
      // GET Recommended Minimum Specific GPS/Transit data
      GetGPRMCInfos(pData);
      break;
      
    case Teseo::GSA:
      // GET GPS DOP and Active Satellites
      GetGSAMsgInfos(pData);
      break;
      
    case Teseo::GSV:
      // GET GPS Satellites in View
      GetGSVMsgInfos(pData);
      break;
      
    default:
      break;
    }
    break;
  }
  case LOC_OUTPUT_PSTM: {
    Teseo::ePSTMsg msg = (Teseo::ePSTMsg)msgType;
    switch(msg) {
    case Teseo::PSTMGEOFENCE:
      // GET Geofence info
      GetGeofenceInfos(pData);
      break;
    case Teseo::PSTMODO:
      // GET Geofence info
      GetOdometerInfos(pData);
      break;
    case Teseo::PSTMDATALOG:
      // GET Datalog info
      GetDatalogInfos(pData);
      break;
    case Teseo::PSTMSGL:
      // GET Message List ack
      GetMsgListAck(pData);
      break;
    case Teseo::PSTMSAVEPAR:
      // GET SAVE PAR ack
      GetAck(pData);
      break;
    default:
      break;
    }
    break;
  }
  
  default:
    break;
  }
}


void
_AppEventCallback(eTeseoLocEventType event, uint32_t data)
{
    switch (event) {
        case TESEO_LOC_EVENT_START_RESULT:
            if (data != 0) {
                TESEO_APP_LOG_INFO("start failed.\r\n");
            } else {
                TESEO_APP_LOG_INFO("start OK.\r\n");
            }
            break;
        case TESEO_LOC_EVENT_STOP_RESULT:
            if (data != 0) {
                TESEO_APP_LOG_INFO("stop failed.\r\n");
            } else {
                TESEO_APP_LOG_INFO("stop OK.\r\n");
            }
            break;
        default:
            break;
    }
}

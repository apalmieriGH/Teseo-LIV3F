/**
*******************************************************************************
* @file    NMEAParser.c
* @author  AST / Central Lab
* @version V1.0.0
* @date    18-May-2017
* @brief   NMEA sentence parser
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

#include "string.h"
#include "NMEAParser.h"
#include "NMEAUtils.h"

/** @defgroup Middlewares
 *  @brief Contains all platform independent modules (eg. NMEA Sentence Parser, ...).
 *  @{
 */

/** @defgroup ST
 *  @{
 */
 
/** @defgroup LIB_NMEA
 *  @{
 */

/** @defgroup NMEA_PARSER 
 * @{
 */

/** @addtogroup NMEA_PARSER_PUBLIC_FUNCTIONS
 * @{
 */
/**
 * @brief  Function that makes the parsing of the $GPGGA NMEA string with all Global Positioning System Fixed data.
 * @param  gpgga_data     Pointer to GPGGA_Infos struct
 * @param  NMEA           NMEA string read by the Gps expansion
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_gpgga(GPGGA_Infos *gpgga_data, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;
  
  if(NMEA == NULL)
    return status;
  
  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
    memset(app[i], 0, MAX_MSG_LEN);
  }
 
  for(unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {  
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';
      
      if (strcmp((char *)app[0], "$GPGGA") == 0) {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while(NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }
  
  if (valid_msg == 1) {
    unsigned int valid;
    sscanf ((char *)app[6], "%u", &valid);
    gpgga_data->valid = (GPS_ValidTypedef)valid;
    if (gpgga_data->valid == VALID) {
      scan_utc ((char *)app[1],  &gpgga_data->utc);
      sscanf   ((char *)app[2],  "%lf", &gpgga_data->xyz.lat);
      sscanf   ((char *)app[3],  "%c", &gpgga_data->xyz.ns);
      sscanf   ((char *)app[4],  "%lf", &gpgga_data->xyz.lon);
      sscanf   ((char *)app[5],  "%c", &gpgga_data->xyz.ew);
      sscanf   ((char *)app[7],  "%d", &gpgga_data->sats);
      sscanf   ((char *)app[8],  "%f", &gpgga_data->acc);
      sscanf   ((char *)app[9],  "%f", &gpgga_data->xyz.alt);
      sscanf   ((char *)app[10], "%c", &gpgga_data->xyz.mis);      
      sscanf   ((char *)app[11], "%d", &gpgga_data->geoid.height);
      sscanf   ((char *)app[12], "%c", &gpgga_data->geoid.mis);      
      sscanf   ((char *)app[13], "%d", &gpgga_data->update);      
      sscanf   ((char *)app[14], "%x", &gpgga_data->checksum);
            
      valid_msg = 0;
      status = PARSE_SUCC;  
    }    
  }
  return status;
} 
   
/**
 * @brief  Function that makes the parsing of the string read by the Gps expansion, capturing the right parameters from it.
 * @param  gns_data       Pointer to GNS_Infos struct
 * @param  NMEA           NMEA string read by the Gps expansion
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_gnsmsg (GNS_Infos *gns_data, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;
  
  if(NMEA == NULL)
    return status;
  
  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
    memset(app[i], 0, MAX_MSG_LEN);
  }
  
  for (unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';
      
      if ((strcmp((char *)app[0], "$GPGNS") == 0) ||
          (strcmp((char *)app[0], "$GLGNS") == 0) ||
          (strcmp((char *)app[0], "$GAGNS") == 0) ||
          (strcmp((char *)app[0], "$BDGNS") == 0) ||
          (strcmp((char *)app[0], "$QZGNS") == 0) ||
          (strcmp((char *)app[0], "$GNGNS") == 0))
      {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while (NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }  
  
  if (valid_msg == 1) {    
    sscanf   ((char *)app[0],  "%s", gns_data->constellation);    
    scan_utc ((char *)app[1],  &gns_data->utc); 
    sscanf   ((char *)app[2],  "%lf", &gns_data->xyz.lat);
    sscanf   ((char *)app[3],  "%c", &gns_data->xyz.ns);
    sscanf   ((char *)app[4],  "%lf", &gns_data->xyz.lon);
    sscanf   ((char *)app[5],  "%c", &gns_data->xyz.ew);
    sscanf   ((char *)app[6],  "%c", &gns_data->gps_mode);
    sscanf   ((char *)app[7],  "%c", &gns_data->glonass_mode);    
    sscanf   ((char *)app[8],  "%d", &gns_data->sats);
    sscanf   ((char *)app[9],  "%f", &gns_data->hdop);
    sscanf   ((char *)app[10], "%f", &gns_data->xyz.alt);
    sscanf   ((char *)app[11], "%f", &gns_data->geo_sep);
    sscanf   ((char *)app[12], "%c", &gns_data->dgnss_age);
    sscanf   ((char *)app[13], "%c", &gns_data->dgnss_ref);
    sscanf   ((char *)app[14], "%x", &gns_data->checksum);
    
    valid_msg = 0;
    status = PARSE_SUCC;
  }

  return status;
}
  
/**
 * @brief  Function that makes the parsing of the $GPGST NMEA string with GPS Pseudorange Noise Statistics.
 * @param  GPGST_Infos    Pointer to a GPGST_Infos struct
 * @param  NMEA           NMEA string read by the Gps expansion.
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_gpgst (GPGST_Infos *gpgst_data, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;
  
  if(NMEA == NULL)
    return status;
  
  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
    memset(app[i], 0, MAX_MSG_LEN);
  }

  for (unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';
      
      if (strcmp((char *)app[0], "$GPGST") == 0)
      {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while (NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }  
  
  if (valid_msg == 1) {         
    scan_utc ((char *)app[1], &gpgst_data->utc);
    sscanf   ((char *)app[2], "%f", &gpgst_data->EHPE);
    sscanf   ((char *)app[3], "%f", &gpgst_data->semi_major_dev);
    sscanf   ((char *)app[4], "%f", &gpgst_data->semi_minor_dev);
    sscanf   ((char *)app[5], "%f", &gpgst_data->semi_major_angle);
    sscanf   ((char *)app[6], "%f", &gpgst_data->lat_err_dev);
    sscanf   ((char *)app[7], "%f", &gpgst_data->lon_err_dev);
    sscanf   ((char *)app[8], "%f", &gpgst_data->alt_err_dev);
    sscanf   ((char *)app[9], "%x", &gpgst_data->checksum);
    
    valid_msg = 0;
    status = PARSE_SUCC;
  }

  return status;
}

/**
 * @brief  Function that makes the parsing of the $GPRMC NMEA string with Recommended Minimum Specific GPS/Transit data.
 * @param  GPRMC_Infos    Pointer to a GPRMC_Infos struct
 * @param  NMEA           NMEA string read by the Gps expansion.
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_gprmc (GPRMC_Infos *gprmc_data, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;
  
  if(NMEA == NULL)
    return status;
  
  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
    memset(app[i], 0, MAX_MSG_LEN);
  }
  
  for (unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';
      
      if (strcmp((char *)app[0], "$GPRMC") == 0)
      {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while (NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }  

  if (valid_msg == 1) {      
    scan_utc ((char *)app[1],  &gprmc_data->utc);
    sscanf   ((char *)app[2],  "%c", &gprmc_data->status); 
    sscanf   ((char *)app[3],  "%lf", &gprmc_data->xyz.lat);
    sscanf   ((char *)app[4],  "%c", &gprmc_data->xyz.ns);
    sscanf   ((char *)app[5],  "%lf", &gprmc_data->xyz.lon);
    sscanf   ((char *)app[6],  "%c", &gprmc_data->xyz.ew);
    sscanf   ((char *)app[7],  "%f", &gprmc_data->speed);
    sscanf   ((char *)app[8],  "%f", &gprmc_data->trackgood);
    sscanf   ((char *)app[9],  "%d", &gprmc_data->date);
    sscanf   ((char *)app[10], "%f", &gprmc_data->mag_var);    
    sscanf   ((char *)app[11], "%c", &gprmc_data->mag_var_dir);
    /* WARNING: from received msg, it seems there is another data (app[12]) before the checksum */
    sscanf   ((char *)app[13], "%x", &gprmc_data->checksum);
    
    valid_msg = 0;
    status = PARSE_SUCC;
  }

  return status;
}

/**
 * @brief  Function that makes the parsing of the string read by the Gps expansion, capturing the right parameters from it.
 * @param  GSA_Infos      Pointer to a GSA_Infos struct
 * @param  NMEA           NMEA string read by the Gps expansion.
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_gsamsg (GSA_Infos *gsa_data, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;
  
  if(NMEA == NULL)
    return status;
  
  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<19; i++) {
    memset(app[i], 0, 19);
  }
  
  for (unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';
      
      if ((strcmp((char *)app[0], "$GPGSA") == 0) ||
          (strcmp((char *)app[0], "$GLGSA") == 0) ||
          (strcmp((char *)app[0], "$GAGSA") == 0) ||
          (strcmp((char *)app[0], "$BDGSA") == 0) ||  
          (strcmp((char *)app[0], "$GNGSA") == 0))
      {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while (NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }  
  
  if (valid_msg == 1) {
    sscanf ((char *)app[0],  "%s", gsa_data->constellation);
    sscanf ((char *)app[1],  "%c", &gsa_data->operating_mode);
    sscanf ((char *)app[2],  "%d", &gsa_data->current_mode);
    for (uint8_t i=0; i<MAX_SAT_NUM; i++) {
      sscanf ((char *)app[3+i], "%d", &gsa_data->sat_prn[i]);
    }
    sscanf ((char *)app[15], "%f", &gsa_data->pdop);
    sscanf ((char *)app[16], "%f", &gsa_data->hdop);
    sscanf ((char *)app[17], "%f", &gsa_data->vdop);
    sscanf ((char *)app[18], "%x", &gsa_data->checksum);
    
    valid_msg = 0;
    status = PARSE_SUCC;
  }
  
  return status;
}

/**
 * @brief  Function that makes the parsing of the string read by the Gps expansion, capturing the right parameters from it.
 * @param  GSV_Infos      Pointer to a GSV_Infos struct
 * @param  NMEA           NMEA string read by the Gps expansion.
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_gsvmsg(GSV_Infos *gsv_data, uint8_t *NMEA)
{
  uint8_t app[32][16];
  uint8_t app_idx;
  uint8_t gsv_idx = 0;
  int msg_amount = 1;
  int curr_msg;
  uint8_t right_msg = 0;
  uint8_t valid_gsv_msg = 0;
  unsigned i, j, k;
  unsigned l = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;
  
  if(NMEA == NULL)
    return status;
  
  while (right_msg < msg_amount)
  {
    /* clear the app[][] buffer */ 
    for (uint8_t pos=0; pos<32; pos++) {
      memset(app[pos], 0, 16);
    }
        
    for (i = l, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
    {
      if (NMEA[i] == ',') {
        app[j][k] = '\0';
        
        if ((strcmp((char *)app[0], "$GPGSV") == 0) ||
            (strcmp((char *)app[0], "$GLGSV") == 0) ||
            (strcmp((char *)app[0], "$GAGSV") == 0) ||
            (strcmp((char *)app[0], "$BDGSV") == 0) ||  
            (strcmp((char *)app[0], "$QZGSV") == 0) ||    
            (strcmp((char *)app[0], "$GNGSV") == 0))
        {
          j++;
          k = 0;           
          valid_gsv_msg = 1;
          continue;
        }
        else {
          while (NMEA[i++] != '\n');
          j = k = 0;
        }
      }
      if ((NMEA[i] == '*') && (k!=0)) {
        j++;
        k=0;
      }
      app[j][k++] = NMEA[i];
    }  
    
    l = i;
    
    if (valid_gsv_msg == 1) {
      valid_gsv_msg = 0;
      sscanf((char *)app[1], "%d", &msg_amount);
      sscanf((char *)app[2], "%d", &curr_msg);    
      if (curr_msg == right_msg+1) {
        valid_gsv_msg = 1;
        right_msg++;
      }
    }
    else {
      right_msg = msg_amount;
    }
    
    if (valid_gsv_msg == 1) {      
      sscanf((char *)app[0], "%s", gsv_data->constellation);      
      sscanf((char *)app[1], "%d", &gsv_data->amount);
      sscanf((char *)app[2], "%d", &gsv_data->number);            
      sscanf((char *)app[3], "%d", &gsv_data->tot_sats);
      app_idx = 4;    
      while (app[app_idx][0] != '*') {      
        sscanf((char *)app[app_idx++], "%d", &gsv_data->gsv_sat_i[gsv_idx].prn);
        sscanf((char *)app[app_idx++], "%d", &gsv_data->gsv_sat_i[gsv_idx].elev);
        sscanf((char *)app[app_idx++], "%d", &gsv_data->gsv_sat_i[gsv_idx].azim);
        if (app[app_idx][0] != '*') {
          sscanf((char *)app[app_idx++], "%d", &gsv_data->gsv_sat_i[gsv_idx].cn0);
        }
        else {
          sscanf("", "%d", &gsv_data->gsv_sat_i[gsv_idx].cn0);
        }
        gsv_idx++;       
      }
      
      valid_gsv_msg = 0;
      
      status = PARSE_SUCC;
    }
  }
  
  return status;
}

/**
 * @brief  
 * @param  Geofence_Infos Pointer to a Geofence_Infos struct
 * @param  NMEA           NMEA string read by the Gps expansion.
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_pstmgeofence(Geofence_Infos *geofence_data, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;

  if(NMEA == NULL)
    return status;

  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
    memset(app[i], 0, MAX_MSG_LEN);
  }
 
  for(unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {  
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';

      if ((strcmp((char *)app[0], "$PSTMCFGGEOFENCEOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMCFGGEOFENCEERROR") == 0) ||
          (strcmp((char *)app[0], "$PSTMGEOFENCECFGOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMGEOFENCECFGERROR") == 0) ||
          (strcmp((char *)app[0], "$PSTMGEOFENCESTATUS") == 0) ||
          (strcmp((char *)app[0], "$PSTMGEOFENCE") == 0) ||
          (strcmp((char *)app[0], "$PSTMGEOFENCEREQERROR") == 0)) {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while(NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }
  
  if (valid_msg == 1) {
    /* Enabling */
    if (strcmp((char *)app[0], "$PSTMCFGGEOFENCEOK") == 0) {
      geofence_data->op = GNSS_FEATURE_EN_MSG;
      geofence_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMCFGGEOFENCEERROR") == 0) {
      geofence_data->op = GNSS_FEATURE_EN_MSG;
      geofence_data->result = 1;
    }
    /* Configuring */
    if (strcmp((char *)app[0], "$PSTMGEOFENCECFGOK") == 0) {
      geofence_data->op = GNSS_GEOFENCE_CFG_MSG;
      geofence_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMGEOFENCECFGERROR") == 0) {
      geofence_data->op = GNSS_GEOFENCE_STATUS_MSG;
      geofence_data->result = 1;
    }
    /* Querying Status */
    if (strcmp((char *)app[0], "$PSTMGEOFENCESTATUS") == 0) {
      geofence_data->op = GNSS_GEOFENCE_STATUS_MSG;
      geofence_data->result = 0;
      sscanf((char *)app[1], "%02d%02d%02d", &geofence_data->timestamp.hh,&geofence_data->timestamp.mm,&geofence_data->timestamp.ss);
      sscanf((char *)app[2], "%04d%02d%02d", &geofence_data->timestamp.year,&geofence_data->timestamp.month,&geofence_data->timestamp.day);
      for(uint8_t i = 0; i<MAX_GEOFENCES_NUM; i++) {
        sscanf((char *)app[3+i], "%d", &geofence_data->status[i]);
      }
    }
    /* Alarm Msg */
    if (strcmp((char *)app[0], "$PSTMGEOFENCE") == 0) {
      geofence_data->op = GNSS_GEOFENCE_ALARM_MSG;
      geofence_data->result = 0;
      sscanf((char *)app[1], "%02d%02d%02d", &geofence_data->timestamp.hh,&geofence_data->timestamp.mm,&geofence_data->timestamp.ss);
      sscanf((char *)app[2], "%04d%02d%02d", &geofence_data->timestamp.year,&geofence_data->timestamp.month,&geofence_data->timestamp.day);
      sscanf((char *)app[3], "%d", &geofence_data->idAlarm);
      sscanf((char *)app[9], "%d", &geofence_data->status[geofence_data->idAlarm]);
     }
    
    valid_msg = 0;
    status = PARSE_SUCC;
  }
  return status;
}

/**
 * @brief  
 * @param  Odometer_Infos Pointer to a Odometer_Infos struct
 * @param  NMEA           NMEA string read by the Gps expansion.
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_pstmodo(Odometer_Infos *odo_data, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;

  if(NMEA == NULL)
    return status;

  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
    memset(app[i], 0, MAX_MSG_LEN);
  }
 
  for(unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {  
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';

      if ((strcmp((char *)app[0], "$PSTMCFGODOOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMCFGODOERROR") == 0) ||
          (strcmp((char *)app[0], "$PSTMODOSTARTOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMODOSTARTERROR") == 0) ||
          (strcmp((char *)app[0], "$PSTMODOSTOPOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMODOSTOPERROR") == 0)) {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while(NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }
  
  if (valid_msg == 1) {
    /* Enabling */
    if (strcmp((char *)app[0], "$PSTMCFGODOOK") == 0) {
      odo_data->op = GNSS_FEATURE_EN_MSG;
      odo_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMCFGODOERROR") == 0) {
      odo_data->op = GNSS_FEATURE_EN_MSG;
      odo_data->result = 1;
    }
    
    /* Start */
    if (strcmp((char *)app[0], "$PSTMODOSTARTOK") == 0) {
      odo_data->op = GNSS_ODO_START_MSG;
      odo_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMODOSTARTERROR") == 0) {
      odo_data->op = GNSS_ODO_START_MSG;
      odo_data->result = 1;
    }
    
    /* Stop */
    if (strcmp((char *)app[0], "$PSTMODOSTOPOK") == 0) {
      odo_data->op = GNSS_ODO_STOP_MSG;
      odo_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMODOSTOPERROR") == 0) {
      odo_data->op = GNSS_ODO_STOP_MSG;
      odo_data->result = 1;
    }

    valid_msg = 0;
    status = PARSE_SUCC;
  }
  return status;
}

/**
 * @brief  
 * @param  Datalog_Infos Pointer to a Datalog_Infos struct
 * @param  NMEA           NMEA string read by the Gps expansion.
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_pstmdatalog(Datalog_Infos *datalog_data, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;

  if(NMEA == NULL)
    return status;

  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
    memset(app[i], 0, MAX_MSG_LEN);
  }
 
  for(unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {  
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';

      if ((strcmp((char *)app[0], "$PSTMCFGLOGOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMCFGLOGERROR") == 0) ||
          (strcmp((char *)app[0], "$PSTMLOGCREATEOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMLOGCREATEERROR") == 0) ||
          (strcmp((char *)app[0], "$PSTMLOGSTARTOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMLOGSTARTERROR") == 0) ||
          (strcmp((char *)app[0], "$PSTMLOGSTOPOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMLOGSTOPERROR") == 0) ||
          (strcmp((char *)app[0], "$PSTMLOGERASEOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMLOGERASEERROR") == 0)) {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while(NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }
  
  if (valid_msg == 1) {
    /* Enabling */
    if (strcmp((char *)app[0], "$PSTMCFGLOGOK") == 0) {
      datalog_data->op = GNSS_FEATURE_EN_MSG;
      datalog_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMCFGLOGERROR") == 0) {
      datalog_data->op = GNSS_FEATURE_EN_MSG;
      datalog_data->result = 1;
    }
    /* Configuring */
    if (strcmp((char *)app[0], "$PSTMLOGCREATEOK") == 0) {
      datalog_data->op = GNSS_DATALOG_CFG_MSG;
      datalog_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMLOGCREATEERROR") == 0) {
      datalog_data->op = GNSS_DATALOG_CFG_MSG;
      datalog_data->result = 1;
    }
    /* Start */
    if (strcmp((char *)app[0], "$PSTMLOGSTARTOK") == 0) {
      datalog_data->op = GNSS_DATALOG_START_MSG;
      datalog_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMLOGSTARTERROR") == 0) {
      datalog_data->op = GNSS_DATALOG_START_MSG;
      datalog_data->result = 1;
    }
    /* Stop */
    if (strcmp((char *)app[0], "$PSTMLOGSTOPOK") == 0) {
      datalog_data->op = GNSS_DATALOG_STOP_MSG;
      datalog_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMLOGSTOPERROR") == 0) {
      datalog_data->op = GNSS_DATALOG_STOP_MSG;
      datalog_data->result = 1;
    }
    /* Erase */
    if (strcmp((char *)app[0], "$PSTMLOGERASEOK") == 0) {
      datalog_data->op = GNSS_DATALOG_ERASE_MSG;
      datalog_data->result = 0;
    }
    if (strcmp((char *)app[0], "$PSTMLOGERASEERROR") == 0) {
      datalog_data->op = GNSS_DATALOG_ERASE_MSG;
      datalog_data->result = 1;
    }

    valid_msg = 0;
    status = PARSE_SUCC;
  }
  return status;
}

/**
 * @brief  
 * @param  Ack_Info       Ack from Teseo
 * @param  NMEA           NMEA string read by the Gps expansion.
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_pstmsgl(Ack_Info *ack, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;

  if(NMEA == NULL)
    return status;

  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
    memset(app[i], 0, MAX_MSG_LEN);
  }
 
  for(unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {  
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';

      if ((strcmp((char *)app[0], "$PSTMCFGMSGLOK") == 0) ||
          (strcmp((char *)app[0], "$PSTMCFGMSGLERROR") == 0)) {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while(NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }
  
  if (valid_msg == 1) {
    /* Enabling */
    if (strcmp((char *)app[0], "$PSTMCFGMSGLOK") == 0) {
      *ack = 0;
    }
    if (strcmp((char *)app[0], "$PSTMCFGMSGLERROR") == 0) {
      *ack = 1;
    }

    valid_msg = 0;
    status = PARSE_SUCC;
  }
  return status;
}

/**
 * @brief  
 * @param  Ack_Info       Ack from Teseo
 * @param  NMEA           NMEA string read by the Gps expansion.
 * @retval ParseStatus_Typedef PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
 */
ParseStatus_Typedef parse_pstmsavepar(Ack_Info *ack, uint8_t *NMEA)
{
  uint8_t app[MAX_MSG_LEN][MAX_MSG_LEN];
  uint8_t valid_msg = 0;
  
  ParseStatus_Typedef status = PARSE_FAIL;

  if(NMEA == NULL)
    return status;

  /* clear the app[][] buffer */ 
  for (uint8_t i=0; i<MAX_MSG_LEN; i++) {
    memset(app[i], 0, MAX_MSG_LEN);
  }
 
  for(unsigned i = 0, j = 0, k = 0; NMEA[i] != '\n' && i < strlen((char *)NMEA) - 1; i++)
  {  
    if ((NMEA[i] == ',') || (NMEA[i] == '*')) {
      app[j][k] = '\0';

      if ((strcmp((char *)app[0], "$PSTMSAVEPAROK") == 0) ||
          (strcmp((char *)app[0], "$PSTMSAVEPARERROR") == 0)) {
        j++;
        k = 0;
        valid_msg = 1;
        continue;
      }
      else {
        while(NMEA[i++] != '\n');
        j = k = 0;
      }
    }
    app[j][k++] = NMEA[i];
  }
  
  if (valid_msg == 1) {
    /* Enabling */
    if (strcmp((char *)app[0], "$PSTMSAVEPAROK") == 0) {
      *ack = 0;
    }
    if (strcmp((char *)app[0], "$PSTMSAVEPARERROR") == 0) {
      *ack = 1;
    }

    valid_msg = 0;
    status = PARSE_SUCC;
  }
  return status;
}

/**
 * @brief  This function makes a copy of the datas stored into gps_t into the 'info' param
 * @param  info  Instance of a GPGGA_Infos object where there are the infos to be copied
 * @param  gps_t Instance of a GPGGA_Infos object pointer where the infos stored into gps_t have to be copied
 * @retval None
 */
void copy_data(GPGGA_Infos *info, GPGGA_Infos gps_t){
  info->acc          = gps_t.acc;
  info->geoid.height = gps_t.geoid.height;
  info->geoid.mis    = gps_t.geoid.mis;
  info->sats         = gps_t.sats;
  info->update       = gps_t.update;
  info->utc.hh       = gps_t.utc.hh;
  info->utc.mm       = gps_t.utc.mm;
  info->utc.ss       = gps_t.utc.ss;
  info->utc.utc      = gps_t.utc.utc;
  info->valid        = gps_t.valid;
  info->xyz.alt      = gps_t.xyz.alt;
  info->xyz.lat      = gps_t.xyz.lat;
  info->xyz.lon      = gps_t.xyz.lon;
  info->xyz.ew       = gps_t.xyz.ew;
  info->xyz.ns       = gps_t.xyz.ns;
  info->xyz.mis      = gps_t.xyz.mis;
  info->checksum     = gps_t.checksum;
}   
/**
* @}
*/
   
/**
* @}
*/

/**
* @}
*/

/**
 * @}
 */

/**
* @}
*/

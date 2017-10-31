/**
*******************************************************************************
* @file    NMEAParser.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NMEA_PARSER_H
#define __NMEA_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif
  
#include <stdio.h>
#include <stdint.h>
  
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

/** @addtogroup NMEA_PARSER_CONSTANTS_DEFINITIONS
 * @{
 */
/**
 * @brief Constant that indicates the maximum lenght of an NMEA sentence
 */
#define MAX_NMEA_SENTENCE_LEN 80
   
/**
 * @brief Constant that indicates the maximum number of satellites.
 */  
#define MAX_SAT_NUM 12  
  
/**
 * @brief Constant that indicates the maximum lenght of a string.
 */  
#define MAX_STR_LEN 32   
  
/**
 * @brief Constant that indicates the maximum lenght of NMEA message field.
 */
#define MAX_MSG_LEN 32//19
/**
 * @}
 */  

#define MAX_GEOFENCES_NUM 8

/** @addtogroup NMEA_PARSER_TYPES_DEFINITIONS
 * @{
 */
/**
 * @brief Enumeration structure that containes the types of feature messages
 */
typedef enum {
  GNSS_FEATURE_EN_MSG  = 0,
  GNSS_GEOFENCE_CFG_MSG,
  GNSS_GEOFENCE_STATUS_MSG,
  GNSS_GEOFENCE_ALARM_MSG,
  GNSS_ODO_START_MSG,
  GNSS_ODO_STOP_MSG,
  GNSS_DATALOG_CFG_MSG,
  GNSS_DATALOG_START_MSG,
  GNSS_DATALOG_STOP_MSG,
  GNSS_DATALOG_ERASE_MSG
} ParseFeatureMsg_Typedef;

/**
 * @brief Enumeration structure that containes the two success states of a parsing process
 */
typedef enum {
  PARSE_SUCC = 0,  /**< Success status */
  PARSE_FAIL = 1 /**< Fail status */
} ParseStatus_Typedef;
  
/**
 * @brief Enumeration structure that containes the tipologies of Gps fixing process got from a NMEA string
 */
typedef enum {
  INVALID = 0,          /**< Invalid Fix status */
  VALID = 1,            /**< Valid Fix status */
  DGPS_FIX = 2,         /**< DGPS Fix status */
  PPS_FIX = 3,          /**< PPS Fix status */
  REAL_TIME = 4,        /**< Real Time Fix status */
  FLOAT_REAL_TIME = 5,  /**< Float Real Time Fix status */
  ESTIMATED = 6,        /**< Estimated Fix status */
  MANUAL_MODE = 7,      /**< Manual Mode Fix status */
  SIMULATION_MODE = 8   /**< Simulation Mode Fix status */
} GPS_ValidTypedef;

/**
 * @brief Data structure that contains the coordinates informations
 */
typedef struct Coords {
  double lat;   /**< Latitude */
  double lon;   /**< Longitude */
  float alt;   /**< Altitude */
  uint8_t ns;  /**< Nord / Sud latitude type */
  uint8_t ew;  /**< East / West longitude type */
  uint8_t mis; /**< Altitude unit misure */
} Coords;
  
/**
 * @brief Data structure that contains the Gps geoids informations
 */
typedef struct Geoid_Info {
  int height;   /**< Geoid height */
  uint8_t mis;  /**< Geoid height misure unit */
} Geoid_Info;
  
/**
 * @brief Data structure that contains the UTC informations
 */
typedef struct UTC_Info {
  int utc; /**< UTC Info */
  int hh;  /**< Hours */
  int mm;  /**< Minutes */
  int ss;  /**< Seconds */
} UTC_Info;

/**
 * @brief Data structure that contains the UTC informations
 */
typedef struct GSV_SAT_Info {
  int prn;  /**< PRN */
  int elev; /**< Elevation of satellite in degree, 0 ... 90 */
  int azim; /**< Azimuth of satellite in degree, ref. "North, " 0 ... 359 */
  int cn0;  /**< Carrier to noise ratio for satellite in dB, 0 ... 99 */
} GSV_SAT_Info;  

/**
 * @brief Data structure that contains all of the informations about the GPS position 
 */
typedef struct GPGGA_Infos {
  UTC_Info utc;           /**< UTC Time */
  Coords xyz;             /**< Coords data member */
  float acc;              /**< GPS Accuracy */
  int sats;           /**< Number of satellities acquired */
  GPS_ValidTypedef valid; /**< GPS Signal fix quality */
  Geoid_Info geoid;   /**< Geoids data info member */
  int update;         /**< Update time from the last acquired GPS Info */
  int checksum;           /**< Checksum of the message bytes */
} GPGGA_Infos;

/**
 * @brief Data structure that contains all of the informations about the fix data for single or 
 *        combined satellite navigation system.
 */
typedef struct GNS_Infos {
  char constellation[MAX_STR_LEN]; /**< Constellation enabled: GPGNS (GPS), GLGNS (GLONASS), GAGNS (GALILEO), BDGNS (BEIDOU), QZGNS (QZSS), GNGNS (more than one) */ 
  UTC_Info utc;                    /**< UTC Time */
  Coords xyz;                      /**< Coords data member */
  char gps_mode;                   /**< N = NO Fix, A = Autonomous, D = Differential GPS, E = Estimated (dead reckoning mode) */
  char glonass_mode;               /**< N = NO Fix, A = Autonomous, D = Differential Glonass, E = Estimated (dead reckoning mode) */
  int sats;                    /**< Number of satellities acquired */
  float hdop;                      /**< Horizontal Dilution of Precision, max: 99.0 */
  float geo_sep;                   /**< Geoidal separation, meter */
  char dgnss_age;                  /**< Not supported */
  char dgnss_ref;                  /**< Not supported */
  int checksum;                    /**< Checksum of the message bytes */
} GNS_Infos;

/**
 * @brief Data structure that contains all of the GPS Pseudorange Noise Statistics.
 */
typedef struct GPGST_Infos {
  UTC_Info utc;           /**< UTC Time */
  float EHPE;             /**< Equivalent Horizontal Position Error */
  float semi_major_dev;   /**< Standard deviation (meters) of semi-major axis of error ellipse */
  float semi_minor_dev;   /**< Standard deviation (meters) of semi-minor axis of error ellipse */
  float semi_major_angle; /**< Orientation of semi-major axis of error ellipse (true north degrees) */
  float lat_err_dev;      /**< Standard deviation (meters) of latitude error */
  float lon_err_dev;      /**< Standard deviation (meters) of longitude error */
  float alt_err_dev;      /**< Standard deviation (meters) of altitude error */
  int checksum;           /**< Checksum of the message bytes */
} GPGST_Infos;

/**
 * @brief Data structure that contains all the Recommended Minimum Specific GPS/Transit data.
 */
typedef struct GPRMC_Infos {
  UTC_Info utc;           /**< UTC Time */
  char status;            /**< “A” = valid, “V” = Warning */
  Coords xyz;             /**< Coords data member */
  float speed;            /**< Speed over ground in knots */
  float trackgood;        /**< Course made good */
  int date;               /**< Date of Fix */
  float mag_var;          /**< Magnetic Variation */
  char mag_var_dir;       /**< Magnetic Variation Direction */
  int checksum;           /**< Checksum of the message bytes */
} GPRMC_Infos;

/**
 * @brief Data structure that contains all of the informations about the GSA satellites 
 */
typedef struct GSA_Infos {
  char constellation[MAX_STR_LEN]; /**< Constellation enabled: GPGSA (GPS), GLGSA (GLONASS), GAGSA (GALILEO), BDGSA (BEIDOU), GNGSA (more than one) */ 
  char operating_mode;             /**< Operating Mode: 'M' = Manual, 'A' = Auto (2D/3D) */
  int current_mode;                /**< Current Mode: 1. no fix available, 2. 2D, 3. 3D */
  int sat_prn[MAX_SAT_NUM];        /**< Satellites list used in position fix (max N 12) */
  float pdop;                      /**< Position Dilution of Precision, max: 99.0 */
  float hdop;                      /**< Horizontal Dilution of Precision, max: 99.0 */
  float vdop;                      /**< Vertical Dilution of Precision, max: 99.0 */
  int checksum;                    /**< Checksum of the message bytes */
} GSA_Infos;
  
/**
 * @brief Data structure that contains all of the informations about the GSV satellites 
 */
typedef struct GSV_Infos {
  char constellation[MAX_STR_LEN];      /**< Constellation enabled: GPGSV (GPS), GLGSV (GLONASS), GAGSV (GALILEO), BDGSV (BEIDOU), QZGSV (QZSS), GNGSV (more than one) */ 
  int amount;                           /**< Total amount of GSV messages, max. 3 */
  int number;                           /**< Continued GSV number of this message */
  int tot_sats;                         /**< Total Number of Satellites in view, max. 12 */
  GSV_SAT_Info gsv_sat_i[MAX_SAT_NUM];  /**< Satellite info  */
  int checksum;                         /**< Checksum of the message bytes */
} GSV_Infos;

/**
 * @brief Data structure that contains timestamp
 */
typedef struct Timestamp_Info {
  int hh;  /**< Hours */
  int mm;  /**< Minutes */
  int ss;  /**< Seconds */
  int year;  /**< Year */
  int month;  /**< Month */
  int day;  /**< Day */
} Timestamp_Info;

/**
 * @brief Data structure that contains all of the information about Geofence 
 */
typedef struct Geofence_Infos {
  uint8_t op;          /**< Geofence type message (configuration/status) */
  uint8_t result;      /**< Geofence cfg/request result (OK/ERROR) */
  int idAlarm;         /**< Id of the circle raising the alarm */
  Timestamp_Info timestamp;
  int status[MAX_GEOFENCES_NUM];
} Geofence_Infos;

/**
 * @brief Data structure that contains all of the information about Ododmeter 
 */
typedef struct Odometer_Infos {
  uint8_t op;          /**< Odometer type message (configuration/status) */
  uint8_t result;      /**< Odometer cfg/request result (OK/ERROR) */
} Odometer_Infos;

/**
 * @brief Data structure that contains all of the information about Datalog 
 */
typedef struct Datalog_Infos {
  uint8_t op;          /**< Datalog type message (configuration/status) */
  uint8_t result;      /**< Datalog cfg/request result (OK/ERROR) */
} Datalog_Infos;

/**
 * @brief Ack from Teseo
 */
typedef uint8_t Ack_Info;

/**
 * @}
 */

/** @addtogroup NMEA_PARSER_PUBLIC_FUNCTIONS
 * @{
 */
ParseStatus_Typedef parse_gpgga  (GPGGA_Infos *gpgga_data, uint8_t *NMEA);
ParseStatus_Typedef parse_gnsmsg (GNS_Infos   *gns_data,   uint8_t *NMEA);
ParseStatus_Typedef parse_gpgst  (GPGST_Infos *gpgst_data, uint8_t *NMEA);
ParseStatus_Typedef parse_gprmc  (GPRMC_Infos *gprmc_data, uint8_t *NMEA);
ParseStatus_Typedef parse_gsamsg (GSA_Infos   *gsa_data,   uint8_t *NMEA);
ParseStatus_Typedef parse_gsvmsg (GSV_Infos   *gsv_data,   uint8_t *NMEA);
void                copy_data    (GPGGA_Infos *, GPGGA_Infos);

ParseStatus_Typedef parse_pstmgeofence(Geofence_Infos *geofence_data, uint8_t *NMEA);
ParseStatus_Typedef parse_pstmodo(Odometer_Infos *odo_data, uint8_t *NMEA);
ParseStatus_Typedef parse_pstmdatalog(Datalog_Infos *datalog_data, uint8_t *NMEA);
ParseStatus_Typedef parse_pstmsgl(Ack_Info *ack, uint8_t *NMEA);
ParseStatus_Typedef parse_pstmsavepar(Ack_Info *ack, uint8_t *NMEA);
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
 
#ifdef __cplusplus
}
#endif

#endif

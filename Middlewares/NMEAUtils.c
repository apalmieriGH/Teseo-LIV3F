/**
*******************************************************************************
* @file    NMEA_utils.c
* @author  AST / Central Lab
* @version V1.0.0
* @date    18-May-2017
* @brief   NMEA utilities
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

/** @defgroup NMEA_UTILS 
 * @{
 */

/** @addtogroup NMEA_UTILS_PUBLIC_FUNCTIONS
 * @{
 */
/**
 * @brief  Scans a string with UTC info and fills all fields of a
 *         UTC_Info struct
 * @param  utc_str       NMEA UTC string
 * @param  utc           The UTC_Info struct to fill 
 * @retval None
 */   
void scan_utc (char* utc_str, UTC_Info* utc) {
  
  sscanf(utc_str, "%d", &utc->utc);
  
  utc->hh = (utc->utc / 10000);
  utc->mm = (utc->utc - (utc->hh * 10000)) / 100;
  utc->ss = utc->utc - ((utc->hh * 10000) + (utc->mm * 100));
  
  return;
}

/**
 * @brief  Scans a bidimensional array with longitude and latitude infos and 
 *         fills the relative fields of a Coords struct
 * @param  xy_str       NMEA string with latitude and longitude infos
 * @param  str_offset   NMEA string with latitude and longitude infos
 * @param  utc          The Coords struct to fill 
 * @retval None
 */
void scan_xy (char* xy_str, uint8_t str_offset, Coords* xy) 
{  
  //printf("NMEA (xy_str): %s\n\r", xy_str);
  sscanf(xy_str,                "%lf", &xy->lat);
  sscanf(xy_str+str_offset,     "%c", &xy->ns);
  sscanf(xy_str+(2*str_offset), "%lf", &xy->lon);
  sscanf(xy_str+(3*str_offset), "%c", &xy->ew);
   
  return;  
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

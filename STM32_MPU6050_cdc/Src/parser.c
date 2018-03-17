
/* Taken from:
 *
 * GPS_FUNC.C Copyright 2005, WinSystems Inc. All Rights reserved
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <math.h>

#include "stm32f10x.h"

#include "ringbuffer.h"
#include "parser.h"

//#define DEBUG  1

/* GPS Global variables */
/* These are derived from the ZDA message */
float gps_utc_time;
int gps_day;
int gps_month;
int gps_year;

/* These are added by the GGA message */
float gps_latitude;
char gps_lat_reference;
float gps_longitude;
char gps_long_reference;
int gps_quality;
int gps_satellite_count;
float gps_hdop;
float gps_altitude;
char gps_altitude_unit;
float gps_separation;
char gps_separation_unit;
float gps_differential_age;
int gps_differential_station_id;

/* These are added by the GLL message */
char gps_status;
char gps_mode_indicator;

/* These are added by the GSA message */
char gps_op_mode;
char gps_fix_mode;
int gps_satellites_in_use[12];
float gps_pdop;
float gps_vdop;

/* These are added by the GSV message */
int gps_gsv_message_count;
int gps_gsv_message_number;
int gps_total_sats_in_view;
int gps_prn_number[3][4];
int gps_elevation[3][4];
int gps_azimuth[3][4];
int gps_snr[3][4];

/* These are added by the RMC message */
char gps_rmc_status;
float gps_sog;
float gps_track;
long gps_rmc_date;
float gps_mag_variation;
char gps_variation_direction;
char gps_sys_mode_indicator;

/* These are added by the VTG message */
float gps_track_magnetic;
float gps_sog_kilometers;


s_nmea_t g_nema = {0,};


/* Local function proto-types */
static char *my_token(char *source, char token);

char *field[50];
/* This function accepts a string believed to contain standard NMEA 0183 sentence
data and parses those fields and loads the appropriate global variables with the results.
 */
int parse_nmea(char *string)
{
   int field_count;
   int x,y;
   void *ptr;

   field_count = 0;

#ifdef DEBUG
   printf("Parsing NMEA string : <%s>\n", string);
#endif
   /* NMEA 0183 fields are delimited by commas. The my_token function returns
    * pointers to the fields.
    */

   g_nema.type = NMEA_INVALID;

   /* Get the first field pointer */
   field[0] = my_token(string, ',');
#ifdef DEBUG
   if(field[0])
      printf("Token = <%s>\n", field[0]);
#endif
   field_count++;
   while(1)
   {
      /* Contiue retrieving fields until there are no more (NULL) */
      field[field_count] = my_token(NULL, ',');
      if(field[field_count] == NULL)
         break;
#ifdef DEBUG
      printf("Token = <%s>\n", field[field_count]);
#endif
      field_count++;
   }
#ifdef DEBUG
   printf("%d fields parsed\n", field_count);
#endif
   /* If we got at least ONE field */
   if(field_count)
   {
      /* Check the first field for the valid NMEA 0183 headers */
      if(strcmp(field[0],"$GPGGA") == 0)
      {
         g_nema.type = NMEA_GGA;
         ptr = &(g_nema.un.gga);

         /* Retrieve the values from the remaining fields */
         ((nmea_GGA_t *)ptr)->utc_time = gps_utc_time = atof(field[1]);
         ((nmea_GGA_t *)ptr)->latitude = gps_latitude = atof(field[2]);
         ((nmea_GGA_t *)ptr)->lat_reference = gps_lat_reference = *(field[3]);
         ((nmea_GGA_t *)ptr)->longitude = gps_longitude = atof(field[4]);
         ((nmea_GGA_t *)ptr)->long_reference = gps_long_reference = *(field[5]);
         ((nmea_GGA_t *)ptr)->quality = gps_quality = atoi(field[6]);
         ((nmea_GGA_t *)ptr)->satellite_count = gps_satellite_count = atoi(field[7]);
         ((nmea_GGA_t *)ptr)->hdop = gps_hdop = atof(field[8]);
         ((nmea_GGA_t *)ptr)->altitude = gps_altitude = atof(field[9]);
         ((nmea_GGA_t *)ptr)->altitude_unit = gps_altitude_unit = *(field[10]);
         ((nmea_GGA_t *)ptr)->separation = gps_separation = atof(field[11]);
         ((nmea_GGA_t *)ptr)->separation_unit = gps_separation_unit = *(field[12]);
         ((nmea_GGA_t *)ptr)->differential_age = gps_differential_age = atof(field[13]);
         ((nmea_GGA_t *)ptr)->differential_station_id = gps_differential_station_id = atoi(field[14]);
#ifdef DEBUG
         printf("NMEA string GPGGA recognized\n");
         printf("Time = %9.2f\n", gps_utc_time);
         printf("Position : %8.3f %c %8.3f %c\n", gps_latitude,gps_lat_reference,
               gps_longitude,gps_long_reference);
         printf("GPS quality = %d, Satelite count = %d, HDOP = %4.2f\n", gps_quality,
               gps_satellite_count, gps_hdop);
         printf("GPS altitude = %9.2f %c, Geoidal Separation = %9.2f %c\n", gps_altitude,
               gps_altitude_unit, gps_separation, gps_separation_unit);
         printf("GPS differential update age = %9.2f.Station ID = %d\n", gps_differential_age,
               gps_differential_station_id);
#endif
      }
      if(strcmp(field[0], "$GPGLL") == 0)
      {
         g_nema.type = NMEA_GLL;
         ptr = &(g_nema.un.gll);

         /* Retrieve the values from the remaining fields */
         ((nmea_GLL_t *)ptr)->latitude = gps_latitude = atof(field[1]);
         ((nmea_GLL_t *)ptr)->lat_reference = gps_lat_reference = *(field[2]);
         ((nmea_GLL_t *)ptr)->longitude = gps_longitude = atof(field[3]);
         ((nmea_GLL_t *)ptr)->long_reference = gps_long_reference = *(field[4]);
         ((nmea_GLL_t *)ptr)->utc_time = gps_utc_time = atof(field[5]);
         ((nmea_GLL_t *)ptr)->status = gps_status = *(field[6]);
         ((nmea_GLL_t *)ptr)->mode_indicator = gps_mode_indicator = *(field[7]);
#ifdef DEBUG
         printf("NMEA string GPGLL recognized\n");
         printf("Position : %8.3f %c %8.3f %c\n", gps_latitude,gps_lat_reference,
               gps_longitude,gps_long_reference);
         printf("Time = %9.2f\n", gps_utc_time);
         printf("GPS status = %c, GPS mode indicator = %c\n", gps_status, gps_mode_indicator);
#endif
      }
      if(strcmp(field[0],"$GPGSA") == 0)
      {
         g_nema.type = NMEA_GSA;
         ptr = &(g_nema.un.gsa);

         /* Retrieve the values from the remaining fields */
         ((nmea_GSA_t *)ptr)->op_mode = gps_op_mode = *(field[1]);
         ((nmea_GSA_t *)ptr)->fix_mode = gps_fix_mode = *(field[2]);
         ((nmea_GSA_t *)ptr)->pdop = gps_pdop = atof(field[15]);
         ((nmea_GSA_t *)ptr)->hdop = gps_hdop = atof(field[16]);
         ((nmea_GSA_t *)ptr)->vdop = gps_vdop = atof(field[17]);

#ifdef DEBUG
         printf("NMEA string GPGSA recognized\n");
         printf("Operation mode = %c, Fix mode = %c\n", gps_op_mode, gps_fix_mode);
         printf("Satelites in use : ");
#endif
         for(x=0; x<12; x++)
         {
            ((nmea_GSA_t *)ptr)->satellites_in_use[x] = gps_satellites_in_use[x] = atoi(field[x+3]);
#ifdef DEBUG
            if(gps_satellites_in_use[x])
               printf("%d ", gps_satellites_in_use[x]);
#endif
         }
#ifdef DEBUG
         printf("\n");
         printf("GPS precision %5.2f PDOP, %5.2f HDOP, %5.2f VDOP\n",
               gps_pdop, gps_hdop, gps_vdop);
#endif
      }
      if(strcmp(field[0],"$GPGSV") == 0)
      {
         g_nema.type = NMEA_GSV;
         ptr = &(g_nema.un.gsv);

         /* Retrieve the data from the remaining fields */
         ((nmea_GSV_t *)ptr)->gsv_message_count = gps_gsv_message_count = atoi(field[1]);
         ((nmea_GSV_t *)ptr)->gsv_message_number = gps_gsv_message_number = atoi(field[2]);
         ((nmea_GSV_t *)ptr)->total_sats_in_view = gps_total_sats_in_view = atoi(field[3]);
#ifdef DEBUG
         printf("NMEA string GPGSV recognized\n");
         printf("Total satelites in view = %d\n", gps_total_sats_in_view);
#endif
         if((gps_gsv_message_number > 0) && (gps_gsv_message_number < 4))
         {
            y = gps_gsv_message_number - 1;
            for(x=0; x< 4; x++)
            {
               ((nmea_GSV_t *)ptr)->prn_number[y][x] = gps_prn_number[y][x] = atoi(field[(x*4)+4]);
               ((nmea_GSV_t *)ptr)->elevation[y][x] = gps_elevation[y][x] = atoi(field[(x*4)+5]);
               ((nmea_GSV_t *)ptr)->azimuth[y][x] = gps_azimuth[y][x] = atoi(field[(x*4)+6]);
               ((nmea_GSV_t *)ptr)->snr[y][x] = gps_snr[y][x] = atoi(field[(x*4)+7]);
#ifdef DEBUG
               printf("Satelite %d - Elev = %d Azim = %d SNR = %d\n",
                     gps_prn_number[y][x], gps_elevation[y][x], gps_azimuth[y][x],
                     gps_snr[y][x]);
#endif
            }
         }
      }
      if(strcmp(field[0],"$GPRMC") == 0)
      {
         g_nema.type = NMEA_RMC;
         ptr = &(g_nema.un.rmc);

         /* Retrieve the data from the remaining fields */
         ((nmea_RMC_t *)ptr)->utc_time = gps_utc_time = atof(field[1]);
         ((nmea_RMC_t *)ptr)->rmc_status = gps_rmc_status = *(field[2]);
         ((nmea_RMC_t *)ptr)->latitude = gps_latitude = atof(field[3]);
         ((nmea_RMC_t *)ptr)->lat_reference = gps_lat_reference = *(field[4]);
         ((nmea_RMC_t *)ptr)->longitude = gps_longitude = atof(field[5]);
         ((nmea_RMC_t *)ptr)->long_reference = gps_long_reference = *(field[6]);
         ((nmea_RMC_t *)ptr)->sog = gps_sog = atof(field[7]);
         ((nmea_RMC_t *)ptr)->track = gps_track = atof(field[8]);
         ((nmea_RMC_t *)ptr)->rmc_date = gps_rmc_date = atol(field[9]);
         ((nmea_RMC_t *)ptr)->mag_variation = gps_mag_variation = atof(field[10]);
         ((nmea_RMC_t *)ptr)->variation_direction = gps_variation_direction = *(field[11]);
         ((nmea_RMC_t *)ptr)->sys_mode_indicator = gps_sys_mode_indicator = *(field[12]);
#ifdef DEBUG
         printf("NMEA string GPRMC recognized\n");
         printf("GPS UTC Time = %9.2f. RMC Status = %c\n", gps_utc_time, gps_rmc_status);
         printf("Position : %7.2f Deg %c %7.2f Deg %c\n", gps_latitude, gps_lat_reference,
               gps_longitude, gps_long_reference);
         printf("GPS Track %7.2f degrees at %7.2f Knot groundspeed\n", gps_track, gps_sog);
         printf("GPS Date : %8ld Mag Variation %6.2f Deg %c GPS Mode %c\n",
               gps_rmc_date, gps_mag_variation, gps_variation_direction, gps_mode_indicator);
#endif
      }
      if(strcmp(field[0],"$GPVTG") == 0)
      {
         g_nema.type = NMEA_VTG;
         ptr = &(g_nema.un.vtg);

         /* Retrieve the data from the remaining fields */
         ((nmea_VTG_t *)ptr)->track = gps_track = atof(field[1]);
         ((nmea_VTG_t *)ptr)->track_magnetic = gps_track_magnetic = atof(field[3]);
         ((nmea_VTG_t *)ptr)->sog = gps_sog = atof(field[5]);
         ((nmea_VTG_t *)ptr)->sog_kilometers = gps_sog_kilometers = atof(field[7]);
         ((nmea_VTG_t *)ptr)->sys_mode_indicator = gps_sys_mode_indicator = *(field[9]);
#ifdef DEBUG
         printf("NMEA string GPVTG recognized\n");
         printf("GPS Track : %7.2f True %7.2f Magnetic. Speed : %9.2f Knots %9.2f Kilometer/Hour\n",
               gps_track, gps_track_magnetic, gps_sog, gps_sog_kilometers);
#endif
      }
      if(strcmp(field[0],"$GPZDA") == 0)
      {
         g_nema.type = NMEA_ZDA;
         ptr = &(g_nema.un.zda);

         /* Retrieve the data from the remaining fields */
         ((nmea_ZDA_t *)ptr)->utc_time = gps_utc_time = atof(field[1]);
         ((nmea_ZDA_t *)ptr)->day = gps_day = atoi(field[2]);
         ((nmea_ZDA_t *)ptr)->month = gps_month = atoi(field[3]);
         ((nmea_ZDA_t *)ptr)->year = gps_year = atoi(field[4]);
#ifdef DEBUG
         printf("NMEA string GPZDA recognized\n");
         printf("%9.2f %2d/%2d/%4d\n", gps_utc_time, gps_month, gps_day, gps_year);
#endif
      }
   }
   return field_count;
}


/* These variables and the function my_token are used to retrieve the comma
delimited field pointers from the input string. Repeated calls to
my_token return the next field until there are no more (NULL).
 */
static char stat_string[128];
static char *current = NULL;
static char *my_token(char *source, char token)
{
   char *start;
   /* The source string is real only for the first call. Subsequent calls
    * are made with the source string pointer as NULL
    */
   if(source != NULL)
   {
      /* If the string is empty return NULL */
      if(strlen(source) == 0)
         return NULL;
      strcpy(stat_string,source);
      /* Current is our ‘current’ position within the string */
      current = stat_string;
   }
   start = current;
   while(1)
   {
      /* If we’re at the end of the string, return NULL */
      if((*current == '\0') && (current == start))
         return NULL;
      /* If we’re at the end now, but weren’t when we started, we need
       * to return the pointer for the last field before the end of string
       */
      if(*current == '\0')
         return start;
      /* If we’ve located our specified token (comma) in the string
       * load its location in the copy with an end of string marker
       * so that it can be handled correctly by the calling program.
       */
      if(*current == token)
      {
         *current = '\0';
         current++;
         return start;
      }
      else
      {
         current++;
      }
   }
}


// returns -1 on error, 0 if ok.
// string should not have the CR-LF (replaced by NULL).
// checksum is xor of all bytes between '$' and '*'.
int nmea_checksum(char *string)
{
   int idx = 0;
   u8 tmpsum = 0;
   u8 strsum;
   u32 len;

   if (string[0] != '$')
      return -1;

   idx = 1;
   len = strlen(string);
   while(idx < len)
   {
      if (string[idx] == '*')
         break;
      tmpsum ^= string[idx++];
   }

   if (string[idx] == '*')
   {
      strsum = 0;
      if (string[idx+1] && string[idx+2] && string[idx+3] == '\x0') // get checksum of the sentence..
      {
         strsum = strtol(&string[idx+1], NULL, 16);
         //printf("strsum=0x%x, tmpsum=0x%x\n", strsum, tmpsum);
         if (strsum == tmpsum) // checksum matches..
         {
            return 0;
         }
      }
      return -1;
   }
   else
   {
      return -1;
   }
}




void writeio(int fhnd, char *buf, int size)
{
   //printf("%s", buf);
}


void display_nmea_data(int fhnd)
{
   int x,y;
   void *ptr;
   char strbuf[256];

   switch(g_nema.type)
   {
      case NMEA_ZDA:
      {
         ptr = &(g_nema.un.zda);

         sprintf(strbuf, "NMEA string GPZDA recognized\n");
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "%9.2f %2d/%2d/%4d\n", ((nmea_ZDA_t *)ptr)->utc_time, ((nmea_ZDA_t *)ptr)->month, ((nmea_ZDA_t *)ptr)->day, ((nmea_ZDA_t *)ptr)->year);
         writeio(fhnd, strbuf, strlen(strbuf));
      }
      break;
      case NMEA_GGA:
      {
         ptr = &(g_nema.un.gga);

         sprintf(strbuf, "NMEA string GPGGA recognized\n");
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "Time = %9.2f\n", ((nmea_GGA_t *)ptr)->utc_time);
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "Position : %8.3f %c %8.3f %c\n", ((nmea_GGA_t *)ptr)->latitude, ((nmea_GGA_t *)ptr)->lat_reference,
               ((nmea_GGA_t *)ptr)->longitude, ((nmea_GGA_t *)ptr)->long_reference);
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "GPS quality = %d, Satelite count = %d, HDOP = %4.2f\n", ((nmea_GGA_t *)ptr)->quality,
               ((nmea_GGA_t *)ptr)->satellite_count, ((nmea_GGA_t *)ptr)->hdop);
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "GPS altitude = %9.2f %c, Geoidal Separation = %9.2f %c\n", ((nmea_GGA_t *)ptr)->altitude,
               ((nmea_GGA_t *)ptr)->altitude_unit, ((nmea_GGA_t *)ptr)->separation, ((nmea_GGA_t *)ptr)->separation_unit);
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "GPS differential update age = %9.2f.Station ID = %d\n", ((nmea_GGA_t *)ptr)->differential_age,
               ((nmea_GGA_t *)ptr)->differential_station_id);
         writeio(fhnd, strbuf, strlen(strbuf));
      }
      break;
      case NMEA_GLL:
      {
         ptr = &(g_nema.un.gll);

         sprintf(strbuf, "NMEA string GPGLL recognized\n");
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "Position : %8.3f %c %8.3f %c\n", ((nmea_GLL_t *)ptr)->latitude, ((nmea_GLL_t *)ptr)->lat_reference,
               ((nmea_GLL_t *)ptr)->longitude, ((nmea_GLL_t *)ptr)->long_reference);
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "Time = %9.2f\n", ((nmea_GLL_t *)ptr)->utc_time);
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "GPS status = %c, GPS mode indicator = %c\n", ((nmea_GLL_t *)ptr)->status, ((nmea_GLL_t *)ptr)->mode_indicator);
         writeio(fhnd, strbuf, strlen(strbuf));
      }
      break;
      case NMEA_GSA:
      {
         ptr = &(g_nema.un.gsa);

         sprintf(strbuf, "NMEA string GPGSA recognized\n");
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "Operation mode = %c, Fix mode = %c\n", ((nmea_GSA_t *)ptr)->op_mode, ((nmea_GSA_t *)ptr)->fix_mode);
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "Satelites in use : ");
         writeio(fhnd, strbuf, strlen(strbuf));

         for(x=0; x<12; x++)
         {
            if(((nmea_GSA_t *)ptr)->satellites_in_use[x])
            {
               sprintf(strbuf, "%d ", ((nmea_GSA_t *)ptr)->satellites_in_use[x]);
               writeio(fhnd, strbuf, strlen(strbuf));
            }
         }
         sprintf(strbuf, "\n");
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "GPS precision %5.2f PDOP, %5.2f HDOP, %5.2f VDOP\n",
               ((nmea_GSA_t *)ptr)->pdop, ((nmea_GSA_t *)ptr)->hdop, ((nmea_GSA_t *)ptr)->vdop);
         writeio(fhnd, strbuf, strlen(strbuf));
      }
      break;
      case NMEA_GSV:
      {
         ptr = &(g_nema.un.gsv);

         sprintf(strbuf, "NMEA string GPGSV recognized\n");
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "Total satelites in view = %d\n", ((nmea_GSV_t *)ptr)->total_sats_in_view);
         writeio(fhnd, strbuf, strlen(strbuf));
         if((((nmea_GSV_t *)ptr)->gsv_message_number > 0) && (((nmea_GSV_t *)ptr)->gsv_message_number < 4))
         {
            y = ((nmea_GSV_t *)ptr)->gsv_message_number - 1;
            for(x=0; x< 4; x++)
            {
               sprintf(strbuf, "Satelite %d - Elev = %d Azim = %d SNR = %d\n",
                     ((nmea_GSV_t *)ptr)->prn_number[y][x], ((nmea_GSV_t *)ptr)->elevation[y][x],
                     ((nmea_GSV_t *)ptr)->azimuth[y][x], ((nmea_GSV_t *)ptr)->snr[y][x]);
               writeio(fhnd, strbuf, strlen(strbuf));
            }
         }
      }
      break;
      case NMEA_RMC:
      {
         int k;
         ptr = &(g_nema.un.rmc);

         sprintf(strbuf, "NMEA string GPRMC recognized\n");
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "GPS UTC Time = %9.2f. RMC Status = %c\n", ((nmea_RMC_t *)ptr)->utc_time, ((nmea_RMC_t *)ptr)->rmc_status);
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "Position : %7.2f Deg %c %7.2f Deg %c\n", ((nmea_RMC_t *)ptr)->latitude, ((nmea_RMC_t *)ptr)->lat_reference,
               ((nmea_RMC_t *)ptr)->longitude, ((nmea_RMC_t *)ptr)->long_reference);
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "GPS Track %7.2f degrees at %7.2f Knot groundspeed\n", ((nmea_RMC_t *)ptr)->track, ((nmea_RMC_t *)ptr)->sog);
         writeio(fhnd, strbuf, strlen(strbuf));
         k = sprintf(strbuf, "GPS Date : %8ld Mag Variation %6.2f Deg %c GPS Mode %c\n",
               ((nmea_RMC_t *)ptr)->rmc_date, ((nmea_RMC_t *)ptr)->mag_variation, ((nmea_RMC_t *)ptr)->variation_direction, ((nmea_RMC_t *)ptr)->sys_mode_indicator);
         writeio(fhnd, strbuf, k);
      }
      break;
      case NMEA_VTG:
      {
         ptr = &(g_nema.un.vtg);

         sprintf(strbuf, "NMEA string GPVTG recognized\n");
         writeio(fhnd, strbuf, strlen(strbuf));
         sprintf(strbuf, "GPS Track : %7.2f True %7.2f Magnetic. Speed : %9.2f Knots %9.2f Kilometer/Hour\n",
               ((nmea_VTG_t *)ptr)->track, ((nmea_VTG_t *)ptr)->track_magnetic, ((nmea_VTG_t *)ptr)->sog, ((nmea_VTG_t *)ptr)->sog_kilometers);
         writeio(fhnd, strbuf, strlen(strbuf));
      }
      break;
      default:
      {

      }
      break;
   }
}





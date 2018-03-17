
#ifndef _PARSER_H
#define _PARSER_H


enum
{
   NMEA_INVALID = 0,
   NMEA_ZDA,
   NMEA_GGA,
   NMEA_GLL,
   NMEA_GSA,
   NMEA_GSV,
   NMEA_RMC,
   NMEA_VTG
};


/* These are derived from the ZDA message */
typedef struct
{
   float utc_time;
   int   day;
   int   month;
   int   year;
}
nmea_ZDA_t;


/* These are added by the GGA message */
typedef struct
{
   float utc_time;
   float latitude;
   char  lat_reference;
   float longitude;
   char  long_reference;
   int   quality;
   int   satellite_count;
   float hdop;
   float altitude;
   char  altitude_unit;
   float separation;
   char  separation_unit;
   float differential_age;
   int   differential_station_id;
}
nmea_GGA_t;


/* These are added by the GLL message */
typedef struct
{
   float utc_time;
   float latitude;
   char  lat_reference;
   float longitude;
   char  long_reference;
   char  status;
   char  mode_indicator;
}
nmea_GLL_t;


/* These are added by the GSA message */
typedef struct
{
   char  op_mode;
   char  fix_mode;
   int   satellites_in_use[12];
   float pdop;
   float hdop;
   float vdop;
}
nmea_GSA_t;


/* These are added by the GSV message */
typedef struct
{
   int   gsv_message_count;
   int   gsv_message_number;
   int   total_sats_in_view;
   int   prn_number[3][4];
   int   elevation[3][4];
   int   azimuth[3][4];
   int   snr[3][4];
}
nmea_GSV_t;


/* These are added by the RMC message */
typedef struct
{
   float utc_time;
   char  rmc_status;
   float latitude;
   char  lat_reference;
   float longitude;
   char  long_reference;
   float sog;
   float track;
   long  rmc_date;
   float mag_variation;
   char  variation_direction;
   char  sys_mode_indicator;
}
nmea_RMC_t;

/* These are added by the VTG message */
typedef struct
{
   float track;
   float track_magnetic;
   float sog;
   float sog_kilometers;
   char  sys_mode_indicator;
}
nmea_VTG_t;


typedef union
{
   nmea_ZDA_t  zda;
   nmea_GGA_t  gga;
   nmea_GLL_t  gll;
   nmea_GSA_t  gsa;
   nmea_GSV_t  gsv;
   nmea_RMC_t  rmc;
   nmea_VTG_t  vtg;
}
un_nmea_t;


/*
 * Structure to encompass all parsed data..
 */
typedef struct
{
   u32         type;
   un_nmea_t   un;
}
s_nmea_t;


int parse_nmea(char *string);
int nmea_checksum(char *string);
void display_nmea_data(int fhnd);

#endif // _PARSER_H

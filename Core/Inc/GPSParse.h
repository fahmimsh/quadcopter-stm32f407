/*
 * GPSParse.h
 *
 *  Created on: Jul 19, 2020
 *      Author: Cahyo Prastyawan
 */

#ifndef INC_GPSPARSE_H_
#define INC_GPSPARSE_H_

#include <string.h>

struct GPS_String {
	char* GNGGA;
	char *GNGGL;
	char *GNRMC;
};

struct GNGGA_DATA{
	char utc[9];
	float lat;
	char lat_dir;
	float lon;
	char lon_dir;
	char quality;
	uint8_t satellites_used;
	float HDOP;
	float altitude;
	float geoidal;
	char satellite_id[4];
	char checksum[2];
};

struct GNGLL_DATA {
	float lat;
	char lat_dir;
	float lon;
	char lon_dir;
	char utc[9];
	char status;
	char checksum[2];
};

struct GNRMC_DATA {
	char utc[9];
	char status;
	float lat;
	char lat_dir;
	float lon;
	char lon_dir;
	float speed;
	float course;
	char date[6];
	char mode;
	char checksum[2];
};

GGNA_DATA GGNA_PARSE(char *GNGGA_STRING);
GNRMC_DATA GNGLL_PARSE(char* GNGLL_STRING);
GNGLL_DATA GNRMC_PARSE(char* GNRMC_STRING);

#endif /* INC_GPSPARSE_H_ */

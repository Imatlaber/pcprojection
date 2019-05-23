/*---------------------------------------------------------------------------------------
		parse_gprmc.c
		Parse $GPRMC information from a pcap file
---------------------------------------------------------------------------------------*/

#define _CRT_SECURE_NO_WARNINGS	// Disable the CRT warnings for the MS Windows

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Source file name
#define SRC_FILENAME			"/home/ubuntu/Documents/20170103-2/2017-01-03-17-36-40_Velodyne-HDL-32-Data.pcap"

// Destination file name (.csv), use NULL to print on the console
#define DEST_FILENAME			"/home/ubuntu/Documents/20170103-2/2017-01-03-17-36-40_Velodyne-HDL-32-Data.csv"

// File buffer size
#define FILE_BUFFER_SIZE		(256 * 1024)

// Maximum $GPRMC line size		
#define MAX_GPRMC_LINE_SIZE		(256)

// The $GPRMC tag
#define	GPRMC_TAG				("$GPRMC")

// File prototypes
static void process_file_block(FILE* fp_out, unsigned char* file_buffer, size_t data_len);
static void parse_nema_gprmc_string(FILE* fp_out, char* gprmc_string);

// The main function
int main(void)
{
	FILE* fp_src;
	FILE* fp_dst;
	size_t bytes_read;
	size_t offset;
	unsigned char* file_buffer;

	// Allocate memory for the file buffer
	file_buffer = (unsigned char*) malloc(FILE_BUFFER_SIZE);
	if(file_buffer == NULL)
	{
		fprintf(stderr, "Error: Failed to allocate memory.\n");
		return -1;
	}

	// Open the source binary file
	if((fp_src = fopen(SRC_FILENAME, "rb")) == NULL)
	{
		fprintf(stderr, "Error: Failed to open source file %s.\n", SRC_FILENAME);
		free(file_buffer);
		return -2;
	}

	// Open the destination file
	if(DEST_FILENAME)
	{
		if((fp_dst = fopen(DEST_FILENAME, "w")) == NULL)
		{
			fprintf(stderr, "Error: Failed to open destination file %s.\n", DEST_FILENAME);
			fclose(fp_src);
			free(file_buffer);
			return -2;
		}
	}
	else
	{
		fp_dst = stdout;
	}

	// Print out the CSV header into the destination file
	fprintf(fp_dst, "Date (UTC),Time (UTC),Latitude,Longitude,GPS Locked,Speed (m/s),Heading, timestamp\n");

	// Read file into the buffer, block by block
	offset = 0;
	bytes_read = fread(file_buffer, 1, FILE_BUFFER_SIZE, fp_src);

	while(bytes_read > 0)
	{
		// Process the buffer
		process_file_block(fp_dst, file_buffer, bytes_read + offset);

		// Prepare the next block
		if(bytes_read > MAX_GPRMC_LINE_SIZE)
		{
			// Keep the last MAX_GPRMC_LINE_SIZE bytes in the buffer, to avoid splitting a single
			// $GPRMC line into two consquential reads.
			offset = MAX_GPRMC_LINE_SIZE;
			memcpy(file_buffer, file_buffer + bytes_read - offset, offset);
		}
		else
		{
			offset = 0;
		}

		// Read the next block into buffer
		bytes_read = fread(file_buffer + offset, 1, FILE_BUFFER_SIZE - offset, fp_src);
	} 

	// Close the file handles
	fclose(fp_src);
	if(DEST_FILENAME)
	{
		fclose(fp_dst);
	}

	// Free the file buffer
	free(file_buffer);

	return 0;
}

// Process a block of the binary file, and print out parsed data into the output file
static void process_file_block(FILE* fp_out, unsigned char* file_buffer, size_t data_len)
{
	unsigned char* pin;
	unsigned char* end;
	static char needle[] = GPRMC_TAG;
	size_t needle_len = strlen(needle);

	// pin: The current pointer to scan through the buffer
	// end: The end of data buffer
	end = file_buffer + data_len - needle_len;

	// Scan the whole buffer

	for (pin = file_buffer; pin != end; pin++)
	{
		unsigned char* pa = pin;
		unsigned char* pb = (unsigned char*) needle;

		while ((*pb) != 0 && (*pa) == (*pb))
		{
			pa++;
			pb++;
		}

		if((*pb) == 0)
		{
			// A $GPRMC tag is found at the pin
			
			// First we remove the leading $, to avoid re-scan the same tag when processing
			// the next block.
			*pin = 'x';

			// Then find the end of the $GPRMC string
			for (pa = pin + needle_len; pa != end; pa ++)
			{
				if (*pa == '\r')
				{
					// The end of the $GPRMC string is found. We terminate and parse the string now.
					*pa = 0;
					parse_nema_gprmc_string(fp_out, (char*) pin);
					break;
				}
			}
		}
	}
}

// Parse the NEMA $GPRMC string, and send the result to the CSV output file
static void parse_nema_gprmc_string(FILE* fp_out, char* gprmc_string)
{
	char latStr[32], lonStr[32];
	char dateStr[32];
	double Lat_v,Lon_v;
	char sn[2], ew[2], valid[2];
	int hh, mm, ss;
	int day, mon, year;
	int64_t mmss ;

	int Lat_deg, Lon_deg;
	double Lat_mi,Lon_mi;
	double Latd, Lond;
	double speed, heading;

	int i;

	// Change all commas to spaces
	for(i = 0; i < (int)strlen(gprmc_string); i++) {
		if (gprmc_string[i]==',') {
			gprmc_string[i]=' ';
		}
	}

	// Scan the string
	sscanf(gprmc_string, "%*s %s %s %s %s %s %s %lf %lf %02d %02d %02d",
		dateStr, valid, latStr, sn, lonStr, ew,
		&speed, &heading, &day, &mon, &year);
	sscanf(dateStr, "%02d %02d %02d", &hh, &mm, &ss);
	sscanf(latStr, "%lf", &Lat_v);
	sscanf(lonStr, "%lf", &Lon_v);
	speed *= 0.514444;
	year += 2000;
	Lat_deg = (int) Lat_v /100;
	Lon_deg = (int) Lon_v /100;
	Lat_mi = Lat_v - Lat_deg * 100;
	Lon_mi = Lon_v - Lon_deg * 100;
	Latd = Lat_deg + Lat_mi / 60;
	Lond = Lon_deg + Lon_mi / 60;

	if(sn == "S") Latd = -Latd;
	if(ew == "W") Lond = -Lond;

	mmss = 60*mm+ss ;	
	// Print out to the output CSV file
	//fprintf(fp_out, "%04d-%02d-%02d,%02d:%02d:%02d,%lf,%lf,%s,%lf,%lf,%ld\n",
	fprintf(fp_out, "%lf,%lf,%lf,%lf,%ld\n",
		//year, mon, day, hh, mm, ss,
		Latd, Lond,
		speed,
		heading,
		mmss);
}

#include  "main.h"
#include <string.h>
#include <stdlib.h>
extern UART_HandleTypeDef huart3;
extern RTC_HandleTypeDef hrtc;
GPS_Data_t gps;
uint8_t date_time_valid=0;

static double GPS_NMEA_ToDecimal(const char *value, char direction)
{
    if (value == NULL || value[0] == '\0')
        return 0.0;

    double v = atof(value);

    int degrees = (int)(v / 100.0);

    double minutes = v - ((double)degrees * 100.0);

    double result = (double)degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W')
        result = -result;

    return result;
}

static void GPS_ParseTime(const char *str)
{
    if (str == NULL || strlen(str) < 6)
        return;

    if (str[0] < '0' || str[0] > '9' ||
        str[1] < '0' || str[1] > '9')
        return;

    gps.hour =
        (uint8_t)((str[0] - '0') * 10 +
                  (str[1] - '0'));

    gps.minute =
        (uint8_t)((str[2] - '0') * 10 +
                  (str[3] - '0'));

    gps.second =
        (uint8_t)((str[4] - '0') * 10 +
                  (str[5] - '0'));

    gps.millisecond = 0;

    const char *dot = strchr(str, '.');

    if (dot != NULL)
    {
        int ms = atoi(dot + 1);

        if (ms < 10)
            gps.millisecond = ms * 100;
        else if (ms < 100)
            gps.millisecond = ms * 10;
        else
            gps.millisecond = ms;
    }
}

static void GPS_ParseDate(const char *str)
{
    if (str == NULL || strlen(str) < 6)
        return;

    gps.day =
        (uint8_t)((str[0] - '0') * 10 +
                  (str[1] - '0'));

    gps.month =
        (uint8_t)((str[2] - '0') * 10 +
                  (str[3] - '0'));

    gps.year =
        (uint8_t)((str[4] - '0') * 10 +
                  (str[5] - '0'));
}



uint8_t GPS_SetRTCFromGPS(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    if (!gps.valid)
        return -1;

    /*
     * GPS year is 00..99.
     * STM32 RTC expects 00..99.
     */
    sDate.Year = gps.year;
    sDate.Month = gps.month;
    sDate.Date = gps.day;

    sDate.WeekDay = RTC_WEEKDAY_MONDAY;   // temporary
    sTime.Hours = gps.hour;
    sTime.Minutes = gps.minute;
    sTime.Seconds = gps.second;

    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&hrtc,
                        &sTime,
                        RTC_FORMAT_BIN) != HAL_OK)
    {
        return -2;
    }

    if (HAL_RTC_SetDate(&hrtc,
                        &sDate,
                        RTC_FORMAT_BIN) != HAL_OK)
    {
        return -3;
    }
    date_time_valid=1;
    return 0;
}

void GPS_ParseRMC(char *sentence)
{
    char *field;
    int index = 0;

    char latitude[20] = {0};
    char longitude[20] = {0};

    char lat_dir = 0;
    char lon_dir = 0;

    field = strtok(sentence, ",");

    while (field != NULL)
    {
        switch (index)
        {
            case 1:
                /* UTC time */
                if (field[0] != '\0')
                    GPS_ParseTime(field);
                break;

            case 2:
                /* A = valid, V = invalid */
                gps.valid = (field[0] == 'A');
                break;

            case 3:
                /* Latitude */
                strncpy(latitude,
                        field,
                        sizeof(latitude) - 1);
                break;

            case 4:
                /* N/S */
                if (field[0] != '\0')
                    lat_dir = field[0];
                break;

            case 5:
                /* Longitude */
                strncpy(longitude,
                        field,
                        sizeof(longitude) - 1);
                break;

            case 6:
                /* E/W */
                if (field[0] != '\0')
                    lon_dir = field[0];
                break;

            case 7:
                /* Speed knots */
                if (field[0] != '\0')
                {
                    gps.speed_knots = atof(field);

                    gps.speed_kmh =
                        gps.speed_knots * 1.852f;
                }
                break;

            case 8:
                /* Course */
                if (field[0] != '\0')
                    gps.course = atof(field);
                break;

            case 9:
                /* Date */
                if (field[0] != '\0')
                    GPS_ParseDate(field);
                break;

            default:
                break;
        }

        field = strtok(NULL, ",");
        index++;
    }

    /* Convert position */
    if (latitude[0] != '\0' && lat_dir != 0)
    {
        gps.latitude =
            GPS_NMEA_ToDecimal(latitude, lat_dir);

        gps.latitude_dir = lat_dir;
    }

    if (longitude[0] != '\0' && lon_dir != 0)
    {
        gps.longitude =
            GPS_NMEA_ToDecimal(longitude, lon_dir);

        gps.longitude_dir = lon_dir;
    }
}


void GPS_ParseGGA(char *sentence)
{
    char *token = strtok(sentence, ",");
    int field_index = 0;

    while (token != NULL)
    {
        switch (field_index)
        {
            case 1: // UTC Time (hhmmss.ss)
                if (strlen(token) >= 6)
                {
                    char hh[3] = {token[0], token[1], '\0'};
                    char mm[3] = {token[2], token[3], '\0'};
                    char ss[3] = {token[4], token[5], '\0'};
                    gps.hour = (uint8_t)atoi(hh);
                    gps.minute = (uint8_t)atoi(mm);
                    gps.second = (uint8_t)atoi(ss);

                    char *dot = strchr(token, '.');
                    if (dot != NULL && strlen(dot) > 1)
                    {
                        char ms_str[4] = {0};
                        strncpy(ms_str, dot + 1, 3);
                        gps.millisecond = (uint16_t)(atoi(ms_str) * 10);
                    }
                }
                break;

            case 2: // Latitude (ddmm.mmmm)
                if (strlen(token) > 0)
                {
                    char deg_str[3] = {token[0], token[1], '\0'};
                    double minutes = atof(&token[2]);
                    gps.latitude = (double)atoi(deg_str) + (minutes / 60.0);
                }
                break;

            case 3: // Latitude Direction (N/S)
                if (strlen(token) > 0)
                {
                    gps.latitude_dir = token[0];
                }
                break;

            case 4: // Longitude (dddmm.mmmm)
                if (strlen(token) > 0)
                {
                    char deg_str[4] = {token[0], token[1], token[2], '\0'};
                    double minutes = atof(&token[3]);
                    gps.longitude = (double)atoi(deg_str) + (minutes / 60.0);
                }
                break;

            case 5: // Longitude Direction (E/W)
                if (strlen(token) > 0)
                {
                    gps.longitude_dir = token[0];
                }
                break;

            case 6: // Fix Quality (0 = invalid, 1 = GPS fix, 2 = DGPS, etc.)
                if (strlen(token) > 0)
                {
                    uint8_t quality = (uint8_t)atoi(token);
                    gps.valid = (quality > 0) ? 1 : 0;
                }
                break;

            case 7: // Number of Satellites in Use
                if (strlen(token) > 0)
                {
                    gps.satellites_used = (uint8_t)atoi(token);
                }
                break;

            default:
                break;
        }

        token = strtok(NULL, ",");
        field_index++;
    }
}


void GPS_ParseGLL(char *sentence)
{
    char *field;
    int index = 0;

    char latitude[20] = {0};
    char longitude[20] = {0};

    char lat_dir = 0;
    char lon_dir = 0;

    field = strtok(sentence, ",");

    while (field != NULL)
    {
        switch (index)
        {
            case 1:
                strncpy(latitude,
                        field,
                        sizeof(latitude) - 1);
                break;

            case 2:
                if (field[0] != '\0')
                    lat_dir = field[0];
                break;

            case 3:
                strncpy(longitude,
                        field,
                        sizeof(longitude) - 1);
                break;

            case 4:
                if (field[0] != '\0')
                    lon_dir = field[0];
                break;

            case 5:
                if (field[0] != '\0')
                    GPS_ParseTime(field);
                break;

            case 6:
                gps.valid = (field[0] == 'A');
                break;

            default:
                break;
        }

        field = strtok(NULL, ",");
        index++;
    }

    if (latitude[0] != '\0' && lat_dir != 0)
    {
        gps.latitude =
            GPS_NMEA_ToDecimal(latitude, lat_dir);

        gps.latitude_dir = lat_dir;
    }

    if (longitude[0] != '\0' && lon_dir != 0)
    {
        gps.longitude =
            GPS_NMEA_ToDecimal(longitude, lon_dir);

        gps.longitude_dir = lon_dir;
    }
}

void GPS_ParseGSA(char *sentence)
{
    char *field;
    int index = 0;

    uint8_t satellite_count = 0;

    field = strtok(sentence, ",");

    while (field != NULL)
    {
        switch (index)
        {
            case 2:
                /* Fix type */
                if (field[0] >= '1' &&
                    field[0] <= '3')
                {
                    gps.fix_type =
                        (uint8_t)(field[0] - '0');
                }
                else
                {
                    gps.fix_type = 1;
                }
                break;

            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:

                /* Satellite PRNs */
                if (field[0] != '\0')
                    satellite_count++;

                break;

            case 15:
                if (field[0] != '\0')
                    gps.pdop = atof(field);
                break;

            case 16:
                if (field[0] != '\0')
                    gps.hdop = atof(field);
                break;

            case 17:
            {
                char *star = strchr(field, '*');

                if (star != NULL)
                    *star = '\0';

                if (field[0] != '\0')
                    gps.vdop = atof(field);

                break;
            }

            default:
                break;
        }

        field = strtok(NULL, ",");
        index++;
    }

    gps.satellites_used = satellite_count;
}

void GPS_ParseVTG(char *sentence)
{
    char *field;
    int index = 0;

    field = strtok(sentence, ",");

    while (field != NULL)
    {
        switch (index)
        {
            case 1:
                /* Course */
                if (field[0] != '\0')
                    gps.course = atof(field);
                break;

            case 5:
                /* Speed knots */
                if (field[0] != '\0')
                    gps.speed_knots = atof(field);
                break;

            case 7:
                /* Speed km/h */
                if (field[0] != '\0')
                    gps.speed_kmh = atof(field);
                break;

            default:
                break;
        }

        field = strtok(NULL, ",");
        index++;
    }
}

void GPS_ParseGSV(char *sentence)
{
    char *field;
    int index = 0;

    field = strtok(sentence, ",");

    while (field != NULL)
    {
        switch (index)
        {
            case 1:
                gps.gsv_total_messages =
                    (uint8_t)atoi(field);
                break;

            case 2:
                gps.gsv_message_number =
                    (uint8_t)atoi(field);
                break;

            case 3:
                gps.satellites_in_view = (uint8_t)atoi(field);
                break;

            default:
                break;
        }

        field = strtok(NULL, ",");
        index++;
    }
}

void GPS_HandleSentence(char *sentence)
{
    if (sentence == NULL)
        return;

    if (strncmp(sentence, "$GNRMC", 6) == 0 ||
        strncmp(sentence, "$GPRMC", 6) == 0)
    {
        GPS_ParseRMC(sentence);
        if(date_time_valid==0)
        	GPS_SetRTCFromGPS();
    }
    else if (strncmp(sentence, "$GNGLL", 6) == 0 ||
             strncmp(sentence, "$GPGLL", 6) == 0)
    {
        GPS_ParseGLL(sentence);
    }
    else if (strncmp(sentence, "$GNGSA", 6) == 0 ||
             strncmp(sentence, "$GPGSA", 6) == 0)
    {
        GPS_ParseGSA(sentence);
    }
    else if (strncmp(sentence, "$GNVTG", 6) == 0 ||
             strncmp(sentence, "$GPVTG", 6) == 0)
    {
        GPS_ParseVTG(sentence);
    }
    else if (strncmp(sentence, "$GNGSV", 6) == 0 ||
             strncmp(sentence, "$GPGSV", 6) == 0)
    {
        GPS_ParseGSV(sentence);
    }
    else if (strncmp(sentence, "$GNGGA", 6) == 0 ||
             strncmp(sentence, "$GPGGA", 6) == 0)
    {
        GPS_ParseGGA(sentence);
    }
}

void process_gps()
{

	for(uint16_t i=0;i<gps_line_size;i++)
	{
		if(gps_buff[i]=='$')//find start GPS_line;
		{
			for(uint16_t j=i;j<gps_line_size;j++)
			{
				if(gps_buff[j]=='\n')//find end gps line;
				{
					uint16_t len = j - i;
                    if (len >= 2 && len < 128)
                    {
                    	char sent[128];
                    	memcpy(sent,gps_buff+i,len);
                    	sent[len]=0;
                    	GPS_HandleSentence(sent);

                    }
                    i=len;
                    break;
				}
			}
		}
	}
}

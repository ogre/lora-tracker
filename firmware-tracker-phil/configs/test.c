#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>

#include "../radio.h"
#include "config.h"
#include "gnss_aid.h"

int main(void)
{
    #ifdef GNSS_AID_AUXILIARY
        uint64_t auxiliary_age = time(NULL) - GNSS_AID_TIMESTAMP;
        printf("GNSS Ionospheric Aid Enabled (");
        if(auxiliary_age > 3600)
        {
            printf("%.1f hours", (double)auxiliary_age / 3600.0);
        }
        else if(auxiliary_age > 60)
        {
            printf("%.1f minutes", (double)auxiliary_age / 60.0);
        }
        else
        {
            printf("<1 minute");
        }
        printf(" old)\n");
    #endif

    #ifdef GNSS_AID_ALMANAC
        uint64_t almanac_age = time(NULL) - GNSS_AID_TIMESTAMP;
        printf("GNSS Almanac Aid Enabled (");
        if(almanac_age > 3600)
        {
            printf("%.1f hours", (double)almanac_age / 3600.0);
        }
        else if(almanac_age > 60)
        {
            printf("%.1f minutes", (double)almanac_age / 60.0);
        }
        else
        {
            printf("<1 minute");
        }
        printf(" old)\n");
    #endif

    #ifdef GNSS_AID_POSITION
        double gnss_aid_position_latitude_degrees = (double)gnss_aid_position_latitude / 1e7;
        double gnss_aid_position_longitude_degrees = (double)gnss_aid_position_longitude / 1e7;
        double gnss_aid_position_altitude_km = (double)gnss_aid_position_altitude / 1e5;
        double gnss_aid_position_stddev_km = (double)gnss_aid_position_stddev / 1e5;
        printf("GNSS Position Aid Enabled (%+.4f°, %+.4f°, %.1fkm, +-%.1fkm)\n",
            gnss_aid_position_latitude_degrees,
            gnss_aid_position_longitude_degrees,
            gnss_aid_position_altitude_km,
            gnss_aid_position_stddev_km);
        printf(" - https://www.google.co.uk/maps/search/?api=1&query=%.5f,%.5f\n",
            gnss_aid_position_latitude_degrees,
            gnss_aid_position_longitude_degrees
            );
    #endif

    return 1;
}
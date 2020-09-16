#ifndef GEO_COORD_H
#define GEO_COORD_H

struct GeoCoord
{
    double altitude, latitude, longitude;

    GeoCoord(): altitude(0.0), latitude(0.0), longitude(0.0) {};
};

namespace GPSInfo
{
}


#endif
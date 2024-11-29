// Location.h
#ifndef LOCATION_H
#define LOCATION_H

#include <string>

struct Location {
    std::string ubigeo;
    std::string department;
    std::string province;
    double latitude;
    double longitude;
    std::string naturalRegion;
    int warehouseCapacity;

    Location() {}
    Location(std::string u, std::string dep, std::string prov, double lat, double lon, std::string region, int capacity)
        : ubigeo(u), department(dep), province(prov), latitude(lat), longitude(lon), naturalRegion(region), warehouseCapacity(capacity) {}
};

#endif // LOCATION_H

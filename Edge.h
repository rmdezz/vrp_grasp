// Edge.h
#ifndef EDGE_H
#define EDGE_H

#include <string>

struct Edge {
    std::string originUbigeo;
    std::string destinationUbigeo;
    double distance;      // En kilómetros
    long long travelTime; // En minutos

    Edge() {}
    Edge(std::string origin, std::string destination, double dist, long long time)
        : originUbigeo(origin), destinationUbigeo(destination), distance(dist), travelTime(time) {}
};

#endif // EDGE_H

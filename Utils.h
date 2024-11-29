// Utils.h
#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <unordered_map>
#include <vector>
#include "Location.h"
#include "Edge.h"

// Funciones auxiliares
double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double getAverageSpeed(const std::string& region1, const std::string& region2);
std::unordered_map<std::string, Location> loadLocations(const std::string& filePath);
std::vector<Edge> loadEdges(const std::string& filePath, const std::unordered_map<std::string, Location>& locations);
int getLocationIndex(const std::vector<std::string>& ubigeos, const std::string& ubigeo);
std::vector<std::vector<long long>> createTimeMatrix(const std::vector<std::string>& ubigeos, const std::vector<Edge>& edges);
bool fileExists(const std::string& filePath);
std::string formatDuration(long long total_minutes);

#endif // UTILS_H

// Utils.cpp
#include "Utils.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>

// Constante para representar un valor infinito
const long long INF = std::numeric_limits<long long>::max();

// Implementación de las funciones auxiliares

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Radio de la Tierra en kilómetros
    const double R = 6371.0;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a =
        sin(dLat / 2) * sin(dLat / 2) +
        cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
        sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = R * c; // Distancia en kilómetros
    return distance;
}

double getAverageSpeed(const std::string& region1, const std::string& region2) {
    // Definir las velocidades promedio
    if (region1 == "COSTA" && region2 == "COSTA") {
        return 70.0;
    }
    else if ((region1 == "COSTA" && region2 == "SIERRA") ||
             (region1 == "SIERRA" && region2 == "COSTA")) {
        return 50.0;
    }
    else if (region1 == "SIERRA" && region2 == "SIERRA") {
        return 60.0;
    }
    else if ((region1 == "SIERRA" && region2 == "SELVA") ||
             (region1 == "SELVA" && region2 == "SIERRA")) {
        return 55.0;
    }
    else if (region1 == "SELVA" && region2 == "SELVA") {
        return 65.0;
    }
    else {
        // Valor por defecto
        return 60.0;
    }
}

bool fileExists(const std::string& filePath) {
    std::ifstream file(filePath);
    return file.good();
}

std::unordered_map<std::string, Location> loadLocations(const std::string& filePath) {
    std::unordered_map<std::string, Location> locations;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error al abrir el archivo de ubicaciones: " << filePath << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string line;
    while (getline(file, line)) {
        // Ignorar líneas vacías o comentarios
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string ubigeo, department, province, latStr, lonStr, naturalRegion, capacityStr;

        // Obtener los campos separados por comas
        if (!getline(ss, ubigeo, ',')) continue;
        if (!getline(ss, department, ',')) continue;
        if (!getline(ss, province, ',')) continue;
        if (!getline(ss, latStr, ',')) continue;
        if (!getline(ss, lonStr, ',')) continue;
        if (!getline(ss, naturalRegion, ',')) continue;
        if (!getline(ss, capacityStr, ',')) continue;

        // Convertir cadenas a los tipos adecuados
        double latitude = std::stod(latStr);
        double longitude = std::stod(lonStr);
        int capacity = std::stoi(capacityStr);

        // Crear y almacenar la ubicación
        Location loc(ubigeo, department, province, latitude, longitude, naturalRegion, capacity);
        locations[ubigeo] = loc;
    }

    file.close();
    return locations;
}

std::vector<Edge> loadEdges(const std::string& filePath, const std::unordered_map<std::string, Location>& locations) {
    std::vector<Edge> edges;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error al abrir el archivo de tramos: " << filePath << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string line;
    while (getline(file, line)) {
        // Ignorar líneas vacías o comentarios
        if (line.empty() || line[0] == '#') continue;

        // Separar por '=>'
        size_t pos = line.find("=>");
        if (pos == std::string::npos) continue;

        std::string originUbigeo = line.substr(0, pos);
        std::string destinationUbigeo = line.substr(pos + 2);

        // Eliminar posibles espacios
        originUbigeo.erase(remove_if(originUbigeo.begin(), originUbigeo.end(), ::isspace), originUbigeo.end());
        destinationUbigeo.erase(remove_if(destinationUbigeo.begin(), destinationUbigeo.end(), ::isspace), destinationUbigeo.end());

        // Verificar que las ubicaciones existan
        auto originIt = locations.find(originUbigeo);
        auto destIt = locations.find(destinationUbigeo);
        if (originIt == locations.end() || destIt == locations.end()) {
            std::cerr << "¡Advertencia! No se pudo encontrar la ubicación para " << originUbigeo 
                      << " o " << destinationUbigeo << std::endl;
            continue;
        }

        // Calcular distancia y tiempo de viaje
        double distance = calculateDistance(originIt->second.latitude, originIt->second.longitude,
                                           destIt->second.latitude, destIt->second.longitude);
        double speed = getAverageSpeed(originIt->second.naturalRegion, destIt->second.naturalRegion);
        double travelTime = (distance / speed) * 60.0; // Convertir horas a minutos

        // Crear y almacenar el tramo
        Edge edge(originUbigeo, destinationUbigeo, distance, static_cast<long long>(round(travelTime)));
        edges.push_back(edge);
    }

    file.close();
    return edges;
}

int getLocationIndex(const std::vector<std::string>& ubigeos, const std::string& ubigeo) {
    for (size_t i = 0; i < ubigeos.size(); ++i) {
        if (ubigeos[i] == ubigeo) return i;
    }
    return -1;
}

std::vector<std::vector<long long>> createTimeMatrix(const std::vector<std::string>& ubigeos, const std::vector<Edge>& edges) {
    int n = ubigeos.size();
    std::vector<std::vector<long long>> timeMatrix(n, std::vector<long long>(n, INF));

    // Inicializar la diagonal a 0 (tiempo a sí mismo es 0)
    for (int i = 0; i < n; ++i) {
        timeMatrix[i][i] = 0;
    }

    // Llenar la matriz con los tiempos de los tramos conocidos
    for (const auto& edge : edges) {
        int from = getLocationIndex(ubigeos, edge.originUbigeo);
        int to = getLocationIndex(ubigeos, edge.destinationUbigeo);
        if (from != -1 && to != -1) {
            timeMatrix[from][to] = edge.travelTime;
            // Asumiendo que los tramos son bidireccionales
            timeMatrix[to][from] = edge.travelTime;
        }
    }

    return timeMatrix;
}

std::string formatDuration(long long total_minutes) {
    long long hours = total_minutes / 60;
    long long minutes = total_minutes % 60;
    std::string result = std::to_string(hours) + " horas " + std::to_string(minutes) + " minutos";
    return result;
}
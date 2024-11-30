// main.cpp
#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <unordered_map>
#include <random>

#include "Location.h"
#include "Edge.h"
#include "Solution.h"
#include "Utils.h"
#include "DataModel.h"
#include "Routes.h"

int main() {
    // Nombres de los archivos
    std::string locations_file = "locations.txt";
    std::string edges_file = "edges.txt";

    // Verificar si los archivos existen
    if (!fileExists(locations_file)) {
        std::cerr << "Error: No se encontró el archivo " << locations_file << " en el directorio actual.\n";
        return EXIT_FAILURE;
    }
    
    if (!fileExists(edges_file)) {
        std::cerr << "Error: No se encontró el archivo " << edges_file << " en el directorio actual.\n";
        return EXIT_FAILURE;
    }

    // Cargar ubicaciones
    std::unordered_map<std::string, Location> locations_map = loadLocations(locations_file);

    // Obtener un vector de ubigeos para indexación
    std::vector<std::string> ubigeos;
    for (const auto& pair : locations_map) {
        ubigeos.push_back(pair.first);
    }

    // Crear un mapa de ubigeo a índice
    std::unordered_map<std::string, int> ubigeo_to_index;
    for (size_t i = 0; i < ubigeos.size(); ++i) {
        ubigeo_to_index[ubigeos[i]] = i;
    }

    // Cargar tramos
    std::vector<Edge> edges = loadEdges(edges_file, locations_map);

    // Crear la matriz de tiempos
    std::vector<std::vector<long long>> timeMatrix = createTimeMatrix(ubigeos, edges);

    // Definir puntos de inicio y fin para cada vehículo
    std::vector<std::string> vehicle_starts = { "150101", "130101", "040101"};
    std::vector<std::string> vehicle_ends = { "090501", "010301", "250201"};

    // Convertir ubigeos a índices
   std::vector<int> starts, ends;
    for (size_t i = 0; i < vehicle_starts.size(); ++i) {
        if (ubigeo_to_index.find(vehicle_starts[i]) != ubigeo_to_index.end()) {
            starts.push_back(ubigeo_to_index[vehicle_starts[i]]);
        } else {
            std::cerr << "Error: Punto de inicio " << vehicle_starts[i] << " no encontrado.\n";
            return EXIT_FAILURE;
        }

        if (ubigeo_to_index.find(vehicle_ends[i]) != ubigeo_to_index.end()) {
            ends.push_back(ubigeo_to_index[vehicle_ends[i]]);
        } else {
            std::cerr << "Error: Punto final " << vehicle_ends[i] << " no encontrado.\n";
            return EXIT_FAILURE;
        }
    }

    // Crear el modelo de datos
    DataModel data;
    data.timeMatrix = timeMatrix;
    data.ubigeos = ubigeos;
    data.starts = starts;
    data.ends = ends;
    data.exclusion_penalty = 100; // Penalización por nodo excluido
    
    // Parámetro alpha
    double alpha = 0.7; // valor entre 0.0 y 1.0

    // Inicializar generador de números aleatorios
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 rng(seed);

    // Parámetro de tiempo máximo en segundos (puedes ajustarlo aquí o pasar como parámetro)
    int max_time_seconds = 10; // Tiempo máximo de búsqueda en segundos
    int max_time_ms = max_time_seconds * 1000; // Convertir a milisegundos

    Solution best_solution;
    best_solution.makespan = std::numeric_limits<long long>::max();

    // Inicio del temporizador
    auto start_time = std::chrono::high_resolution_clock::now();
    auto end_time = start_time + std::chrono::milliseconds(max_time_ms);

    // Fase de Construcción y Mejora
    while (std::chrono::high_resolution_clock::now() < end_time) {
        // Construir una solución inicial
        Solution current_solution = constructGreedyRandomizedSolution(data, rng, alpha);

        // Aplicar búsqueda local hasta que no haya mejoras
        bool improved = true;
        while (improved) {
            improved = localSearch(current_solution, data, rng);
        }

        // Actualizar la mejor solución encontrada
        if (current_solution.makespan < best_solution.makespan) {
            best_solution = current_solution;
        }
    }

    // Fin del temporizador
    auto elapsed_time = std::chrono::high_resolution_clock::now() - start_time;

    // Verificar si se encontró una solución válida
    if (best_solution.routes.empty()) {
        std::cout << "No se encontró una solución válida.\n";
        return EXIT_FAILURE;
    }

    // Imprimir la mejor solución
    printSolution(best_solution, ubigeos, timeMatrix, locations_map, data.exclusion_penalty);

    // Imprimir tiempo de ejecución
    std::cout << "Tiempo total de ejecución: " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() 
              << " ms\n";

    return 0;
}

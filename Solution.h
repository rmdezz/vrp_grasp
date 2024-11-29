#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include <unordered_set>

struct Solution {
    std::vector<std::vector<int>> routes; // Rutas de cada vehículo
    std::vector<long long> total_durations; // Duración de cada ruta
    long long makespan; // Tiempo máximo entre todas las rutas
    std::unordered_set<int> excluded_nodes; // Nodos excluidos opcionalmente
    long long total_penalty; // Penalización total por exclusiones

    Solution() : makespan(0), total_penalty(0) {}
};

#endif // SOLUTION_H

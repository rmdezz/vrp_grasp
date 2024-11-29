// Routes.cpp
#include "Routes.h"
#include "DataModel.h"
#include "Utils.h"
#include <algorithm>
#include <iostream>
#include <queue>

// Constante para representar un valor infinito
const long long INF = std::numeric_limits<long long>::max();

// Funciones auxiliares para incluir y excluir nodos
void excludeNode(Solution& solution, int node, const DataModel& data) {
    solution.excluded_nodes.insert(node);
    solution.total_penalty += data.exclusion_penalty;
    // No es necesario actualizar el makespan aquí
}

void includeNode(Solution& solution, int node, const DataModel& data) {
    solution.excluded_nodes.erase(node);
    solution.total_penalty -= data.exclusion_penalty;
    // No es necesario actualizar el makespan aquí
}

// Algoritmo de Dijkstra para encontrar el camino más corto entre dos nodos
std::vector<int> dijkstra(int start_node, int end_node, const DataModel& data) {
    size_t n = data.ubigeos.size();
    std::vector<long long> distances(n, INF);
    std::vector<int> previous(n, -1);
    distances[start_node] = 0;

    using NodePair = std::pair<long long, int>;
    std::priority_queue<NodePair, std::vector<NodePair>, std::greater<NodePair>> pq;
    pq.emplace(0, start_node);

    while (!pq.empty()) {
        long long dist = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (u == end_node) break;

        if (dist > distances[u]) continue;

        for (size_t v = 0; v < n; ++v) {
            long long edge_cost = data.timeMatrix[u][v];
            if (edge_cost != INF) {
                if (distances[u] + edge_cost < distances[v]) {
                    distances[v] = distances[u] + edge_cost;
                    previous[v] = u;
                    pq.emplace(distances[v], v);
                }
            }
        }
    }

    // Reconstruir el camino
    std::vector<int> path;
    if (distances[end_node] == INF) {
        // No existe un camino
        return path;
    }

    for (int at = end_node; at != -1; at = previous[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Construir una solución greedy randomizada con penalizaciones suaves
Solution constructGreedyRandomizedSolution(const DataModel& data, std::mt19937& rng) {
    int num_vehicles = data.starts.size();
    Solution solution;
    solution.routes.resize(num_vehicles);
    solution.total_durations.resize(num_vehicles, 0);
    solution.makespan = 0;
    solution.total_penalty = 0;

    // Inicializar las rutas con el camino más corto desde el inicio al fin
    for (int i = 0; i < num_vehicles; ++i) {
        int start_node = data.starts[i];
        int end_node = data.ends[i];

        std::vector<int> path = dijkstra(start_node, end_node, data);
        if (path.empty()) {
            // No existe un camino; manejar adecuadamente
            std::cerr << "No existe un camino entre " << data.ubigeos[start_node]
                      << " y " << data.ubigeos[end_node] << " para el vehículo " << i + 1 << ".\n";
            // Excluir el nodo final
            excludeNode(solution, end_node, data);
            // Inicializar ruta solo con el nodo de inicio
            solution.routes[i].push_back(start_node);
            continue;
        }

        solution.routes[i] = path;
        // Las duraciones totales se actualizarán al insertar nodos
    }

    // Crear una lista de nodos por asignar (excluyendo starts y ends)
    std::vector<int> unassigned_nodes;
    for (size_t i = 0; i < data.ubigeos.size(); ++i) {
        bool is_start_or_end = false;
        for (size_t j = 0; j < num_vehicles; ++j) {
            if (i == data.starts[j] || i == data.ends[j]) {
                is_start_or_end = true;
                break;
            }
        }
        if (!is_start_or_end) {
            unassigned_nodes.push_back(i);
        }
    }

    // Mezclar los nodos por asignar
    std::shuffle(unassigned_nodes.begin(), unassigned_nodes.end(), rng);

    // Insertar nodos en rutas
    while (!unassigned_nodes.empty()) {
        int node = unassigned_nodes.back();
        unassigned_nodes.pop_back();

        long long min_cost_increase = INF;
        int best_vehicle = -1;
        size_t best_position = 0;

        // Para cada vehículo
        for (int v = 0; v < num_vehicles; ++v) {
            // Para cada posible posición de inserción en la ruta (entre nodos)
            for (size_t pos = 1; pos < solution.routes[v].size(); ++pos) {
                int prev_node = solution.routes[v][pos - 1];
                int next_node = solution.routes[v][pos];

                // Verificar si existen tramos desde prev_node a node y de node a next_node
                long long cost1 = data.timeMatrix[prev_node][node];
                long long cost2 = data.timeMatrix[node][next_node];
                long long cost0 = data.timeMatrix[prev_node][next_node];

                if (cost1 != INF && cost2 != INF) {
                    // Calcular incremento de costo
                    long long cost_increase = cost1 + cost2 - cost0;

                    if (cost_increase < min_cost_increase) {
                        min_cost_increase = cost_increase;
                        best_vehicle = v;
                        best_position = pos;
                    }
                }
            }
        }

        if (best_vehicle != -1 && min_cost_increase < data.exclusion_penalty) {
            // Insertar nodo en la ruta en la mejor posición
            solution.routes[best_vehicle].insert(solution.routes[best_vehicle].begin() + best_position, node);

            // Actualizar duración total
            int prev_node = solution.routes[best_vehicle][best_position - 1];
            int node_i = node;
            int next_node = solution.routes[best_vehicle][best_position + 1];

            long long cost0 = data.timeMatrix[prev_node][next_node];
            long long cost1 = data.timeMatrix[prev_node][node_i];
            long long cost2 = data.timeMatrix[node_i][next_node];

            solution.total_durations[best_vehicle] += (cost1 + cost2 - cost0);

            // Actualizar makespan
            solution.makespan = std::max(solution.makespan, solution.total_durations[best_vehicle]);
        } else {
            // Excluir nodo
            excludeNode(solution, node, data);
        }
    }

    // Recalcular duraciones totales y validar rutas
    solution.makespan = 0;
    for (int v = 0; v < num_vehicles; ++v) {
        long long total_duration = 0;
        bool route_valid = true;
        for (size_t i = 0; i < solution.routes[v].size() - 1; ++i) {
            int from = solution.routes[v][i];
            int to = solution.routes[v][i + 1];
            long long duration = data.timeMatrix[from][to];
            if (duration != INF) {
                total_duration += duration;
            } else {
                // El tramo no existe
                route_valid = false;
                break;
            }
        }
        if (route_valid) {
            solution.total_durations[v] = total_duration;
            solution.makespan = std::max(solution.makespan, total_duration);
        } else {
            // Ruta inválida; establecer makespan a INF
            solution.makespan = INF;
            break;
        }
    }

    return solution;
}

// Función de búsqueda local que considera exclusiones
bool localSearch(Solution& current_solution, const DataModel& data, std::mt19937& rng) {
    bool improved = false;
    int num_vehicles = current_solution.routes.size();
    long long best_makespan = current_solution.makespan;

    // Intentar intercambiar nodos entre pares de vehículos
    for (int i = 0; i < num_vehicles; ++i) {
        for (int j = i + 1; j < num_vehicles; ++j) {
            // Iterar sobre nodos de la ruta del vehículo i (excluir start y end)
            for (size_t m = 1; m < current_solution.routes[i].size() - 1; ++m) {
                // Iterar sobre nodos de la ruta del vehículo j (excluir start y end)
                for (size_t n = 1; n < current_solution.routes[j].size() - 1; ++n) {
                    // Intercambiar los nodos
                    std::swap(current_solution.routes[i][m], current_solution.routes[j][n]);

                    // Verificar si todos los tramos en ambas rutas existen después del intercambio
                    bool edges_exist = true;

                    // Verificar ruta i
                    for (size_t k = 0; k < current_solution.routes[i].size() - 1; ++k) {
                        int from = current_solution.routes[i][k];
                        int to = current_solution.routes[i][k + 1];
                        if (data.timeMatrix[from][to] == INF) {
                            edges_exist = false;
                            break;
                        }
                    }

                    // Verificar ruta j si ruta i es válida
                    if (edges_exist) {
                        for (size_t k = 0; k < current_solution.routes[j].size() - 1; ++k) {
                            int from = current_solution.routes[j][k];
                            int to = current_solution.routes[j][k + 1];
                            if (data.timeMatrix[from][to] == INF) {
                                edges_exist = false;
                                break;
                            }
                        }
                    }

                    if (edges_exist) {
                        // Recalcular duraciones para rutas i y j
                        long long new_duration_i = 0;
                        for (size_t k = 0; k < current_solution.routes[i].size() - 1; ++k) {
                            int from = current_solution.routes[i][k];
                            int to = current_solution.routes[i][k + 1];
                            new_duration_i += data.timeMatrix[from][to];
                        }

                        long long new_duration_j = 0;
                        for (size_t k = 0; k < current_solution.routes[j].size() - 1; ++k) {
                            int from = current_solution.routes[j][k];
                            int to = current_solution.routes[j][k + 1];
                            new_duration_j += data.timeMatrix[from][to];
                        }

                        // Actualizar makespan temporal para comparación
                        long long temp_makespan = std::max(new_duration_i, new_duration_j);

                        // Iterar sobre todas las rutas para encontrar la duración máxima actual
                        for (int k = 0; k < num_vehicles; ++k) {
                            if (k != i && k != j) {
                                if (current_solution.total_durations[k] > temp_makespan) {
                                    temp_makespan = current_solution.total_durations[k];
                                }
                            }
                        }

                        // Verificar si el nuevo makespan es mejor
                        if (temp_makespan < best_makespan) {
                            // Actualizar las duraciones y makespan
                            current_solution.total_durations[i] = new_duration_i;
                            current_solution.total_durations[j] = new_duration_j;
                            current_solution.makespan = temp_makespan;
                            best_makespan = temp_makespan;
                            improved = true;
                        } else {
                            // Revertir intercambio si no hay mejora
                            std::swap(current_solution.routes[i][m], current_solution.routes[j][n]);
                        }
                    } else {
                        // Revertir intercambio si los tramos no existen
                        std::swap(current_solution.routes[i][m], current_solution.routes[j][n]);
                    }
                }
            }
        }
    }

    return improved;
}

// Función para imprimir la solución, incluyendo nodos excluidos
void printSolution(const Solution& solution, 
                   const std::vector<std::string>& ubigeos, 
                   const std::vector<std::vector<long long>>& timeMatrix, 
                   const std::unordered_map<std::string, Location>& locations_map,
                   long long exclusion_penalty) {
    if (solution.makespan == INF) {
        std::cout << "No se encontró una solución válida.\n";
        return;
    }

    std::cout << "Mejor solución encontrada con makespan: " 
              << formatDuration(solution.makespan) << ".\n\n";

    for (size_t i = 0; i < solution.routes.size(); ++i) {
        std::cout << "Vehículo " << i + 1 << ":\n";
        std::cout << "Ruta: ";
        for (size_t j = 0; j < solution.routes[i].size(); ++j) {
            std::string ubigeo = ubigeos[solution.routes[i][j]];
            const auto& location = locations_map.at(ubigeo);
            std::cout << location.province << " (" << location.ubigeo << ")";
            if (j != solution.routes[i].size() - 1) std::cout << " -> ";
        }
        std::cout << "\n";

        std::cout << "Duración Total: " << formatDuration(solution.total_durations[i]) << ".\n";

        std::cout << "Duración de cada tramo:\n";
        for (size_t j = 0; j < solution.routes[i].size() - 1; ++j) {
            int from = solution.routes[i][j];
            int to = solution.routes[i][j + 1];
            std::string from_ubigeo = ubigeos[from];
            std::string to_ubigeo = ubigeos[to];
            const auto& from_location = locations_map.at(from_ubigeo);
            const auto& to_location = locations_map.at(to_ubigeo);

            long long duration = (timeMatrix[from][to] != INF) 
                     ? timeMatrix[from][to] 
                     : exclusion_penalty;

            std::cout << "  " << from_location.province << " (" << from_ubigeo << ") -> " 
                      << to_location.province << " (" << to_ubigeo << "): " 
                      << formatDuration(duration) << ".\n";
        }
        std::cout << "\n";
    }
}

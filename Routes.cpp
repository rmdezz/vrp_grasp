// Routes.cpp
#include "Routes.h"
#include "DataModel.h"
#include "Utils.h"
#include <algorithm>
#include <iostream>
#include <queue>
#include <unordered_set>

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

// Construir una solución greedy randomizada con penalizaciones suaves y α
Solution constructGreedyRandomizedSolution(const DataModel& data, std::mt19937& rng, double alpha) {
    int num_vehicles = data.starts.size(); // Número de vehículos
    Solution solution; // Estructura para almacenar la solución
    solution.routes.resize(num_vehicles); // Inicializar las rutas para cada vehículo
    solution.total_durations.resize(num_vehicles, 0); // Inicializar las duraciones totales de cada vehículo
    solution.makespan = 0; // Inicializar el makespan a 0
    solution.total_penalty = 0; // Inicializar la penalización total a 0

    // Inicializar las rutas con el camino más corto desde el nodo de inicio al nodo de fin
    for (int i = 0; i < num_vehicles; ++i) {
        int start_node = data.starts[i]; // Nodo inicial del vehículo
        int end_node = data.ends[i]; // Nodo final del vehículo

        // Usar el algoritmo de Dijkstra para encontrar el camino más corto
        std::vector<int> path = dijkstra(start_node, end_node, data);
        if (path.empty()) {
            // Si no existe un camino válido, manejar el error
            std::cerr << "No existe un camino entre " << data.ubigeos[start_node]
                      << " y " << data.ubigeos[end_node] << " para el vehículo " << i + 1 << ".\n";
            // Excluir el nodo final y asignar solo el nodo inicial a la ruta
            excludeNode(solution, end_node, data);
            solution.routes[i].push_back(start_node);
            continue;
        }

        solution.routes[i] = path; // Asignar el camino más corto a la ruta del vehículo
        // Las duraciones totales se actualizarán posteriormente
    }

    // Crear un conjunto de nodos asignados a partir de las rutas iniciales
    std::unordered_set<int> assigned_nodes;
    for (const auto& route : solution.routes) {
        for (int node : route) {
            assigned_nodes.insert(node); // Marcar los nodos como asignados
        }
    }

    // Crear una lista de nodos no asignados (excluyendo los ya asignados)
    std::vector<int> unassigned_nodes;
    for (size_t i = 0; i < data.ubigeos.size(); ++i) {
        if (assigned_nodes.find(i) == assigned_nodes.end()) {
            unassigned_nodes.push_back(i); // Agregar los nodos no asignados
        }
    }

    // Mientras queden nodos por asignar
    while (!unassigned_nodes.empty()) {
        // Lista para almacenar posibles inserciones
        struct Insertion {
            int node; // Nodo a insertar
            int vehicle; // Vehículo al que se insertará
            size_t position; // Posición en la ruta donde se insertará
            long long cost_increase; // Incremento en el costo al insertar
        };
        std::vector<Insertion> candidate_insertions; // Candidatos a inserción

        long long min_cost_increase = INF; // Costo mínimo inicial
        long long max_cost_increase = 0; // Costo máximo inicial

        // Para cada nodo no asignado
        for (int node : unassigned_nodes) {
            // Para cada vehículo
            for (int v = 0; v < num_vehicles; ++v) {
                // Para cada posición posible en la ruta (entre nodos existentes)
                for (size_t pos = 1; pos < solution.routes[v].size(); ++pos) {
                    int prev_node = solution.routes[v][pos - 1]; // Nodo previo
                    int next_node = solution.routes[v][pos]; // Nodo siguiente

                    // Verificar si existen los tramos
                    long long cost1 = data.timeMatrix[prev_node][node]; // Costo al nodo
                    long long cost2 = data.timeMatrix[node][next_node]; // Costo desde el nodo
                    long long cost0 = data.timeMatrix[prev_node][next_node]; // Costo directo previo-siguiente

                    if (cost1 != INF && cost2 != INF && cost0 != INF) {
                        // Calcular el incremento de costo al insertar el nodo
                        long long cost_increase = cost1 + cost2 - cost0;

                        // **Incluir solo si el incremento de costo es menor a la penalización por exclusión**
                        if (cost_increase < data.exclusion_penalty) {
                            // Actualizar los costos mínimos y máximos
                            if (cost_increase < min_cost_increase) min_cost_increase = cost_increase;
                            if (cost_increase > max_cost_increase) max_cost_increase = cost_increase;

                            // Almacenar la inserción como candidata
                            candidate_insertions.push_back({node, v, pos, cost_increase});
                        }
                    }
                }
            }
        }

        if (candidate_insertions.empty()) {
            // Si no hay inserciones posibles, excluir los nodos restantes
            for (int node : unassigned_nodes) {
                excludeNode(solution, node, data);
            }
            break;
        }

        // Calcular el umbral para la lista restringida de candidatos (RCL)
        long long threshold = min_cost_increase + alpha * (max_cost_increase - min_cost_increase);

        // **Asegurar que el umbral no exceda la penalización por exclusión**
        if (threshold >= data.exclusion_penalty) {
            threshold = data.exclusion_penalty - 1;
        }

        // Construir la RCL
        std::vector<Insertion> RCL;
        for (const auto& insertion : candidate_insertions) {
            if (insertion.cost_increase <= threshold) {
                RCL.push_back(insertion); // Agregar las inserciones dentro del umbral
            }
        }

        if (RCL.empty()) {
            // Si la RCL está vacía, incluir el mejor candidato
            RCL.push_back(*std::min_element(candidate_insertions.begin(), candidate_insertions.end(),
                                            [](const Insertion& a, const Insertion& b) {
                                                return a.cost_increase < b.cost_increase;
                                            }));
        }

        // Seleccionar aleatoriamente una inserción de la RCL
        std::uniform_int_distribution<size_t> dist(0, RCL.size() - 1);
        const Insertion& selected_insertion = RCL[dist(rng)];

        // Insertar el nodo
        int node = selected_insertion.node;
        int vehicle = selected_insertion.vehicle;
        size_t position = selected_insertion.position;

        solution.routes[vehicle].insert(solution.routes[vehicle].begin() + position, node);

        // Actualizar las duraciones totales y el makespan
        long long total_duration = 0;
        bool route_valid = true;
        for (size_t i = 0; i < solution.routes[vehicle].size() - 1; ++i) {
            int from = solution.routes[vehicle][i];
            int to = solution.routes[vehicle][i + 1];
            long long duration = data.timeMatrix[from][to];
            if (duration != INF) {
                total_duration += duration; // Sumar la duración de los tramos
            } else {
                route_valid = false;
                break;
            }
        }
        if (route_valid) {
            solution.total_durations[vehicle] = total_duration;
            solution.makespan = *std::max_element(solution.total_durations.begin(), solution.total_durations.end());
        }

        // Eliminar el nodo de la lista de nodos no asignados
        unassigned_nodes.erase(std::remove(unassigned_nodes.begin(), unassigned_nodes.end(), node), unassigned_nodes.end());
    }

    // Recalcular las duraciones totales y validar rutas
    solution.makespan = *std::max_element(solution.total_durations.begin(), solution.total_durations.end());

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

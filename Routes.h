// Routes.h
#ifndef ROUTES_H
#define ROUTES_H

#include <vector>
#include "Solution.h"
#include "Utils.h"
#include "DataModel.h"
#include <random>

// Declaración de funciones específicas de GRASP
Solution constructGreedyRandomizedSolution(const DataModel& data, std::mt19937& rng);

bool localSearch(Solution& current_solution, const DataModel& data, std::mt19937& rng);

void printSolution(const Solution& solution, 
                   const std::vector<std::string>& ubigeos, 
                   const std::vector<std::vector<long long>>& timeMatrix, 
                   const std::unordered_map<std::string, Location>& locations_map,
                   long long exclusion_penalty);

#endif // ROUTES_H

// DataModel.h
#ifndef DATAMODEL_H
#define DATAMODEL_H

#include <vector>
#include <string>

struct DataModel {
    std::vector<std::vector<long long>> timeMatrix; // Matriz de tiempos
    std::vector<std::string> ubigeos; // Lista de ubigeos
    std::vector<int> starts; // Índices de nodos de inicio por vehículo
    std::vector<int> ends; // Índices de nodos de fin por vehículo
    long long exclusion_penalty; // Penalización por nodo excluido
};

#endif // DATAMODEL_H

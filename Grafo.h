#ifndef GRAFO_H
#define GRAFO_H

#include <iostream>
#include <list>
#include <unordered_map>
#include <stdexcept>
#include <stack>
#include <queue>
#include <limits>
#include <fstream>
#include <sstream>
#include <vector>

#include "Personaje.h"
#include "Relacion.h"


template<typename T, typename E>

class Grafo{
private:
    std::unordered_map<std::string, int> nodoIndices;
    std::vector<T> nodos;
    std::vector<E> aristas;
    std::vector<std::vector<E>> matrizAdyacencia;

public:
    Grafo();
    ~Grafo();

    void agregarNodo(const std::string& id, const T& nodo);
    void eliminarNodo(const std::string& id);
    void agregarArista(const E& arista);

    void DFS(const std::string& inicio) const;
    void BFS(const std::string& inicio) const;
    void dijkstra(const std::string& inicio) const;
    void floydWarshall();


    const std::vector<std::vector<E>>& obtenerMatrizAdyacencia();
    void planoGrafo();

    void generarGrafo(const std::string& pathNodos, const std::string& pathAristas);
};

#endif

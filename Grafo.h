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
#include <iomanip>
#include <set>
#include <algorithm>

#include "Personaje.h"
#include "Relacion.h"
#include "Comando.h"


template<typename T, typename E>

class Grafo{
private:
    std::unordered_map<std::string, int> nodoIndices;
    std::vector<T> nodos;
    std::vector<E> aristas;
    std::vector<std::vector<int>> matrizAdyacencia;
    std::vector<Comando> comandos;

public:
    Grafo();
    ~Grafo();

    void agregarNodo(const std::string& id, const T& nodo);
    void eliminarNodo(const std::string& id);
    void agregarArista(const E& arista);

    void DFSTriangulos(int nodoActual, int nodoInicial, std::vector<int>& camino,std::vector<bool>& visitado, std::set<std::set<int>>& triangulos) const;
	void encontrarTriangulosDFS() const;
    void BFS(const std::string& inicio) const;
    void dijkstra(const std::string& inicio) const;
    void floydWarshall();


    const std::vector<std::vector<int>>& obtenerMatrizAdyacencia();
    void planoGrafo();

    void mostrarAyuda();
    void borrarPantalla();
    void procesarComando(const std::string& comando);
    void cargarComandos(const std::string& nombreArchivo);
    void listarComandos();

    void generarGrafo(const std::string& pathNodos, const std::string& pathAristas);
};

#endif

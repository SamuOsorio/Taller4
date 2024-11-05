#include <iostream>
#include "Grafo.h"
#include "Relacion.h"
#include "Personaje.h"
#include <string>

int main() {
    try {
        Grafo<Personaje, Relacion> grafo;

        // Cargar grafo desde archivos CSV
        std::string pathNodos = "Nodes.csv";
        std::string pathAristas = "book1.csv";

        std::cout << "Cargando archivos..." << std::endl;
        grafo.generarGrafo(pathNodos, pathAristas);

        // Mostrar nodos disponibles
        std::cout << "\nNodos disponibles en el grafo:" << std::endl;
        grafo.planoGrafo();

        // Solicitar nodo inicial
        std::string nodoInicial;
        std::cout << "\nIngrese el nombre de un nodo de la lista anterior para comenzar el análisis: ";
        std::getline(std::cin, nodoInicial);

        // Realizar búsqueda DFS
        std::cout << "\nBúsqueda en profundidad (DFS) desde el nodo " << nodoInicial << ":" << std::endl;
        grafo.DFS(nodoInicial);

        // Realizar búsqueda BFS
        std::cout << "\nBúsqueda en amplitud (BFS) desde el nodo " << nodoInicial << ":" << std::endl;
        grafo.BFS(nodoInicial);

        // Ejecutar Dijkstra
        std::cout << "\nResultados de Dijkstra desde el nodo " << nodoInicial << ":" << std::endl;
        grafo.dijkstra(nodoInicial);

        // Ejecutar Floyd-Warshall
        std::cout << "\nResultados de Floyd-Warshall:" << std::endl;
        grafo.floydWarshall();

    } catch (const std::ifstream::failure& e) {
        std::cerr << "Error al abrir o leer los archivos: " << e.what() << std::endl;
        return 1;
    } catch (const std::out_of_range& e) {
        std::cerr << "Error: Nodo no encontrado en el grafo. Por favor, elija un nodo válido de la lista mostrada." << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error inesperado: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

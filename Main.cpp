#include <iostream>
#include "Grafo.h"
#include "Relacion.h"
#include "Personaje.h"


int main() {
    Grafo<Personaje, Relacion> grafo;  // Reemplaza 'Nodo' y 'Arista' con los nombres reales de tus clases.

    // Cargar grafo desde archivos CSV
    std::string pathNodos = "Nodes.csv";  // Cambia la ruta si es necesario
    std::string pathAristas = "book1.csv";  // Cambia la ruta si es necesario
    grafo.generarGrafo(pathNodos, pathAristas);

    // Mostrar nodos
    std::cout << "Nodos en el grafo:" << std::endl;
    grafo.planoGrafo();

    // Realizar búsqueda DFS desde un nodo inicial, por ejemplo "A"
    std::cout << "\nBúsqueda en profundidad (DFS) desde el nodo A:" << std::endl;
    grafo.DFS("A");

    // Realizar búsqueda BFS desde el mismo nodo
    std::cout << "\nBúsqueda en amplitud (BFS) desde el nodo A:" << std::endl;
    grafo.BFS("A");

    // Ejecutar Dijkstra desde el nodo A
    std::cout << "\nResultados de Dijkstra desde el nodo A:" << std::endl;
    grafo.dijkstra("A");

    // Ejecutar Floyd-Warshall
    std::cout << "\nResultados de Floyd-Warshall:" << std::endl;
    grafo.floydWarshall();

    return 0;
}

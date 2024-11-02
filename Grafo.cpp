#include "Grafo.h"

template<typename T, typename E>
Grafo<T,E>::Grafo() {}

template<typename T, typename E>
Grafo<T,E>::~Grafo() {}

template<typename T, typename E>
void Grafo<T,E>::agregarNodo(const std::string& id, const T& nodo)
{
    if (nodoIndices.find(id) == nodoIndices.end())
    {
        int index = nodos.size();
        nodoIndices[id] = index;
        nodos.push_back(nodo);

        for (auto& fila : matrizAdyacencia)
        {
            fila.push_back(std::numeric_limits<int>::max());
        }
        matrizAdyacencia.push_back(std::vector<int>(nodos.size(), std::numeric_limits<int>::max()));
        matrizAdyacencia[index][index] = 0;
    }
}

template<typename T, typename E>
void Grafo<T,E>::eliminarNodo(const std::string& id)
{
    auto it = nodoIndices.find(id);
    if (it != nodoIndices.end())
    {
        int index = it->second;
        nodos.erase(nodos.begin() + index);
        matrizAdyacencia.erase(matrizAdyacencia.begin() + index);
        for (auto& fila : matrizAdyacencia)
        {
            fila.erase(fila.begin() + index);
        }
        nodoIndices.erase(it);
    }
}

template<typename T, typename E>
void Grafo<T,E>::agregarArista(const E& arista)
{
    int peso = arista.getPeso();
    std::string source = arista.getSource();
    std::string target = arista.getTarget();

    if (nodoIndices.find(source) != nodoIndices.end() && nodoIndices.find(target) != nodoIndices.end())
    {
        int sourceIndex = nodoIndices[source];
        int targetIndex = nodoIndices[target];

        matrizAdyacencia[sourceIndex][targetIndex] = peso;

        if (arista.getType() != "Directed")
        {
            matrizAdyacencia[targetIndex][sourceIndex] = peso;
        }

        aristas.push_back(arista);
    }
}


template<typename T, typename E>
void Grafo<T,E>::DFS(const std::string& inicio) const
{
    std::vector<bool> visitado(nodos.size(), false);
    std::stack<int> stack;
    int inicioIndex = nodoIndices.at(inicio);
    stack.push(inicioIndex);

    while (!stack.empty())
    {
        int nodoActual = stack.top();
        stack.pop();

        if (!visitado[nodoActual])
        {
            visitado[nodoActual] = true;
            std::cout << nodos[nodoActual].getNombre() << " " << std::endl;

            for (size_t i = 0; i < matrizAdyacencia.size(); ++i)
            {
                if (matrizAdyacencia[nodoActual][i] != std::numeric_limits<int>::max() && !visitado[i])
                {
                    stack.push(i);
                }
            }
        }
    }
}

template<typename T, typename E>
void Grafo<T,E>::BFS(const std::string& inicio) const
{
    std::vector<bool> visitado(nodos.size(), false);
    std::queue<int> queue;
    int inicioIndex = nodoIndices.at(inicio);
    queue.push(inicioIndex);

    while (!queue.empty())
    {
        int nodoActual = queue.front();
        queue.pop();

        if (!visitado[nodoActual])
        {
            visitado[nodoActual] = true;
            std::cout << nodos[nodoActual].getNombre() << " " << std::endl;

            for (size_t i = 0; i < matrizAdyacencia.size(); ++i)
            {
                if (matrizAdyacencia[nodoActual][i] != std::numeric_limits<int>::max() && !visitado[i])
                {
                    queue.push(i);
                }
            }
        }
    }
}

template<typename T, typename E>
void Grafo<T, E>::dijkstra(const std::string& inicio) const
{
    int inicioIndex = nodoIndices.at(inicio);
    std::vector<int> distancias(nodos.size(), std::numeric_limits<int>::max());
    distancias[inicioIndex] = 0;

    for (int i = 0; i < nodos.size(); ++i)
    {
        for (int u = 0; u < nodos.size(); ++u)
        {
            for (int v = 0; v < nodos.size(); ++v)
            {
                if (matrizAdyacencia[u][v] != std::numeric_limits<int>::max() && distancias[u] != std::numeric_limits<int>::max() && distancias[u] + matrizAdyacencia[u][v] < distancias[v])
                {
                    distancias[v] = distancias[u] + matrizAdyacencia[u][v];
                }
            }
        }
    }

    for (size_t i = 0; i < distancias.size(); ++i)
    {
        std::cout << nodos[i].getNombre() << ": " << distancias[i] << "\n";
    }
}

template<typename T, typename E>
void Grafo<T, E>::floydWarshall()
{
    std::vector<std::vector<int>> dist = matrizAdyacencia;

    for (int k = 0; k < nodos.size(); ++k)
    {
        for (int i = 0; i < nodos.size(); ++i)
        {
            for (int j = 0; j < nodos.size(); ++j)
            {
                if (dist[i][k] != std::numeric_limits<int>::max() && dist[k][j] != std::numeric_limits<int>::max())
                {
                    dist[i][j] = std::min(dist[i][j], dist[i][k] + dist[k][j]);
                }
            }
        }
    }

    for (int i = 0; i < nodos.size(); ++i)
    {
        for (int j = 0; j < nodos.size(); ++j)
        {
            if (dist[i][j] == std::numeric_limits<int>::max())
            {
                std::cout << "INF ";
            }
            else
            {
                std::cout << dist[i][j] << " ";
            }
        }
        std::cout << "\n";
    }
}

template<typename T, typename E>
void Grafo<T,E>::planoGrafo() {
    std::cout << "Lista de nodos disponibles:" << std::endl;
    std::cout << "-------------------------" << std::endl;
    for (const auto& nodo : nodos) {
        std::cout << "Nombre: " << nodo.getNombre()
                  << " | Edad: " << nodo.getEdad()
                  << " | Altura: " << nodo.getAltura()
                  << " | Popularidad: " << nodo.getPopularidad()
                  << std::endl;
    }
    std::cout << "-------------------------" << std::endl;
}

template<typename T, typename E>
void Grafo<T, E>::generarGrafo(const std::string& pathNodos, const std::string& pathAristas) {
    std::ifstream fileNodos(pathNodos);
    std::ifstream fileAristas(pathAristas);
    std::string linea;

    if (!fileNodos.is_open() || !fileAristas.is_open()) {
        throw std::runtime_error("Error al abrir el/los archivos: " + pathNodos + " " + pathAristas);
    }

    std::cout << "Leyendo archivo de nodos..." << std::endl;

    // Saltar la primera línea (cabecera) del archivo de nodos
    std::getline(fileNodos, linea);

    // Procesar el resto de las líneas
    while (std::getline(fileNodos, linea)) {
        try {
            std::istringstream iss(linea);
            std::string nombre, edadStr, alturaStr, popularidadStr;

            getline(iss, nombre, ',');
            getline(iss, edadStr, ',');
            getline(iss, alturaStr, ',');
            getline(iss, popularidadStr, ',');

            // Eliminar espacios en blanco al inicio y final si los hay
            nombre.erase(0, nombre.find_first_not_of(" \t\r\n"));
            nombre.erase(nombre.find_last_not_of(" \t\r\n") + 1);
            edadStr.erase(0, edadStr.find_first_not_of(" \t\r\n"));
            edadStr.erase(edadStr.find_last_not_of(" \t\r\n") + 1);
            alturaStr.erase(0, alturaStr.find_first_not_of(" \t\r\n"));
            alturaStr.erase(alturaStr.find_last_not_of(" \t\r\n") + 1);
            popularidadStr.erase(0, popularidadStr.find_first_not_of(" \t\r\n"));
            popularidadStr.erase(popularidadStr.find_last_not_of(" \t\r\n") + 1);

            int edad = std::stoi(edadStr);
            float altura = std::stof(alturaStr);
            int popularidad = std::stoi(popularidadStr);

            agregarNodo(nombre, T(nombre, edad, altura, popularidad));
        } catch (const std::exception& e) {
            throw std::runtime_error("Error procesando nodo en línea: " + linea + ": " + e.what());
        }
    }
    fileNodos.close();

    std::cout << "Leyendo archivo de aristas..." << std::endl;

    // Saltar la primera línea (cabecera) del archivo de aristas
    std::getline(fileAristas, linea);

    while (std::getline(fileAristas, linea)) {
        try {
            std::istringstream iss(linea);
            std::string source, target, type, pesoStr, libroStr;

            getline(iss, source, ',');
            getline(iss, target, ',');
            getline(iss, type, ',');
            getline(iss, pesoStr, ',');
            getline(iss, libroStr, ',');

            // Eliminar espacios en blanco
            source.erase(0, source.find_first_not_of(" \t\r\n"));
            source.erase(source.find_last_not_of(" \t\r\n") + 1);
            target.erase(0, target.find_first_not_of(" \t\r\n"));
            target.erase(target.find_last_not_of(" \t\r\n") + 1);
            type.erase(0, type.find_first_not_of(" \t\r\n"));
            type.erase(type.find_last_not_of(" \t\r\n") + 1);
            pesoStr.erase(0, pesoStr.find_first_not_of(" \t\r\n"));
            pesoStr.erase(pesoStr.find_last_not_of(" \t\r\n") + 1);
            libroStr.erase(0, libroStr.find_first_not_of(" \t\r\n"));
            libroStr.erase(libroStr.find_last_not_of(" \t\r\n") + 1);

            std::string tipo = (type == "Directed") ? "Directed" : "Undirected";
            int peso = std::stoi(pesoStr);
            int libro = std::stoi(libroStr);

            agregarArista(E(source, target, tipo, peso, libro));
        } catch (const std::exception& e) {
            throw std::runtime_error("Error procesando arista en línea: " + linea + ": " + e.what());
        }
    }
    fileAristas.close();
}

template<typename T, typename E>
const std::vector<std::vector<int>>& Grafo<T,E> ::obtenerMatrizAdyacencia()
{
    return matrizAdyacencia;
}

template class Grafo<Personaje, Relacion>;

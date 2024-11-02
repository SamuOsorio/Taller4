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
    if (nodoIndices.find(inicio) == nodoIndices.end()) {
        throw std::out_of_range("Nodo inicial no encontrado");
    }

    std::vector<bool> visitado(nodos.size(), false);
    std::stack<int> stack;
    int inicioIndex = nodoIndices.at(inicio);

    std::cout << "Recorrido DFS comenzando desde " << inicio << ":\n";

    // Comenzar con el nodo inicial
    stack.push(inicioIndex);

    while (!stack.empty()) {
        int nodoActual = stack.top();
        stack.pop();

        if (!visitado[nodoActual]) {
            // Marcar como visitado y mostrar
            visitado[nodoActual] = true;
            std::cout << nodos[nodoActual].getNombre();

            // Verificar si hay más nodos por visitar
            bool tieneVecinos = false;
            for (int i = matrizAdyacencia[nodoActual].size() - 1; i >= 0; --i) {
                if (matrizAdyacencia[nodoActual][i] != std::numeric_limits<int>::max() && !visitado[i]) {
                    stack.push(i);
                    tieneVecinos = true;
                }
            }

            // Agregar una flecha si hay más nodos por visitar
            if (!stack.empty() && tieneVecinos) {
                std::cout << " -> ";
            } else {
                std::cout << std::endl;
            }
        }
    }
    std::cout << "\nFin del recorrido DFS" << std::endl;
}

template<typename T, typename E>
void Grafo<T,E>::BFS(const std::string& inicio) const
{
    if (nodoIndices.find(inicio) == nodoIndices.end()) {
        throw std::out_of_range("Nodo inicial no encontrado");
    }

    std::vector<bool> visitado(nodos.size(), false);
    std::queue<int> cola;
    int inicioIndex = nodoIndices.at(inicio);

    std::cout << "Recorrido BFS comenzando desde " << inicio << ":\n";

    // Comenzar con el nodo inicial
    cola.push(inicioIndex);
    visitado[inicioIndex] = true;  // Marcamos como visitado al agregar a la cola

    while (!cola.empty()) {
        int nodoActual = cola.front();
        cola.pop();

        // Mostrar el nodo actual
        std::cout << nodos[nodoActual].getNombre();

        // Contador para vecinos no visitados
        int vecinosNoVisitados = 0;

        // Agregar todos los vecinos no visitados a la cola
        for (size_t i = 0; i < matrizAdyacencia[nodoActual].size(); ++i) {
            if (matrizAdyacencia[nodoActual][i] != std::numeric_limits<int>::max() && !visitado[i]) {
                cola.push(i);
                visitado[i] = true;  // Marcamos como visitado al agregar a la cola
                vecinosNoVisitados++;
            }
        }

        // Agregar una flecha si hay más nodos por visitar
        if (!cola.empty() && vecinosNoVisitados > 0) {
            std::cout << " -> ";
        } else {
            std::cout << std::endl;
        }
    }
    std::cout << "\nFin del recorrido BFS" << std::endl;
}

template<typename T, typename E>
void Grafo<T, E>::dijkstra(const std::string& inicio) const {
    if (nodoIndices.find(inicio) == nodoIndices.end()) {
        throw std::out_of_range("Nodo inicial no encontrado");
    }

    int inicioIndex = nodoIndices.at(inicio);
    int V = nodos.size();

    // Vector de distancias y vector para almacenar el camino
    std::vector<int> distancias(V, std::numeric_limits<int>::max());
    std::vector<int> previo(V, -1);
    std::vector<bool> visitado(V, false);

    // Cola de prioridad para mantener los nodos por explorar
    // pair<distancia, nodoIndex>
    std::priority_queue<std::pair<int, int>,
                       std::vector<std::pair<int, int>>,
                       std::greater<std::pair<int, int>>> pq;

    // Inicializar distancia del nodo inicial
    distancias[inicioIndex] = 0;
    pq.push({0, inicioIndex});

    std::cout << "\nCalculando distancias mínimas desde " << inicio << ":\n";
    std::cout << "----------------------------------------\n";

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visitado[u]) continue;
        visitado[u] = true;

        // Explorar todos los vecinos del nodo actual
        for (int v = 0; v < V; v++) {
            // Si hay una arista y no ha sido visitado
            if (matrizAdyacencia[u][v] != std::numeric_limits<int>::max()) {
                int peso = matrizAdyacencia[u][v];

                // Si encontramos un camino más corto
                if (distancias[u] != std::numeric_limits<int>::max() &&
                    distancias[u] + peso < distancias[v]) {
                    distancias[v] = distancias[u] + peso;
                    previo[v] = u;
                    pq.push({distancias[v], v});
                }
            }
        }
    }

    // Mostrar resultados
    for (int i = 0; i < V; i++) {
        std::cout << "Distancia a " << nodos[i].getNombre() << ": ";
        if (distancias[i] == std::numeric_limits<int>::max()) {
            std::cout << "INF";
        } else {
            std::cout << distancias[i];

            // Mostrar el camino
            std::cout << " | Camino: ";
            std::vector<int> camino;
            int actual = i;
            while (actual != -1) {
                camino.push_back(actual);
                actual = previo[actual];
            }

            // Imprimir el camino en orden correcto
            for (int j = camino.size() - 1; j >= 0; j--) {
                std::cout << nodos[camino[j]].getNombre();
                if (j > 0) std::cout << " -> ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << "----------------------------------------\n";
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

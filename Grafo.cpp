#include "Grafo.h"
#include "Comando.h"

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
    if (nodoIndices.find(inicio) == nodoIndices.end())
    {
        throw std::out_of_range("Nodo inicial no encontrado");
    }

    std::vector<bool> visitado(nodos.size(), false);
    std::stack<int> stack;
    int inicioIndex = nodoIndices.at(inicio);

    std::cout << "Recorrido DFS comenzando desde " << inicio << ":\n";
    std::cout << std::string(40, '-') << std::endl;

    // Comenzar con el nodo inicial
    stack.push(inicioIndex);

    while (!stack.empty())
    {
        int nodoActual = stack.top();
        stack.pop();

        if (!visitado[nodoActual])
        {
            // Marcar como visitado y
            visitado[nodoActual] = true;
            std::cout << "- " << std::setw(20) << std::left << nodos[nodoActual].getNombre() << std::endl;

            // Verificar si hay más nodos por visitar
            bool tieneVecinos = false;
            for (int i = matrizAdyacencia[nodoActual].size() - 1; i >= 0; --i)
            {
                if (matrizAdyacencia[nodoActual][i] != std::numeric_limits<int>::max() && !visitado[i])
                {
                    stack.push(i);
                    tieneVecinos = true;
                }
            }
            /*

            // Agregar una flecha si hay más nodos por visitar
            if (!stack.empty() && tieneVecinos)
            {
                std::cout << " -> ";
            }
            else
            {
                std::cout << std::endl;
            }
            */
        }
    }
    std::cout << "\nFin del recorrido DFS" << std::endl;
}

template<typename T, typename E>
void Grafo<T,E>::BFS(const std::string& inicio) const
{
    if (nodoIndices.find(inicio) == nodoIndices.end())
    {
        throw std::out_of_range("Nodo inicial no encontrado");
    }

    std::vector<bool> visitado(nodos.size(), false);
    std::queue<int> cola;
    int inicioIndex = nodoIndices.at(inicio);

    std::cout << "Recorrido BFS comenzando desde " << inicio << ":\n";
    std::cout << std::string(40, '-') << std::endl;

    // Comenzar con el nodo inicial
    cola.push(inicioIndex);
    visitado[inicioIndex] = true;  // Marcamos como visitado al agregar a la cola

    while (!cola.empty())
    {
        int nodoActual = cola.front();
        cola.pop();

        // Mostrar el nodo actual
        std::cout << "- " << std::setw(20) << std::left << nodos[nodoActual].getNombre() << std::endl;



        // Contador para vecinos no visitados
        int vecinosNoVisitados = 0;

        // Agregar todos los vecinos no visitados a la cola
        for (size_t i = 0; i < matrizAdyacencia[nodoActual].size(); ++i)
        {
            if (matrizAdyacencia[nodoActual][i] != std::numeric_limits<int>::max() && !visitado[i])
            {
                cola.push(i);
                visitado[i] = true;  // Marcamos como visitado al agregar a la cola
                vecinosNoVisitados++;
            }
        }
        /*

        // Agregar una flecha si hay más nodos por visitar
        if (!cola.empty() && vecinosNoVisitados > 0)
        {
            std::cout << " -> ";
        }
        else
        {
            std::cout << std::endl;
        }
        */
    }
    std::cout << "\nFin del recorrido BFS" << std::endl;
}

template<typename T, typename E>
void Grafo<T, E>::dijkstra(const std::string& inicio) const
{
    if (nodoIndices.find(inicio) == nodoIndices.end())
    {
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

    std::cout << "\nDistancias minimas desde el nodo '" << inicio << "':" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    std::cout << std::setw(20) << std::left << "Destino" << std::setw(20) << "Distancia" << "Camino" << std::endl;

    while (!pq.empty())
    {
        int u = pq.top().second;
        pq.pop();

        if (visitado[u]) continue;
        visitado[u] = true;

        // Explorar todos los vecinos del nodo actual
        for (int v = 0; v < V; v++)
        {
            // Si hay una arista y no ha sido visitado
            if (matrizAdyacencia[u][v] != std::numeric_limits<int>::max())
            {
                int peso = matrizAdyacencia[u][v];

                // Si encontramos un camino más corto
                if (distancias[u] != std::numeric_limits<int>::max() &&
                        distancias[u] + peso < distancias[v])
                {
                    distancias[v] = distancias[u] + peso;
                    previo[v] = u;
                    pq.push({distancias[v], v});
                }
            }
        }
    }

    for (int i = 0; i < V; i++) {
        if (distancias[i] == std::numeric_limits<int>::max()) continue;
        std::cout << std::setw(20) << nodos[i].getNombre() << std::setw(10) << distancias[i];

        // Imprimir camino
        std::vector<int> camino;
        for (int actual = i; actual != -1; actual = previo[actual]) {
            camino.push_back(actual);
        }
        for (int j = camino.size() - 1; j >= 0; j--) {
            std::cout << nodos[camino[j]].getNombre();
            if (j > 0) std::cout << " -> ";
        }
        std::cout << std::endl;
    }
}

template<typename T, typename E>
void Grafo<T, E>::floydWarshall()
{
    int V = nodos.size();
    std::vector<std::vector<int>> dist = matrizAdyacencia;

    // Algoritmo de Floyd-Warshall
    for (int k = 0; k < V; ++k)
    {
        for (int i = 0; i < V; ++i)
        {
            for (int j = 0; j < V; ++j)
            {
                if (dist[i][k] != std::numeric_limits<int>::max() &&
                        dist[k][j] != std::numeric_limits<int>::max())
                {
                    long long suma = static_cast<long long>(dist[i][k]) + dist[k][j];
                    if (suma < std::numeric_limits<int>::max())
                    {
                        dist[i][j] = std::min(dist[i][j], static_cast<int>(suma));
                    }
                }
            }
        }
    }

    // Calcular el ancho máximo necesario para los nombres
    size_t maxNombreLen = 0;
    for (const auto& nodo : nodos)
    {
        maxNombreLen = std::max(maxNombreLen, nodo.getNombre().length());
    }
    maxNombreLen = std::max(maxNombreLen, size_t(4)); // Mínimo 4 caracteres

    // Imprimir la matriz con formato mejorado
    std::cout << "\nMatriz de distancias minimas:\n\n";

    // Imprimir encabezado
     for (const auto& nodo : nodos) {
        std::cout << std::setw(6) << nodo.getNombre().substr(0, 5);
    }
    std::cout << '\n' << std::string((maxNombreLen + 2) + nodos.size() * 6, '-') << '\n';

    for (int i = 0; i < V; ++i) {
        std::cout << std::setw(maxNombreLen + 2) << nodos[i].getNombre();
        for (int j = 0; j < V; ++j) {
            if (dist[i][j] == std::numeric_limits<int>::max()) {
                std::cout << std::setw(6) << "∞";
            } else {
                std::cout << std::setw(6) << dist[i][j];
            }
        }
        std::cout << '\n';
    }
    std::cout << "\nLeyenda: ∞ = No hay camino directo entre los nodos\n";
}

template<typename T, typename E>
void Grafo<T,E>::planoGrafo()
{
    std::cout << "Lista de nodos disponibles:" << std::endl;
    std::cout << "-------------------------" << std::endl;
    for (const auto& nodo : nodos)
    {
        std::cout << "Nombre: " << nodo.getNombre()
                  << " | Edad: " << nodo.getEdad()
                  << " | Altura: " << nodo.getAltura()
                  << " | Popularidad: " << nodo.getPopularidad()
                  << std::endl;
    }
    std::cout << "-------------------------" << std::endl;
}

template<typename T, typename E>
void Grafo<T, E>::generarGrafo(const std::string& pathNodos, const std::string& pathAristas)
{
    std::ifstream fileNodos(pathNodos);
    std::ifstream fileAristas(pathAristas);
    std::string linea;

    if (!fileNodos.is_open() || !fileAristas.is_open())
    {
        throw std::runtime_error("Error al abrir el/los archivos: " + pathNodos + " " + pathAristas);
    }

    std::cout << "Leyendo archivo de nodos..." << std::endl;

    // Saltar la primera línea (cabecera) del archivo de nodos
    std::getline(fileNodos, linea);

    // Procesar el resto de las líneas
    while (std::getline(fileNodos, linea))
    {
        try
        {
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
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error("Error procesando nodo en línea: " + linea + ": " + e.what());
        }
    }
    fileNodos.close();

    std::cout << "Leyendo archivo de aristas..." << std::endl;

    // Saltar la primera línea (cabecera) del archivo de aristas
    std::getline(fileAristas, linea);

    while (std::getline(fileAristas, linea))
    {
        try
        {
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
        }
        catch (const std::exception& e)
        {
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

template<typename T, typename E>
void Grafo<T, E>::cargarComandos(const std::string& nombreArchivo)
{
    std::ifstream archivo(nombreArchivo);
    if (!archivo.is_open())
    {
        std::cerr << "No se pudo abrir el archivo de comandos: " << nombreArchivo << std::endl;
        return;
    }

    std::string linea;
    while (std::getline(archivo, linea))
    {
        std::istringstream iss(linea);
        std::string nombre, parametros, descripcion;

        if (std::getline(iss, nombre, '|') &&
                std::getline(iss, parametros, '|') &&
                std::getline(iss, descripcion, '|'))
        {
            Comando comando(nombre, parametros, descripcion);

            comandos.push_back(comando);
        }
    }

    archivo.close();
}

// Método para procesar comando -- Procesa un comando introducido por consolo por el usuario
template<typename T, typename E>
void Grafo<T, E>::procesarComando(const std::string& comando)
{
    std::istringstream iss(comando);
    std::string nombre;
    iss >> nombre;

    if (nombre == "ayuda")
    {
        mostrarAyuda();
    }
    else if (nombre == "generarGrafo")
    {
        std::string nombreArchivo;
        std::string nombreArchivo2;
        if (iss >> nombreArchivo >> nombreArchivo2)
        {
            generarGrafo(nombreArchivo, nombreArchivo2);
        }
        else
        {
            std::cerr << "\nError: Falta el nombre del archivo para cargar.\n" << std::endl;
        }
    }
    else if (nombre == "DFS")
    {
        std::string inicio;
        if (iss >> inicio)
        {
            DFS(inicio);
        }
        else
        {
            std::cerr << "\nError: Falta el inicio para hacer el recorrido.\n" << std::endl;
        }
    }
    else if (nombre == "BFS")
    {
        std::string inicio;
        if (iss >> inicio)
        {
            BFS(inicio);
        }
        else
        {
            std::cerr << "\nError: Falta el inicio para hacer el recorrido.\n" << std::endl;
        }
    }
    else if (nombre == "Dijkstra")
    {
        std::string inicio;
        if (iss >> inicio)
        {
            dijkstra(inicio);
        }
        else
        {
            std::cerr << "\nError: Falta el inicio para hacer el algoritmo.\n" << std::endl;
        }
    }
    else if (nombre == "Floyd-Warshall")
    {
        floydWarshall();
    }
    else if (nombre == "plano")
    {
        planoGrafo();
    }
    else if (nombre == "guardar")
    {
        std::string inicio;
        if (iss >> inicio)
        {
            dijkstra(inicio);
        }
        else
        {
            std::cerr << "\nError: Falta el inicio para hacer el algoritmo.\n" << std::endl;
        }
    }
    else if(nombre=="clear")
    {
        borrarPantalla();
    }
    else
    {
        std::cout << "\nComando no reconocido: " << comando << "\n" << std::endl;
    }
}

// Método para mostrar ayuda de los comandos -- Muestra lista de comandos disponibles y su descripción
template<typename T, typename E>
void Grafo<T, E>::mostrarAyuda()
{
    std::cout << std::left
              << std::setw(20) << "\nComando"
              << std::setw(35) << "Parametros"
              << "Descripcion" << std::endl;
    std::cout << std::string(120, '-') << std::endl;

    for (const auto& comando : comandos)
    {
        std::cout << std::left
                  << std::setw(20) << comando.getNombre()
                  << std::setw(35) << comando.getParametros()
                  << comando.getDescripcion() << std::endl;
    }
}

// Método para listar comandos -- Lista los nombres de los comandos disponibles
template<typename T, typename E>
void Grafo<T, E>::listarComandos()
{
    std::cout << "\nLista de comandos disponibles: \n" << std::endl;
    for (const auto& comando : comandos)
    {
        std::cout << comando.getNombre() << std::endl;
    }
}

// Método para borrar pantalla
template<typename T, typename E>
void Grafo<T, E>::borrarPantalla()
{

    system("cls");
}

template class Grafo<Personaje, Relacion>;

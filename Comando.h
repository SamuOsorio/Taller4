

#ifndef COMANDO_H
#define COMANDO_H

#include <string>

class Comando {
public:
    Comando(const std::string& nombre, const std::string& parametros, const std::string& descripcion);
    std::string getNombre() const;
    std::string getParametros() const;
    std::string getDescripcion() const;

private:
    std::string nombre;
    std::string parametros;
    std::string descripcion;
};

#endif






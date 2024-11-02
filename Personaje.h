#ifndef PERSONAJE_H
#define PERSONAJE_H

#include <string>

class Personaje{
private:
    std::string nombre;
    int edad;
    float altura;
    int popularidad;

public:
    Personaje(const std::string& nombre, int edad, float altura, int popularidad);

    std::string getNombre() const;
    int getEdad() const;
    float getAltura() const;
    int getPopularidad() const;

	void setNombre(std::string nombre);
    void setEdad(int edad);
    void setAltura(float altura);
    void setPopularidad(int popularidad);
};

#endif

#include "Personaje.h"

Personaje::Personaje(const std::string& nombre, int edad, float altura, int popularidad)
	: nombre(nombre), edad(edad), altura(altura), popularidad(popularidad){}
	
std::string Personaje::getNombre() const { return nombre; }
int Personaje::getEdad() const { return edad; }
float Personaje::getAltura() const { return altura; }
int Personaje::getPopularidad() const { return popularidad; }

void Personaje::setEdad(int edad) { this->edad = edad; }
void Personaje::setAltura(float altura) { this->altura = altura; }
void Personaje::setPopularidad(int popularidad) { this->popularidad = popularidad; }
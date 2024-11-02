#include "Relacion.h"

Relacion::Relacion(const std::string& source, const std::string& target, const std::string& type, int peso, int libro)
	:source(source), target(target), type(type), peso(peso), libro(libro){}

std::string Relacion::getSource() const {return source;}
std::string Relacion::getTarget() const {return target;}
std::string Relacion::getType() const {return type;};
int Relacion::getPeso() const {return peso;};
int Relacion::getLibro() const {return libro;};

void Relacion::setSource(std::string source) {this->source = source;}
void Relacion::setTarget(std::string target) {this->target = target;}
void Relacion::setType(std::string type) {this->type = type;}
void Relacion::setPeso(int peso) {this->peso = peso;}
void Relacion::setLibro(int libro) {this->libro = libro;}
